// SPDX-License-Identifier: GPL-3.0-or-later
//! Read QSP1 snapshots from the existing hardware owner. This module never
//! opens V4L2, changes a sensor, or depends on the debug server or browser.
use crate::gpu::{Buffer, Gpu};
use crate::renderer::{colour_range, image_barrier};
use anyhow::{ensure, Context, Result};
use ash::vk;
use serde::Deserialize;
use std::{
    collections::VecDeque,
    fs::{File, OpenOptions},
    io::{Cursor, Read},
    os::fd::AsRawFd,
    os::unix::fs::{MetadataExt, OpenOptionsExt},
    path::{Path, PathBuf},
    rc::Rc,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    thread,
    time::Duration,
};

const MAX_AGE_NS: u64 = 150_000_000;
const MAX_POSE_GAP_NS: u64 = 40_000_000;
const RADIAL_LUT_INTERVALS: usize = 1024;

fn radial_projection(k: [f32; 4], theta: f32) -> f32 {
    let t2 = theta * theta;
    theta * (1. + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))
}

fn radial_lut(k: [f32; 4]) -> Vec<f32> {
    (0..=RADIAL_LUT_INTERVALS)
        .map(|i| {
            // r/(r+z) spans the complete forward hemisphere smoothly. Unlike
            // r/z, this coordinate remains bounded at the edge of the fisheye.
            let t = i as f32 / RADIAL_LUT_INTERVALS as f32;
            radial_projection(k, t.atan2(1. - t))
        })
        .collect()
}

#[derive(Clone, Copy, Default, Deserialize, PartialEq)]
#[serde(rename_all = "kebab-case")]
enum MotionCompensation {
    #[default]
    None,
    DequeueRotation,
}

fn unit_quaternion(q: openxr::Quaternionf) -> Option<[f32; 4]> {
    let q = [q.x, q.y, q.z, q.w];
    let norm = q.iter().map(|x| x * x).sum::<f32>();
    (q.iter().all(|x| x.is_finite()) && (norm - 1.).abs() < 0.01)
        .then(|| q.map(|x| x / norm.sqrt()))
}

#[derive(Default)]
struct PoseHistory(VecDeque<(u64, [[f32; 4]; 2])>);

impl PoseHistory {
    fn observe(&mut self, flags: openxr::ViewStateFlags, views: &[openxr::View], stamp: u64) {
        let required =
            openxr::ViewStateFlags::ORIENTATION_VALID | openxr::ViewStateFlags::ORIENTATION_TRACKED;
        let poses = if flags.contains(required) && views.len() == 2 {
            unit_quaternion(views[0].pose.orientation)
                .zip(unit_quaternion(views[1].pose.orientation))
                .map(|(left, right)| [left, right])
        } else {
            None
        };
        let Some(poses) = poses else {
            self.0.clear();
            return;
        };
        if self
            .0
            .back()
            .is_some_and(|p| stamp <= p.0 || stamp - p.0 > MAX_POSE_GAP_NS)
        {
            self.0.clear();
        }
        if self.0.len() == 64 {
            self.0.pop_front();
        }
        self.0.push_back((stamp, poses));
    }

    // No extrapolation: a missing bracket or a tracking gap uses the original
    // projection. These are app-observed poses, not exposure-time measurements.
    fn at(&self, eye: usize, stamp: u64) -> Option<[f32; 4]> {
        let mut previous = None;
        for (time, poses) in &self.0 {
            if *time == stamp {
                return Some(poses[eye]);
            }
            if *time > stamp {
                let (before, q): (u64, [f32; 4]) = previous?;
                if time - before > MAX_POSE_GAP_NS {
                    return None;
                }
                let fraction = (stamp - before) as f32 / (time - before) as f32;
                let sign = if q.iter().zip(poses[eye]).map(|(a, b)| a * b).sum::<f32>() < 0. {
                    -1.
                } else {
                    1.
                };
                let result: [f32; 4] =
                    std::array::from_fn(|i| q[i] + fraction * (sign * poses[eye][i] - q[i]));
                let norm = result.iter().map(|x| x * x).sum::<f32>().sqrt();
                return (norm > 0.5).then(|| result.map(|x| x / norm));
            }
            previous = Some((*time, poses[eye]));
        }
        None
    }
}

// camera_from_view * inverse(world_from_capture_view) * world_from_current_view.
// The base OEM camera rotation and each eye's own source timestamp are retained.
fn compensated_rotation(
    base: [[f32; 3]; 3],
    capture: [f32; 4],
    current: [f32; 4],
) -> ([[f32; 3]; 3], f32) {
    let [ax, ay, az, aw] = [-capture[0], -capture[1], -capture[2], capture[3]];
    let [bx, by, bz, bw] = current;
    let [x, y, z, w] = [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ];
    let rotation = [
        [
            1. - 2. * (y * y + z * z),
            2. * (x * y - z * w),
            2. * (x * z + y * w),
        ],
        [
            2. * (x * y + z * w),
            1. - 2. * (x * x + z * z),
            2. * (y * z - x * w),
        ],
        [
            2. * (x * z - y * w),
            2. * (y * z + x * w),
            1. - 2. * (x * x + y * y),
        ],
    ];
    (
        std::array::from_fn(|row| {
            std::array::from_fn(|col| (0..3).map(|k| base[row][k] * rotation[k][col]).sum())
        }),
        2. * (x * x + y * y + z * z).sqrt().atan2(w.abs()).to_degrees(),
    )
}
fn now_ns() -> u64 {
    let mut t = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t) } != 0 {
        return 0;
    }
    t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64
}
#[derive(Deserialize)]
struct EyeCalibration {
    slot: u32,
    intrinsics: [f32; 4],
    distortion: [f32; 4],
    camera_from_view: [[f32; 3]; 3],
}
#[derive(Deserialize)]
struct Config {
    schema: String,
    version: u32,
    directory: PathBuf,
    source_size: [u32; 2],
    eyes: [EyeCalibration; 2],
    #[serde(default = "unit_gain")]
    colour_gain: [f32; 3],
    black_level_10bit: u16,
    white_balance: String,
    #[serde(default)]
    motion_compensation: MotionCompensation,
}
fn load_config() -> Result<Config> {
    let Some(path) = std::env::var_os("XROS_PASSTHROUGH_CONFIG") else {
        anyhow::bail!("XROS_PASSTHROUGH_CONFIG is not configured");
    };
    let config: Config =
        serde_json::from_reader(File::open(path).context("open passthrough configuration")?)?;
    ensure!(
        config.schema == "org.openkylin.xr.passthrough"
            && config.version == 2
            && matches!(config.source_size, [1928, 1088] | [1728, 1728])
            && config.directory.is_absolute()
            && config.eyes[0].slot == 4
            && config.eyes[1].slot == 5,
        "unsupported passthrough configuration"
    );
    ensure!(
        config.black_level_10bit < 256
            && matches!(config.white_balance.as_str(), "gray-world" | "none"),
        "unsupported RAW colour controls"
    );
    for eye in &config.eyes {
        ensure!(
            eye.intrinsics
                .iter()
                .chain(eye.distortion.iter())
                .all(|x| x.is_finite())
                && eye.intrinsics[0] > 0.
                && eye.intrinsics[1] > 0.,
            "invalid camera intrinsics"
        );
    }
    ensure!(
        config
            .colour_gain
            .iter()
            .all(|x| x.is_finite() && *x > 0. && *x <= 16.),
        "invalid colour gain"
    );
    for eye in &config.eyes {
        ensure!(
            eye.camera_from_view.iter().flatten().all(|x| x.is_finite()),
            "invalid camera rotation"
        );
    }
    Ok(config)
}
pub fn validate_config() -> Result<()> {
    load_config().map(|_| ())
}
fn unit_gain() -> [f32; 3] {
    [1.; 3]
}
struct Frame {
    generation: u64,
    sequence: u64,
    received: u64,
    stride: usize,
    raw: Vec<u8>,
}
struct Stereo {
    frames: [Arc<Frame>; 2],
    gains: [f32; 3],
}
#[derive(Default)]
struct WorkerStats {
    reads: [AtomicU64; 2],
    cpu_ns: [AtomicU64; 2],
}
// Multiple visible layers may request the owner's two IMX streams. Closing
// the directory descriptor releases this reader's request even after a crash.
fn request_stereo(directory: &Path) -> Option<File> {
    let file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(directory)
        .ok()?;
    (unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH | libc::LOCK_NB) } == 0).then_some(file)
}

fn snapshot(
    directory: &Path,
    slot: u32,
    previous: Option<&Frame>,
    width: usize,
    height: usize,
) -> Option<Frame> {
    let started = now_ns();
    let max_stride = (width * 2 * 5 / 4).div_ceil(256) * 256;
    // Inspect only the small headers first. Keep a shared lock solely on the
    // newest candidate; no full-image decoding is performed under that lock.
    let mut selected: Option<(File, u64, u64, u64, usize, usize)> = None;
    for index in 0..3 {
        let mut f = match OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(directory.join(format!("slot{slot}.{index}.raw")))
        {
            Ok(f) => f,
            Err(_) => continue,
        };
        let meta = match f.metadata() {
            Ok(m) => m,
            Err(_) => continue,
        };
        if !meta.is_file()
            || meta.uid() != 0
            || meta.mode() & 0o022 != 0
            || unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_SH | libc::LOCK_NB) } != 0
        {
            continue;
        }
        let mut header = [0u8; 64];
        if f.read_exact(&mut header).is_err() {
            continue;
        }
        let word = |at| u32::from_le_bytes(header[at..at + 4].try_into().unwrap());
        let wide = |at| u64::from_le_bytes(header[at..at + 8].try_into().unwrap());
        let (stride, bytes) = (word(16) as usize, word(28) as usize);
        let (generation, sequence, received) = (wide(32), wide(40), wide(48));
        if word(0) != 0x31505351
            || word(4) != 1
            || word(8) as usize != width * 2
            || word(12) as usize != height * 2
            || word(20) != u32::from_le_bytes(*b"pRAA")
            || word(24) != slot
            || !(width * 2 * 5 / 4..=max_stride).contains(&stride)
            || bytes != stride * height * 2
            || meta.len() != 64 + bytes as u64
            || received > started
            || started.saturating_sub(received) >= MAX_AGE_NS
            || previous.is_some_and(|p| p.generation == generation && p.sequence >= sequence)
            || selected.as_ref().is_some_and(|p| p.3 >= received)
        {
            continue;
        }
        selected = Some((f, generation, sequence, received, stride, bytes));
    }
    let (mut f, generation, sequence, received, stride, bytes) = selected?;
    let mut raw = vec![0u8; bytes];
    f.read_exact(&mut raw).ok()?;
    Some(Frame {
        generation,
        sequence,
        received,
        stride,
        raw,
    })
}

fn accumulate_colour_samples(
    frame: &Frame,
    width: usize,
    height: usize,
    black: u16,
    sums: &mut [f64; 3],
    count: &mut u32,
) {
    let sample = |x: usize, y: usize| -> u16 {
        let at = y * frame.stride + (x / 4) * 5;
        u16::from(frame.raw[at + x % 4]) * 4 + u16::from((frame.raw[at + 4] >> (2 * (x % 4))) & 3)
    };
    // Sparse grey-world statistics only. The GPU unpacks every output pixel.
    for y in (0..height).step_by(8) {
        for x in (0..width).step_by(8) {
            let values = [
                sample(x * 2, y * 2),
                (sample(x * 2 + 1, y * 2) + sample(x * 2, y * 2 + 1)) / 2,
                sample(x * 2 + 1, y * 2 + 1),
            ];
            let rgb = values.map(|v| {
                ((u32::from(v.saturating_sub(black)) * 255) / u32::from(1023 - black)).min(255)
                    as u8
            });
            if rgb.iter().all(|v| *v > 6 && *v < 225) {
                for c in 0..3 {
                    sums[c] += f64::from(rgb[c]);
                }
                *count += 1;
            }
        }
    }
}

pub struct Passthrough {
    gpu: Rc<Gpu>,
    image: vk::Image,
    memory: vk::DeviceMemory,
    view: vk::ImageView,
    sampler: vk::Sampler,
    staging: Buffer,
    _calibration: Buffer,
    latest: Arc<Mutex<Option<Stereo>>>,
    stop: Arc<AtomicBool>,
    visible: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<()>>,
    received: [u64; 2],
    initialized: bool,
    uploaded_frames: u64,
    width: usize,
    height: usize,
    coefficients: Vec<f32>,
    gains: [f32; 3],
    white_balance: String,
    plane_bytes: usize,
    black: u32,
    compute_layout: vk::PipelineLayout,
    compute_pipeline: vk::Pipeline,
    compute_set_layout: vk::DescriptorSetLayout,
    compute_pool: vk::DescriptorPool,
    compute_set: vk::DescriptorSet,
    uploaded_ids: [(u64, u64); 2],
    uploaded_distinct: [u64; 2],
    submitted_ids: [(u64, u64); 2],
    submitted_distinct: [u64; 2],
    submitted_render_frames: u64,
    strides: [u32; 2],
    stats: Arc<WorkerStats>,
    last_report_ns: u64,
    motion_compensation: MotionCompensation,
    pose_history: PoseHistory,
    camera_from_view: [[[f32; 3]; 3]; 2],
    compensated: [bool; 2],
    correction_degrees: [f32; 2],
    compensated_render_frames: u64,
}
impl Passthrough {
    pub fn from_environment(gpu: Rc<Gpu>, descriptor: vk::DescriptorSet) -> Result<Option<Self>> {
        let config = load_config()?;
        let width = config.source_size[0] as usize / 2;
        let height = config.source_size[1] as usize / 2;
        let mut coefficients = Vec::<f32>::new();
        for eye in &config.eyes {
            coefficients.extend(eye.intrinsics);
            coefficients.extend(eye.distortion);
            for row in eye.camera_from_view {
                ensure!(row.iter().all(|x| x.is_finite()), "invalid camera rotation");
                coefficients.extend([row[0], row[1], row[2], 0.]);
            }
        }
        ensure!(
            config
                .colour_gain
                .iter()
                .all(|x| x.is_finite() && *x > 0. && *x <= 16.),
            "invalid colour gain"
        );
        coefficients.extend([
            config.colour_gain[0],
            config.colour_gain[1],
            config.colour_gain[2],
            0.,
        ]);
        coefficients.extend([width as f32, height as f32, 0., 0.]);
        let row_capacity = (width * 2 * 5 / 4).div_ceil(256) * 256;
        let plane_bytes = row_capacity * height * 2;
        let staging = Buffer::host(
            gpu.clone(),
            (plane_bytes * 2) as u64,
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )?;
        staging.write(&vec![0; plane_bytes * 2])?;
        let mut calibration_data = coefficients.clone();
        for eye in &config.eyes {
            for value in radial_lut(eye.distortion) {
                calibration_data.extend([value, 0., 0., 0.]);
            }
        }
        let calibration = Buffer::host(
            gpu.clone(),
            (calibration_data.len() * 4) as u64,
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )?;
        calibration.write(unsafe {
            std::slice::from_raw_parts(
                calibration_data.as_ptr() as *const u8,
                calibration_data.len() * 4,
            )
        })?;
        let mut p = Self {
            gpu,
            image: vk::Image::null(),
            memory: vk::DeviceMemory::null(),
            view: vk::ImageView::null(),
            sampler: vk::Sampler::null(),
            staging,
            _calibration: calibration,
            latest: Arc::new(Mutex::new(None)),
            stop: Arc::new(AtomicBool::new(false)),
            visible: Arc::new(AtomicBool::new(true)),
            worker: None,
            received: [0; 2],
            initialized: false,
            uploaded_frames: 0,
            width,
            height,
            coefficients,
            gains: config.colour_gain,
            white_balance: config.white_balance.clone(),
            plane_bytes,
            black: u32::from(config.black_level_10bit),
            compute_layout: vk::PipelineLayout::null(),
            compute_pipeline: vk::Pipeline::null(),
            compute_set_layout: vk::DescriptorSetLayout::null(),
            compute_pool: vk::DescriptorPool::null(),
            compute_set: vk::DescriptorSet::null(),
            uploaded_ids: [(0, 0); 2],
            uploaded_distinct: [0; 2],
            submitted_ids: [(0, 0); 2],
            submitted_distinct: [0; 2],
            submitted_render_frames: 0,
            strides: [(width * 2 * 5 / 4) as u32; 2],
            stats: Arc::new(WorkerStats::default()),
            last_report_ns: 0,
            motion_compensation: config.motion_compensation,
            pose_history: PoseHistory::default(),
            camera_from_view: config.eyes.each_ref().map(|eye| eye.camera_from_view),
            compensated: [false; 2],
            correction_degrees: [0.; 2],
            compensated_render_frames: 0,
        };
        unsafe {
            let d = &p.gpu.device;
            p.image = d.create_image(
                &vk::ImageCreateInfo::default()
                    .image_type(vk::ImageType::TYPE_2D)
                    .format(vk::Format::R8G8B8A8_UNORM)
                    .extent(vk::Extent3D {
                        width: (width * 2) as u32,
                        height: height as u32,
                        depth: 1,
                    })
                    .mip_levels(1)
                    .array_layers(1)
                    .samples(vk::SampleCountFlags::TYPE_1)
                    .tiling(vk::ImageTiling::OPTIMAL)
                    .usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::STORAGE),
                None,
            )?;
            let req = d.get_image_memory_requirements(p.image);
            p.memory = d.allocate_memory(
                &vk::MemoryAllocateInfo::default()
                    .allocation_size(req.size)
                    .memory_type_index(p.gpu.memory_type(
                        req.memory_type_bits,
                        vk::MemoryPropertyFlags::DEVICE_LOCAL,
                    )?),
                None,
            )?;
            d.bind_image_memory(p.image, p.memory, 0)?;
            p.view = d.create_image_view(
                &vk::ImageViewCreateInfo::default()
                    .image(p.image)
                    .view_type(vk::ImageViewType::TYPE_2D)
                    .format(vk::Format::R8G8B8A8_UNORM)
                    .subresource_range(colour_range()),
                None,
            )?;
            p.sampler = d.create_sampler(
                &vk::SamplerCreateInfo::default()
                    .mag_filter(vk::Filter::LINEAR)
                    .min_filter(vk::Filter::LINEAR)
                    .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                    .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE),
                None,
            )?;
            d.update_descriptor_sets(
                &[
                    vk::WriteDescriptorSet::default()
                        .dst_set(descriptor)
                        .dst_binding(3)
                        .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                        .image_info(&[vk::DescriptorImageInfo::default()
                            .sampler(p.sampler)
                            .image_view(p.view)
                            .image_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)]),
                    vk::WriteDescriptorSet::default()
                        .dst_set(descriptor)
                        .dst_binding(4)
                        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                        .buffer_info(&[vk::DescriptorBufferInfo::default()
                            .buffer(p._calibration.handle)
                            .range(p._calibration.size)]),
                ],
                &[],
            );
        }
        p.create_decoder()?;
        let latest = p.latest.clone();
        let stop = p.stop.clone();
        let stats = p.stats.clone();
        let visible = p.visible.clone();
        p.worker = Some(thread::spawn(move || {
            let mut frames: [Option<Arc<Frame>>; 2] = [None, None];
            let mut gains = config.colour_gain;
            let mut next_balance = 0;
            let mut request = None;
            while !stop.load(Ordering::Acquire) {
                if !visible.load(Ordering::Acquire) {
                    if request.take().is_some() {
                        frames = [None, None];
                        if let Ok(mut target) = latest.lock() {
                            *target = None;
                        }
                    }
                    thread::park();
                    continue;
                }
                if request.is_none() {
                    request = request_stereo(&config.directory);
                    if request.is_none() {
                        thread::sleep(std::time::Duration::from_millis(30));
                        continue;
                    }
                }
                let mut changed = false;
                for eye in 0..2 {
                    let started = now_ns();
                    if let Some(f) = snapshot(
                        &config.directory,
                        config.eyes[eye].slot,
                        frames[eye].as_deref(),
                        width,
                        height,
                    ) {
                        stats.reads[eye].fetch_add(1, Ordering::Relaxed);
                        stats.cpu_ns[eye]
                            .fetch_add(now_ns().saturating_sub(started), Ordering::Relaxed);
                        frames[eye] = Some(Arc::new(f));
                        changed = true;
                    }
                }
                if changed
                    && frames.iter().all(|f| {
                        f.as_ref()
                            .is_some_and(|f| now_ns().saturating_sub(f.received) < MAX_AGE_NS)
                    })
                {
                    let now = now_ns();
                    // Keep adaptation time stable as source frame rate increases.
                    if config.white_balance == "gray-world" && now >= next_balance {
                        next_balance = now + 33_333_333;
                        let mut sums = [0.; 3];
                        let mut count = 0;
                        for (eye, frame) in frames.iter().enumerate() {
                            let started = now_ns();
                            accumulate_colour_samples(
                                frame.as_ref().unwrap(),
                                width,
                                height,
                                config.black_level_10bit,
                                &mut sums,
                                &mut count,
                            );
                            stats.cpu_ns[eye]
                                .fetch_add(now_ns().saturating_sub(started), Ordering::Relaxed);
                        }
                        if count > 100 && sums.iter().all(|v| *v > 0.) {
                            for c in 0..3 {
                                let target = (sums[1] / sums[c]).clamp(0.5, 4.) as f32
                                    * config.colour_gain[c];
                                gains[c] += 0.05 * (target - gains[c]);
                            }
                        }
                    }
                    if let Ok(mut target) = latest.lock() {
                        *target = Some(Stereo {
                            frames: [
                                frames[0].as_ref().unwrap().clone(),
                                frames[1].as_ref().unwrap().clone(),
                            ],
                            gains,
                        });
                    }
                }
                thread::sleep(Duration::from_millis(2));
            }
        }));
        eprintln!(
            "XR_PASSTHROUGH=enabled SOURCE=QSP1 SIZE={}x{} STEREO=left4,right5 DECODE=GPU-RAW10 COLOUR=black-subtracted-linear GEOMETRY=OEM-camera-body-rotation",
            width, height
        );
        Ok(Some(p))
    }
    fn create_decoder(&mut self) -> Result<()> {
        unsafe {
            let d = &self.gpu.device;
            self.compute_set_layout = d.create_descriptor_set_layout(
                &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                    vk::DescriptorSetLayoutBinding::default()
                        .binding(0)
                        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                        .descriptor_count(1)
                        .stage_flags(vk::ShaderStageFlags::COMPUTE),
                    vk::DescriptorSetLayoutBinding::default()
                        .binding(1)
                        .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
                        .descriptor_count(1)
                        .stage_flags(vk::ShaderStageFlags::COMPUTE),
                ]),
                None,
            )?;
            self.compute_pool = d.create_descriptor_pool(
                &vk::DescriptorPoolCreateInfo::default()
                    .max_sets(1)
                    .pool_sizes(&[
                        vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                        },
                        vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::STORAGE_IMAGE,
                            descriptor_count: 1,
                        },
                    ]),
                None,
            )?;
            self.compute_set = d.allocate_descriptor_sets(
                &vk::DescriptorSetAllocateInfo::default()
                    .descriptor_pool(self.compute_pool)
                    .set_layouts(&[self.compute_set_layout]),
            )?[0];
            d.update_descriptor_sets(
                &[
                    vk::WriteDescriptorSet::default()
                        .dst_set(self.compute_set)
                        .dst_binding(0)
                        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                        .buffer_info(&[vk::DescriptorBufferInfo::default()
                            .buffer(self.staging.handle)
                            .range(self.staging.size)]),
                    vk::WriteDescriptorSet::default()
                        .dst_set(self.compute_set)
                        .dst_binding(1)
                        .descriptor_type(vk::DescriptorType::STORAGE_IMAGE)
                        .image_info(&[vk::DescriptorImageInfo::default()
                            .image_view(self.view)
                            .image_layout(vk::ImageLayout::GENERAL)]),
                ],
                &[],
            );
            self.compute_layout = d.create_pipeline_layout(
                &vk::PipelineLayoutCreateInfo::default()
                    .set_layouts(&[self.compute_set_layout])
                    .push_constant_ranges(&[vk::PushConstantRange {
                        stage_flags: vk::ShaderStageFlags::COMPUTE,
                        offset: 0,
                        size: 28,
                    }]),
                None,
            )?;
            let code = ash::util::read_spv(&mut Cursor::new(include_bytes!(concat!(
                env!("OUT_DIR"),
                "/passthrough.comp.spv"
            ))))?;
            let shader =
                d.create_shader_module(&vk::ShaderModuleCreateInfo::default().code(&code), None)?;
            let result = d.create_compute_pipelines(
                vk::PipelineCache::null(),
                &[vk::ComputePipelineCreateInfo::default()
                    .layout(self.compute_layout)
                    .stage(
                        vk::PipelineShaderStageCreateInfo::default()
                            .stage(vk::ShaderStageFlags::COMPUTE)
                            .module(shader)
                            .name(c"main"),
                    )],
                None,
            );
            d.destroy_shader_module(shader, None);
            match result {
                Ok(p) => self.compute_pipeline = p[0],
                Err((partial, error)) => {
                    for pipeline in partial {
                        d.destroy_pipeline(pipeline, None);
                    }
                    return Err(error.into());
                }
            }
        }
        Ok(())
    }
    pub fn status(&self) -> serde_json::Value {
        let reads = self
            .stats
            .reads
            .each_ref()
            .map(|x| x.load(Ordering::Relaxed));
        serde_json::json!({"source":"QSP1","decode":"gpu-raw10","reported_monotonic_ns":now_ns(),"size_per_eye":[self.width,self.height],"slots_left_right":[4,5],"white_balance":self.white_balance,"colour_gain":self.gains,
            "active":self.active(),"uploaded_frames":self.uploaded_frames,"uploaded_distinct_source_frames":self.uploaded_distinct,"submitted_distinct_source_frames":self.submitted_distinct,"submitted_render_frames":self.submitted_render_frames,
            "source_generation":self.uploaded_ids.map(|x|x.0),"source_sequence":self.uploaded_ids.map(|x|x.1),"raw_reads":reads,"average_raw_read_cpu_ms":std::array::from_fn::<_,2,_>(|i|if reads[i]==0 {0.}else{self.stats.cpu_ns[i].load(Ordering::Relaxed) as f64/reads[i] as f64/1e6}),
            "received_monotonic_ns":self.received,"age_ms":self.received.map(|t|if t==0 {None}else{Some(now_ns().saturating_sub(t) as f64/1e6)}),"pair_dq_delta_ms":self.received[0].abs_diff(self.received[1]) as f64/1e6,
            "motion_compensation":if self.motion_compensation == MotionCompensation::DequeueRotation {"dequeue-rotation"}else{"none"},
            "motion_compensated_eyes":self.compensated,"motion_correction_degrees":self.correction_degrees,"motion_compensated_render_frames":self.compensated_render_frames,
            "motion_time_basis":"QSP1 dequeue CLOCK_MONOTONIC and runtime-located OpenXR orientation; not exposure-time pose",
            "fisheye_projection":"oem-radial-lut-1024","fisheye_lut_intervals":RADIAL_LUT_INTERVALS,
            "exposure_sync_verified":false,"physical_extrinsics_verified":false,"colour_calibrated":false})
    }
    pub fn set_visible(&self, visible: bool) {
        if self.visible.load(Ordering::Relaxed) != visible {
            self.visible.store(visible, Ordering::Release);
            if visible {
                if let Some(worker) = &self.worker {
                    worker.thread().unpark();
                }
            }
        }
    }
    pub fn active(&self) -> bool {
        self.visible.load(Ordering::Relaxed)
            && self.initialized
            && self
                .received
                .iter()
                .all(|t| *t > 0 && now_ns().saturating_sub(*t) < MAX_AGE_NS)
    }
    pub fn submitted(&mut self, active: bool) {
        if active {
            self.submitted_render_frames += 1;
            if self.compensated.iter().all(|x| *x) {
                self.compensated_render_frames += 1;
            }
            for i in 0..2 {
                if self.submitted_ids[i] != self.uploaded_ids[i] {
                    self.submitted_distinct[i] += 1;
                    self.submitted_ids[i] = self.uploaded_ids[i];
                }
            }
        }
        let now = now_ns();
        if now.saturating_sub(self.last_report_ns) >= 1_000_000_000 {
            self.last_report_ns = now;
            let mut status = self.status();
            status["render_active"] = active.into();
            eprintln!("XR_PASSTHROUGH_STATS {}", status);
        }
    }
    pub fn observe_views(
        &mut self,
        flags: openxr::ViewStateFlags,
        views: &[openxr::View],
        stamp: u64,
    ) {
        if self.motion_compensation == MotionCompensation::DequeueRotation {
            self.pose_history.observe(flags, views, stamp);
        }
    }

    pub fn reset_pose_history(&mut self) {
        self.pose_history.0.clear();
    }

    pub fn update_projection(&mut self, views: &[openxr::View]) -> Result<()> {
        self.compensated = [false; 2];
        self.correction_degrees = [0.; 2];
        if self.motion_compensation == MotionCompensation::DequeueRotation {
            for eye in 0..2 {
                let base = self.camera_from_view[eye];
                let correction = self
                    .pose_history
                    .at(eye, self.received[eye])
                    .zip(
                        views
                            .get(eye)
                            .and_then(|v| unit_quaternion(v.pose.orientation)),
                    )
                    .map(|(capture, current)| compensated_rotation(base, capture, current));
                let matrix = if let Some((matrix, degrees)) = correction {
                    self.compensated[eye] = true;
                    self.correction_degrees[eye] = degrees;
                    matrix
                } else {
                    base
                };
                for (row, values) in matrix.iter().enumerate() {
                    let offset = eye * 20 + 8 + row * 4;
                    self.coefficients[offset..offset + 3].copy_from_slice(values);
                }
            }
        }
        self._calibration.write(unsafe {
            std::slice::from_raw_parts(
                self.coefficients.as_ptr() as *const u8,
                self.coefficients.len() * 4,
            )
        })?;
        Ok(())
    }

    pub fn prepare(&mut self, cmd: vk::CommandBuffer) -> Result<()> {
        if !self.visible.load(Ordering::Relaxed) && self.initialized {
            return Ok(());
        }
        let next = match self.latest.try_lock() {
            Ok(mut f) => f.take(),
            Err(_) => None,
        };
        let mut mask = if self.initialized { 0u32 } else { 3 };
        if let Some(stereo) = next {
            for (i, frame) in stereo.frames.iter().enumerate() {
                let id = (frame.generation, frame.sequence);
                if id != self.uploaded_ids[i] {
                    ensure!(
                        frame.raw.len() <= self.plane_bytes,
                        "RAW plane exceeds allocated capacity"
                    );
                    self.staging.write_at(i * self.plane_bytes, &frame.raw)?;
                    self.strides[i] = frame.stride as u32;
                    self.received[i] = frame.received;
                    self.uploaded_ids[i] = id;
                    self.uploaded_distinct[i] += 1;
                    mask |= 1 << i;
                }
            }
            self.gains = stereo.gains;
            self.coefficients[40..43].copy_from_slice(&stereo.gains);
        }
        if mask != 0 {
            unsafe {
                let d = &self.gpu.device;
                let (old, source, stage) = if self.initialized {
                    (
                        vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                        vk::AccessFlags::SHADER_READ,
                        vk::PipelineStageFlags::FRAGMENT_SHADER,
                    )
                } else {
                    (
                        vk::ImageLayout::UNDEFINED,
                        vk::AccessFlags::empty(),
                        vk::PipelineStageFlags::TOP_OF_PIPE,
                    )
                };
                d.cmd_pipeline_barrier(
                    cmd,
                    stage,
                    vk::PipelineStageFlags::COMPUTE_SHADER,
                    vk::DependencyFlags::empty(),
                    &[],
                    &[],
                    &[image_barrier(
                        self.image,
                        old,
                        vk::ImageLayout::GENERAL,
                        source,
                        vk::AccessFlags::SHADER_WRITE,
                    )],
                );
                d.cmd_bind_pipeline(cmd, vk::PipelineBindPoint::COMPUTE, self.compute_pipeline);
                d.cmd_bind_descriptor_sets(
                    cmd,
                    vk::PipelineBindPoint::COMPUTE,
                    self.compute_layout,
                    0,
                    &[self.compute_set],
                    &[],
                );
                let params = [
                    self.width as u32,
                    self.height as u32,
                    self.plane_bytes as u32,
                    self.strides[0],
                    self.strides[1],
                    self.black,
                    mask,
                ];
                d.cmd_push_constants(
                    cmd,
                    self.compute_layout,
                    vk::ShaderStageFlags::COMPUTE,
                    0,
                    std::slice::from_raw_parts(params.as_ptr() as *const u8, 28),
                );
                d.cmd_dispatch(
                    cmd,
                    (self.width as u32 * 2).div_ceil(8),
                    (self.height as u32).div_ceil(8),
                    1,
                );
                d.cmd_pipeline_barrier(
                    cmd,
                    vk::PipelineStageFlags::COMPUTE_SHADER,
                    vk::PipelineStageFlags::FRAGMENT_SHADER,
                    vk::DependencyFlags::empty(),
                    &[],
                    &[],
                    &[image_barrier(
                        self.image,
                        vk::ImageLayout::GENERAL,
                        vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                        vk::AccessFlags::SHADER_WRITE,
                        vk::AccessFlags::SHADER_READ,
                    )],
                );
            }
            self.initialized = true;
            if self.received.iter().all(|t| *t > 0) {
                self.uploaded_frames += 1;
            }
        }
        Ok(())
    }
}

impl Drop for Passthrough {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            let _ = worker.join();
        }
        unsafe {
            let d = &self.gpu.device;
            d.destroy_pipeline(self.compute_pipeline, None);
            d.destroy_pipeline_layout(self.compute_layout, None);
            d.destroy_descriptor_pool(self.compute_pool, None);
            d.destroy_descriptor_set_layout(self.compute_set_layout, None);
            d.destroy_sampler(self.sampler, None);
            d.destroy_image_view(self.view, None);
            d.destroy_image(self.image, None);
            d.free_memory(self.memory, None);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radial_lut_interpolation_preserves_the_analytic_fisheye_projection() {
        let k = [0.1, -0.025, 0.004, -0.0001];
        let lut = radial_lut(k);
        let mut worst_error = 0_f32;
        for i in 0..=65536 {
            let t = i as f32 / 65536.;
            let x = t * RADIAL_LUT_INTERVALS as f32;
            let lo = (x as usize).min(RADIAL_LUT_INTERVALS - 1);
            let approximate = lut[lo] + (lut[lo + 1] - lut[lo]) * (x - lo as f32);
            let analytic = radial_projection(k, t.atan2(1. - t));
            worst_error = worst_error.max((approximate - analytic).abs());
        }
        // Even at 2048 px focal length the interpolation stays below .02 px.
        assert!(worst_error * 2048. < 0.02, "{} pixels", worst_error * 2048.);
    }

    #[test]
    fn dequeue_rotation_keeps_oem_axes_and_rotates_current_ray_into_capture() {
        let base = [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]];
        let identity = [0., 0., 0., 1.];
        let half = std::f32::consts::FRAC_PI_4;
        let yaw = [0., half.sin(), 0., half.cos()];
        let (unchanged, degrees) = compensated_rotation(base, yaw, yaw.map(|v| -v));
        for row in 0..3 {
            for col in 0..3 {
                assert!((unchanged[row][col] - base[row][col]).abs() < 0.00001);
            }
        }
        assert!(degrees < 0.01);
        let (matrix, degrees) = compensated_rotation(base, identity, yaw);
        let forward: [f32; 3] = std::array::from_fn(|row| -matrix[row][2]);
        assert!((forward[0] + 1.).abs() < 0.00001);
        assert!(forward[1].abs() < 0.00001 && forward[2].abs() < 0.00001);
        assert!((degrees - 90.).abs() < 0.01);
    }

    #[test]
    fn dequeue_rotation_history_requires_a_valid_same_epoch_bracket() {
        let flags =
            openxr::ViewStateFlags::ORIENTATION_VALID | openxr::ViewStateFlags::ORIENTATION_TRACKED;
        let mut history = PoseHistory::default();
        let mut views = [openxr::View::default(); 2];
        for v in &mut views {
            v.pose.orientation.w = 1.;
        }
        history.observe(flags, &views, 10_000_000);
        for v in &mut views {
            let half = 10_f32.to_radians();
            v.pose.orientation = openxr::Quaternionf {
                x: 0.,
                y: half.sin(),
                z: 0.,
                w: half.cos(),
            };
        }
        history.observe(flags, &views, 30_000_000);
        let q = history.at(0, 20_000_000).unwrap();
        assert!((q[1] - 5_f32.to_radians().sin()).abs() < 0.00001);
        assert!(history.at(0, 9_000_000).is_none());
        assert!(history.at(1, 31_000_000).is_none());
        history.observe(flags, &views, 100_000_000);
        assert!(history.at(0, 20_000_000).is_none());
        history.observe(openxr::ViewStateFlags::EMPTY, &views, 101_000_000);
        assert!(history.at(0, 100_000_000).is_none());
    }
}
