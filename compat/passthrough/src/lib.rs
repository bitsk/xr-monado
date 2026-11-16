// SPDX-License-Identifier: GPL-3.0-or-later
//! Private ABI used only by the co-built OpenXR runtime layer.
#![allow(unsafe_op_in_unsafe_fn)]
mod camera;
mod gpu;
mod renderer;
use ash::vk::{self, Handle};
use renderer::Renderer;
use std::{ffi::c_void, path::PathBuf, rc::Rc};

#[repr(C)]
pub struct Eye {
    orientation: [f32; 4],
    position: [f32; 3],
    fov: [f32; 4],
}
impl Eye {
    fn view(&self) -> openxr::View {
        openxr::View {
            pose: openxr::Posef {
                orientation: openxr::Quaternionf {
                    x: self.orientation[0],
                    y: self.orientation[1],
                    z: self.orientation[2],
                    w: self.orientation[3],
                },
                position: openxr::Vector3f {
                    x: self.position[0],
                    y: self.position[1],
                    z: self.position[2],
                },
            },
            fov: openxr::Fovf {
                angle_left: self.fov[0],
                angle_right: self.fov[1],
                angle_up: self.fov[2],
                angle_down: self.fov[3],
            },
        }
    }
}
struct Context {
    renderer: Renderer,
    last_status: u64,
    status_path: Option<PathBuf>,
}
fn now_ns() -> u64 {
    unsafe {
        let mut t = std::mem::zeroed();
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut t);
        t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64
    }
}
impl Context {
    fn status(&mut self, available: bool, submitted: bool, force: bool) {
        let now = now_ns();
        if !force && now.saturating_sub(self.last_status) < 500_000_000 {
            return;
        }
        self.last_status = now;
        let Some(path) = &self.status_path else {
            return;
        };
        let camera = self.renderer.camera.as_ref().unwrap();
        let value = serde_json::json!({"schema":1,"provider":"xros-openxr-passthrough","available":available,
            "reported_monotonic_ns":now,"pid":std::process::id(),
            "session_token":std::env::var("XROS_SESSION_TOKEN").unwrap_or_default(),
            "render_submitted":submitted,"camera":camera.status()});
        // The session owner supplies a private directory; never write a global file.
        let temp = path.with_extension(format!("{}.tmp", std::process::id()));
        if let Ok(data) = serde_json::to_vec(&value) {
            if std::fs::write(&temp, data).is_ok() {
                let _ = std::fs::rename(&temp, path);
            }
        }
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn xros_camera_config_valid() -> bool {
    match camera::validate_config() {
        Ok(()) => true,
        Err(e) => {
            eprintln!("XROS_PASSTHROUGH_CONFIG_ERROR {e:#}");
            false
        }
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_create(
    gpa: vk::PFN_vkGetInstanceProcAddr,
    instance: u64,
    physical: u64,
    device: u64,
    family: u32,
    index: u32,
    format: i32,
) -> *mut c_void {
    let run = || -> anyhow::Result<Context> {
        let static_fn = ash::StaticFn {
            get_instance_proc_addr: gpa,
        };
        let instance = ash::Instance::load(&static_fn, vk::Instance::from_raw(instance));
        let physical = vk::PhysicalDevice::from_raw(physical);
        let families = instance.get_physical_device_queue_family_properties(physical);
        anyhow::ensure!(
            families
                .get(family as usize)
                .is_some_and(|f| index < f.queue_count
                    && f.queue_flags
                        .contains(vk::QueueFlags::GRAPHICS | vk::QueueFlags::COMPUTE)),
            "requires graphics and compute queue"
        );
        let device = ash::Device::load(instance.fp_v1_0(), vk::Device::from_raw(device));
        let gpu = Rc::new(gpu::Gpu {
            queue: device.get_device_queue(family, index),
            device,
            memory: instance.get_physical_device_memory_properties(physical),
            family,
        });
        Ok(Context {
            renderer: Renderer::new(gpu, vk::Format::from_raw(format))?,
            last_status: 0,
            status_path: std::env::var_os("XR_RUNTIME_PASSTHROUGH_STATUS_FILE").map(PathBuf::from),
        })
    };
    match run() {
        Ok(mut c) => {
            c.status(true, false, true);
            Box::into_raw(Box::new(c)).cast()
        }
        Err(e) => {
            eprintln!("XROS_PASSTHROUGH_CREATE_ERROR {e:#}");
            std::ptr::null_mut()
        }
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_add_image(
    ctx: *mut c_void,
    image: u64,
    format: i32,
    width: u32,
    height: u32,
) -> i32 {
    let c = &mut *ctx.cast::<Context>();
    match c.renderer.add_image(
        vk::Image::from_raw(image),
        vk::Format::from_raw(format),
        width,
        height,
    ) {
        Ok(i) => i as i32,
        Err(e) => {
            eprintln!("XROS_PASSTHROUGH_IMAGE_ERROR {e:#}");
            -1
        }
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_observe(
    ctx: *mut c_void,
    eyes: *const Eye,
    flags: u64,
    stamp: u64,
) {
    let c = &mut *ctx.cast::<Context>();
    let eyes = std::slice::from_raw_parts(eyes, 2);
    c.renderer.camera.as_mut().unwrap().observe_views(
        openxr::ViewStateFlags::from_raw(flags),
        &[eyes[0].view(), eyes[1].view()],
        stamp,
    );
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_reset(ctx: *mut c_void) {
    (&mut *ctx.cast::<Context>())
        .renderer
        .camera
        .as_mut()
        .unwrap()
        .reset_pose_history();
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_render(
    ctx: *mut c_void,
    left: u32,
    right: u32,
    eyes: *const Eye,
) -> i32 {
    let c = &mut *ctx.cast::<Context>();
    let eyes = std::slice::from_raw_parts(eyes, 2);
    match c
        .renderer
        .render([left, right], &[eyes[0].view(), eyes[1].view()])
    {
        Ok(active) => i32::from(active),
        Err(e) => {
            eprintln!("XROS_PASSTHROUGH_RENDER_ERROR {e:#}");
            c.status(false, false, true);
            -1
        }
    }
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_submitted(ctx: *mut c_void, active: bool) {
    let c = &mut *ctx.cast::<Context>();
    c.renderer.camera.as_mut().unwrap().submitted(active);
    c.status(true, active, false);
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_idle(ctx: *mut c_void) {
    let c = &mut *ctx.cast::<Context>();
    c.renderer.camera.as_ref().unwrap().set_visible(false);
    c.status(true, false, false);
}
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xros_camera_destroy(ctx: *mut c_void) {
    if !ctx.is_null() {
        let mut c = Box::from_raw(ctx.cast::<Context>());
        c.status(false, false, true);
    }
}
