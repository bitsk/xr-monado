// SPDX-License-Identifier: GPL-3.0-or-later
//! Owns only the camera projection resources, never the application's device.
use crate::{camera::Passthrough, gpu::Gpu};
use anyhow::{Context, Result, ensure};
use ash::vk;
use std::{io::Cursor, rc::Rc};

pub(crate) fn colour_range() -> vk::ImageSubresourceRange {
    vk::ImageSubresourceRange::default()
        .aspect_mask(vk::ImageAspectFlags::COLOR)
        .level_count(1)
        .layer_count(1)
}
pub(crate) fn image_barrier(
    image: vk::Image,
    old: vk::ImageLayout,
    new: vk::ImageLayout,
    src: vk::AccessFlags,
    dst: vk::AccessFlags,
) -> vk::ImageMemoryBarrier<'static> {
    vk::ImageMemoryBarrier::default()
        .image(image)
        .old_layout(old)
        .new_layout(new)
        .src_access_mask(src)
        .dst_access_mask(dst)
        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .subresource_range(colour_range())
}
struct Target {
    view: vk::ImageView,
    framebuffer: vk::Framebuffer,
    width: u32,
    height: u32,
}
pub struct Renderer {
    gpu: Rc<Gpu>,
    pub camera: Option<Passthrough>,
    set_layout: vk::DescriptorSetLayout,
    pool: vk::DescriptorPool,
    layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
    pass: vk::RenderPass,
    command_pool: vk::CommandPool,
    cmd: vk::CommandBuffer,
    fence: vk::Fence,
    pending: bool,
    set: vk::DescriptorSet,
    targets: Vec<Target>,
}
impl Renderer {
    pub fn new(gpu: Rc<Gpu>, format: vk::Format) -> Result<Self> {
        let mut r = Self {
            gpu,
            camera: None,
            set_layout: Default::default(),
            pool: Default::default(),
            layout: Default::default(),
            pipeline: Default::default(),
            pass: Default::default(),
            command_pool: Default::default(),
            cmd: Default::default(),
            fence: Default::default(),
            pending: false,
            set: Default::default(),
            targets: Vec::new(),
        };
        unsafe {
            let d = &r.gpu.device;
            r.set_layout = d.create_descriptor_set_layout(
                &vk::DescriptorSetLayoutCreateInfo::default().bindings(&[
                    vk::DescriptorSetLayoutBinding::default()
                        .binding(3)
                        .descriptor_count(1)
                        .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                        .stage_flags(vk::ShaderStageFlags::FRAGMENT),
                    vk::DescriptorSetLayoutBinding::default()
                        .binding(4)
                        .descriptor_count(1)
                        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
                        .stage_flags(vk::ShaderStageFlags::FRAGMENT),
                ]),
                None,
            )?;
            r.pool = d.create_descriptor_pool(
                &vk::DescriptorPoolCreateInfo::default()
                    .max_sets(1)
                    .pool_sizes(&[
                        vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
                            descriptor_count: 1,
                        },
                        vk::DescriptorPoolSize {
                            ty: vk::DescriptorType::STORAGE_BUFFER,
                            descriptor_count: 1,
                        },
                    ]),
                None,
            )?;
            r.set = d.allocate_descriptor_sets(
                &vk::DescriptorSetAllocateInfo::default()
                    .descriptor_pool(r.pool)
                    .set_layouts(&[r.set_layout]),
            )?[0];
            r.camera = Passthrough::from_environment(r.gpu.clone(), r.set)?;
            ensure!(r.camera.is_some(), "camera configuration required");
            r.layout = d.create_pipeline_layout(
                &vk::PipelineLayoutCreateInfo::default()
                    .set_layouts(&[r.set_layout])
                    .push_constant_ranges(&[vk::PushConstantRange {
                        stage_flags: vk::ShaderStageFlags::FRAGMENT,
                        offset: 0,
                        size: 64,
                    }]),
                None,
            )?;
            let attachments = [vk::AttachmentDescription::default()
                .format(format)
                .samples(vk::SampleCountFlags::TYPE_1)
                .load_op(vk::AttachmentLoadOp::CLEAR)
                .store_op(vk::AttachmentStoreOp::STORE)
                .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
                .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
                .initial_layout(vk::ImageLayout::UNDEFINED)
                .final_layout(vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL)];
            let colours = [vk::AttachmentReference {
                attachment: 0,
                layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
            }];
            let subpasses = [vk::SubpassDescription::default()
                .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
                .color_attachments(&colours)];
            let deps = [vk::SubpassDependency::default()
                .src_subpass(vk::SUBPASS_EXTERNAL)
                .dst_subpass(0)
                .src_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
                .dst_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
                .dst_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE)];
            r.pass = d.create_render_pass(
                &vk::RenderPassCreateInfo::default()
                    .attachments(&attachments)
                    .subpasses(&subpasses)
                    .dependencies(&deps),
                None,
            )?;
            let vertex = ash::util::read_spv(&mut Cursor::new(include_bytes!(concat!(
                env!("OUT_DIR"),
                "/background.vert.spv"
            ))))?;
            let fragment = ash::util::read_spv(&mut Cursor::new(include_bytes!(concat!(
                env!("OUT_DIR"),
                "/background.frag.spv"
            ))))?;
            let vs =
                d.create_shader_module(&vk::ShaderModuleCreateInfo::default().code(&vertex), None)?;
            let fs = match d
                .create_shader_module(&vk::ShaderModuleCreateInfo::default().code(&fragment), None)
            {
                Ok(s) => s,
                Err(e) => {
                    d.destroy_shader_module(vs, None);
                    return Err(e.into());
                }
            };
            let stages = [
                vk::PipelineShaderStageCreateInfo::default()
                    .stage(vk::ShaderStageFlags::VERTEX)
                    .module(vs)
                    .name(c"main"),
                vk::PipelineShaderStageCreateInfo::default()
                    .stage(vk::ShaderStageFlags::FRAGMENT)
                    .module(fs)
                    .name(c"main"),
            ];
            let vertex_input = vk::PipelineVertexInputStateCreateInfo::default();
            let assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
                .topology(vk::PrimitiveTopology::TRIANGLE_LIST);
            let viewport = vk::PipelineViewportStateCreateInfo::default()
                .viewport_count(1)
                .scissor_count(1);
            let raster = vk::PipelineRasterizationStateCreateInfo::default()
                .polygon_mode(vk::PolygonMode::FILL)
                .cull_mode(vk::CullModeFlags::NONE)
                .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
                .line_width(1.);
            let samples = vk::PipelineMultisampleStateCreateInfo::default()
                .rasterization_samples(vk::SampleCountFlags::TYPE_1);
            let blends = [vk::PipelineColorBlendAttachmentState::default()
                .color_write_mask(vk::ColorComponentFlags::RGBA)];
            let blend = vk::PipelineColorBlendStateCreateInfo::default().attachments(&blends);
            let dynamic = vk::PipelineDynamicStateCreateInfo::default()
                .dynamic_states(&[vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR]);
            let pipelines = d.create_graphics_pipelines(
                vk::PipelineCache::null(),
                &[vk::GraphicsPipelineCreateInfo::default()
                    .stages(&stages)
                    .vertex_input_state(&vertex_input)
                    .input_assembly_state(&assembly)
                    .viewport_state(&viewport)
                    .rasterization_state(&raster)
                    .multisample_state(&samples)
                    .color_blend_state(&blend)
                    .dynamic_state(&dynamic)
                    .layout(r.layout)
                    .render_pass(r.pass)],
                None,
            );
            d.destroy_shader_module(vs, None);
            d.destroy_shader_module(fs, None);
            match pipelines {
                Ok(p) => r.pipeline = p[0],
                Err((p, e)) => {
                    for h in p {
                        d.destroy_pipeline(h, None);
                    }
                    return Err(e.into());
                }
            }
            r.command_pool = d.create_command_pool(
                &vk::CommandPoolCreateInfo::default()
                    .queue_family_index(r.gpu.family)
                    .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER),
                None,
            )?;
            r.cmd = d.allocate_command_buffers(
                &vk::CommandBufferAllocateInfo::default()
                    .command_pool(r.command_pool)
                    .level(vk::CommandBufferLevel::PRIMARY)
                    .command_buffer_count(1),
            )?[0];
            r.fence = d.create_fence(&vk::FenceCreateInfo::default(), None)?;
        }
        Ok(r)
    }
    pub fn add_image(
        &mut self,
        image: vk::Image,
        format: vk::Format,
        width: u32,
        height: u32,
    ) -> Result<u32> {
        unsafe {
            let d = &self.gpu.device;
            let view = d.create_image_view(
                &vk::ImageViewCreateInfo::default()
                    .image(image)
                    .view_type(vk::ImageViewType::TYPE_2D)
                    .format(format)
                    .subresource_range(colour_range()),
                None,
            )?;
            let framebuffer = match d.create_framebuffer(
                &vk::FramebufferCreateInfo::default()
                    .render_pass(self.pass)
                    .attachments(&[view])
                    .width(width)
                    .height(height)
                    .layers(1),
                None,
            ) {
                Ok(f) => f,
                Err(e) => {
                    d.destroy_image_view(view, None);
                    return Err(e.into());
                }
            };
            self.targets.push(Target {
                view,
                framebuffer,
                width,
                height,
            });
            Ok((self.targets.len() - 1) as u32)
        }
    }
    /// Called on the application's XR submission thread. xrEndFrame requires the
    /// bound Vulkan queue to be externally synchronized by the application.
    pub fn render(&mut self, indices: [u32; 2], views: &[openxr::View; 2]) -> Result<bool> {
        unsafe {
            let d = &self.gpu.device;
            if self.pending {
                d.wait_for_fences(&[self.fence], true, u64::MAX)?;
                self.pending = false;
            }
            d.reset_command_buffer(self.cmd, vk::CommandBufferResetFlags::empty())?;
            d.begin_command_buffer(
                self.cmd,
                &vk::CommandBufferBeginInfo::default()
                    .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT),
            )?;
            let camera = self.camera.as_mut().context("camera renderer missing")?;
            camera.set_visible(true);
            camera.prepare(self.cmd)?;
            camera.update_projection(views)?;
            let active = camera.active();
            for eye in 0..2 {
                let target = self
                    .targets
                    .get(indices[eye] as usize)
                    .context("invalid swapchain image index")?;
                let rect = vk::Rect2D {
                    offset: Default::default(),
                    extent: vk::Extent2D {
                        width: target.width,
                        height: target.height,
                    },
                };
                let clear = [vk::ClearValue {
                    color: vk::ClearColorValue {
                        float32: [0., 0., 0., 1.],
                    },
                }];
                d.cmd_begin_render_pass(
                    self.cmd,
                    &vk::RenderPassBeginInfo::default()
                        .render_pass(self.pass)
                        .framebuffer(target.framebuffer)
                        .render_area(rect)
                        .clear_values(&clear),
                    vk::SubpassContents::INLINE,
                );
                if active {
                    d.cmd_set_viewport(
                        self.cmd,
                        0,
                        &[vk::Viewport {
                            x: 0.,
                            y: 0.,
                            width: target.width as f32,
                            height: target.height as f32,
                            min_depth: 0.,
                            max_depth: 1.,
                        }],
                    );
                    d.cmd_set_scissor(self.cmd, 0, &[rect]);
                    d.cmd_bind_pipeline(self.cmd, vk::PipelineBindPoint::GRAPHICS, self.pipeline);
                    d.cmd_bind_descriptor_sets(
                        self.cmd,
                        vk::PipelineBindPoint::GRAPHICS,
                        self.layout,
                        0,
                        &[self.set],
                        &[],
                    );
                    let f = views[eye].fov;
                    // Vulkan framebuffer origin is top-left: uv.y=0 looks up.
                    let constants: [f32; 16] = [
                        f.angle_left.tan(),
                        f.angle_right.tan(),
                        f.angle_up.tan(),
                        f.angle_down.tan(),
                        0.,
                        0.,
                        0.,
                        1.,
                        0.,
                        0.,
                        1.,
                        eye as f32,
                        0.,
                        0.,
                        0.,
                        0.,
                    ];
                    d.cmd_push_constants(
                        self.cmd,
                        self.layout,
                        vk::ShaderStageFlags::FRAGMENT,
                        0,
                        std::slice::from_raw_parts(constants.as_ptr().cast(), 64),
                    );
                    d.cmd_draw(self.cmd, 3, 1, 0, 0);
                }
                d.cmd_end_render_pass(self.cmd);
            }
            d.end_command_buffer(self.cmd)?;
            d.reset_fences(&[self.fence])?;
            d.queue_submit(
                self.gpu.queue,
                &[vk::SubmitInfo::default().command_buffers(&[self.cmd])],
                self.fence,
            )?;
            self.pending = true;
            // Complete writes before releasing runtime-owned swapchain images.
            d.wait_for_fences(&[self.fence], true, u64::MAX)?;
            self.pending = false;
            Ok(active)
        }
    }
}
impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            let d = &self.gpu.device;
            if self.pending {
                let _ = d.wait_for_fences(&[self.fence], true, u64::MAX);
            }
            self.camera.take();
            for t in self.targets.drain(..) {
                d.destroy_framebuffer(t.framebuffer, None);
                d.destroy_image_view(t.view, None);
            }
            d.destroy_fence(self.fence, None);
            d.destroy_command_pool(self.command_pool, None);
            d.destroy_pipeline(self.pipeline, None);
            d.destroy_pipeline_layout(self.layout, None);
            d.destroy_render_pass(self.pass, None);
            d.destroy_descriptor_pool(self.pool, None);
            d.destroy_descriptor_set_layout(self.set_layout, None);
        }
    }
}
