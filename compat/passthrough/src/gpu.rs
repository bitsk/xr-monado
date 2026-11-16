// SPDX-License-Identifier: GPL-3.0-or-later
// Borrowed application Vulkan handles. This module never destroys the device or instance.
use anyhow::{Context, Result, ensure};
use ash::vk;
use std::{ffi::c_void, rc::Rc};
pub struct Gpu {
    pub device: ash::Device,
    pub memory: vk::PhysicalDeviceMemoryProperties,
    pub queue: vk::Queue,
    pub family: u32,
}
impl Gpu {
    pub fn memory_type(&self, bits: u32, flags: vk::MemoryPropertyFlags) -> Result<u32> {
        (0..self.memory.memory_type_count)
            .find(|&i| {
                bits & (1 << i) != 0
                    && self.memory.memory_types[i as usize]
                        .property_flags
                        .contains(flags)
            })
            .context("compatible Vulkan memory type not found")
    }
}
pub struct Buffer {
    gpu: Rc<Gpu>,
    pub handle: vk::Buffer,
    memory: vk::DeviceMemory,
    mapped: *mut c_void,
    pub size: u64,
}
impl Buffer {
    pub fn host(gpu: Rc<Gpu>, size: u64, usage: vk::BufferUsageFlags) -> Result<Self> {
        let mut b = Self {
            gpu,
            handle: vk::Buffer::null(),
            memory: vk::DeviceMemory::null(),
            mapped: std::ptr::null_mut(),
            size,
        };
        unsafe {
            b.handle = b.gpu.device.create_buffer(
                &vk::BufferCreateInfo::default()
                    .size(size)
                    .usage(usage)
                    .sharing_mode(vk::SharingMode::EXCLUSIVE),
                None,
            )?;
            let req = b.gpu.device.get_buffer_memory_requirements(b.handle);
            let ty = b.gpu.memory_type(
                req.memory_type_bits,
                vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT,
            )?;
            b.memory = b.gpu.device.allocate_memory(
                &vk::MemoryAllocateInfo::default()
                    .allocation_size(req.size)
                    .memory_type_index(ty),
                None,
            )?;
            b.gpu.device.bind_buffer_memory(b.handle, b.memory, 0)?;
            // HOST_COHERENT buffers are written/read only after the existing
            // submission fence. Keep one mapping for their entire lifetime:
            // Turnip otherwise mmap/munmaps the BO on every small UI update
            // and every full RAW upload, faulting the same pages repeatedly.
            b.mapped = b
                .gpu
                .device
                .map_memory(b.memory, 0, b.size, vk::MemoryMapFlags::empty())?;
        }
        Ok(b)
    }
    pub fn write(&self, data: &[u8]) -> Result<()> {
        self.write_at(0, data)
    }
    pub fn write_at(&self, offset: usize, data: &[u8]) -> Result<()> {
        ensure!(
            offset
                .checked_add(data.len())
                .is_some_and(|end| end as u64 <= self.size),
            "staging buffer overflow"
        );
        unsafe {
            std::ptr::copy_nonoverlapping(
                data.as_ptr(),
                (self.mapped as *mut u8).add(offset),
                data.len(),
            );
        }
        Ok(())
    }
    pub fn read(&self) -> Result<Vec<u8>> {
        unsafe {
            let result =
                std::slice::from_raw_parts(self.mapped as *const u8, self.size as usize).to_vec();
            Ok(result)
        }
    }
}
impl Drop for Buffer {
    fn drop(&mut self) {
        unsafe {
            if !self.mapped.is_null() {
                self.gpu.device.unmap_memory(self.memory);
            }
            self.gpu.device.destroy_buffer(self.handle, None);
            self.gpu.device.free_memory(self.memory, None);
        }
    }
}
