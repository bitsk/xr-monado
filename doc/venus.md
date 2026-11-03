# Native Vulkan image sharing with Venus

Venus on Linux requires modifier tiling for external images. The conventional
optimal-tiling OPAQUE_FD image path can report no supported swapchain formats
even when ordinary Vulkan rendering and external-memory extensions work.

`XRT_VK_DRM_MODIFIERS=1` opts the native Vulkan allocator into DRM modifier
images. The device must enable `VK_EXT_image_drm_format_modifier` and its
dependencies. The allocator enumerates format modifiers, queries real external
image import/export support for the requested usage, and selects a supported
single-memory-plane layout. A supported tiled modifier is preferred to LINEAR;
modifier zero remains valid. Formats without a compatible layout are omitted.

The allocator exports the actual chosen modifier with each native image. IPC
carries per-image sizes, dedicated-allocation requirements and modifiers alongside
the transferred native handles. The Vulkan client checks import support for the
received modifier, then recreates the same single-modifier image description.
This uses Vulkan OPAQUE_FD sharing on the same physical device: the imported
image must be identical to the allocating image. Driver-private depth and color
layouts remain opaque, without applying explicit dma-buf plane-layout rules or
querying depth images with incompatible memory-plane aspect masks. Existing
native handle ownership rules are retained.

`XR_KHR_vulkan_enable2` device creation enables the optional modifier extension
and its dependencies when available, then propagates their actual enablement to
the client compositor. The option defaults off. Android, Win32 and existing
optimal-tiling allocation are not switched to modifier images by default.

Service and runtime must be rebuilt together: the native image and IPC metadata
layouts change. Do not mix the resulting client library with an older service.
This implementation is for the native Vulkan client. Legacy Vulkan device
creation and GL/EGL modifier imports are outside the validated path; compressed
multi-plane allocations are not selected.

Acceptance uses a real x86_64 openKylin QEMU guest, Venus over an Intel host GPU,
bounded `hello_xr -g Vulkan2` sessions, visible stereo output, native Monado
metrics and the Khronos Vulkan validation layer. This does not establish
physical-headset tracking, optics, display latency or device compatibility.
