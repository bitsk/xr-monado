# Native XR image and teardown compatibility

These explicit layers address two observed A081 SteamVR/Godot Vulkan issues:

* The app's external mutable sRGB stereo images carry a format list and use
  UBWC, while the compositor reconstructs a linear image. The Vulkan layer
  omits the format-list optimization hint for this narrow image class. Both
  sides then use linear storage. Mutable-format support remains enabled;
  ordinary internal HDR/depth images retain their original layouts. No copy
  or frame-loop hook is added. Unknown format pairs or pNext prefixes are
  passed through unchanged.
* Godot can end its XR session while the last GPU frame still references
  runtime-owned images. The OpenXR layer drains the bound Vulkan device before
  session end/storage teardown. After a successful GPU drain and session end it
  skips redundant drains for that teardown; session begin resets this state.
  This is an exit synchronization point, not a per-frame wait. Applications
  must still serialize CPU submission and resource teardown correctly.

Build with Vulkan headers and OpenXR loader-negotiation headers:

```sh
python3 build.py --output /new/output --openxr-include /path/to/include
```

For a cross build, add `--cross-file /path/to/meson-cross.ini`. The file must
provide `binaries.cpp` and optional `built-in options.cpp_args/cpp_link_args`.
Additional compiler options can be passed with `--cxx-option=...`.

The archive installs under `--prefix` (default `/opt/mix-native-interop`).
`profile.json` supplies environment variables **only to the selected app**.
The caller also selects a Vulkan ICD with the required window-system support
and the intended OpenXR runtime. Installation does not select a global ICD,
change services or enable DRS/FFR. A launcher using a wrapper process should
associate the actual render PID with its application key using OpenVR
`IdentifyApplication`.

Acceptance scope is Godot's Vulkan2, sample-count-1, mutable sRGB stereo color
swapchain on the tested A081/SteamVR path. The layout guard preserves existing
extension chains and is opt-in. The lifecycle layer obtains the exact app
Vulkan loader entry point from `xrCreateVulkanInstanceKHR`; its log must report
`vulkan_wait_available=1`. Legacy Vulkan1 bindings, other drivers and other
image classes have not been accepted. Kernel/physical optics and general game
support are separate from this compatibility result.

The app ICD must include its required window-system support (X11 for the
validated Godot invocation). Non-subsampled FFR additionally needs the density
map formats used by the engine; these layers do not add driver capabilities.
The compositor can keep its existing ICD and compression policy.
