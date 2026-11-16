# XROS OpenXR passthrough adapter

`xr-runtime-passthrough` owns the MIX camera background beneath HyperSpace.
The deployed MIX stack uses the Monado SteamVR driver and SteamVR OpenXR runtime;
it does not run Monado's native compositor. This explicit API layer supplies
`XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND` to Vulkan2 stereo clients, projects
stereo cameras into private runtime swapchains, and prepends an opaque background
projection before forwarding application layers with OPAQUE environment blending.
Final layer composition and reprojection stay in the active runtime.

This is runtime integration code executing in the client process. It is not a
new compositor daemon, a modification to the closed SteamVR compositor, or an
implementation of `XR_FB_passthrough`. Applications use the standard OpenXR blend
mode and layer alpha flags. They do not parse camera calibration or snapshot files.
See the [OpenXR environment blend contract](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrEnvironmentBlendMode.html).

## Ownership and build

The camera reader, GPU RAW10 decoder, radial lookup projection and optional
rotation compensation moved from the canonical `xr-shell` commit `f22d5da`
(`src/passthrough.rs`, `shaders/passthrough.comp`, camera branch of
`shaders/panorama.frag`). Their GPL-3.0-or-later license is preserved. There is one
maintained implementation, here. Shell no longer links this code.

CMake target `xros_passthrough` uses the committed Cargo lock and GLSL sources.
The component builder sets `XR_CARGO_TARGET`, `XR_CARGO_TARGET_DIR`, toolchain and
external dependency inputs. Debian installs the library and explicit manifest
under `/usr/lib/<multiarch>/xr/passthrough`. Installation activates no layers.

## Activation

The session manager owns activation via the profile's optional `passthrough`
object (`config` absolute path). Runtime activation uses installed paths and
normal file ownership/permissions; it has no content-digest selector. The existing
version-2 calibration schema is unchanged; calibration is never shipped here.
The manager configures the existing camera producer and supplies:

- `XR_API_LAYER_PATH`: includes this package's `explicit.d` directory.
- `XR_ENABLE_API_LAYERS`: includes `XR_APILAYER_XROS_passthrough` before lifecycle adapters.
- `XROS_PASSTHROUGH_CONFIG`: root-owned system calibration for the existing QSP1 tap.
- `XROS_SESSION_TOKEN` and `XR_RUNTIME_PASSTHROUGH_STATUS_FILE`: session-scoped health reporting.

A client must enable `XR_KHR_vulkan_enable2`, create its Vulkan instance through
`xrCreateVulkanInstanceKHR`, and use a queue supporting graphics and compute.
The layer borrows that loader/device/queue; it never creates or destroys the
application device. The application's ordinary OpenXR/Vulkan queue synchronization
obligations also cover `xrEndFrame`. One runtime layer slot is reserved and removed
from the advertised application limit. The backend must support OPAQUE and at least
two composition layers. Only PRIMARY_STEREO is exposed for this adapter.

Opaque frames preserve application content and stop camera reads. Alpha frames
carry the camera projection under all application layers. Zero-layer frames render
nothing. The adapter waits its own GPU fence before releasing private swapchain
images. Teardown destroys GPU views/framebuffers before releasing XR storage.
Other API layers, application swapchains and submitted layer chains are preserved.

## Timing, failure and evidence

Only the existing hardware owner's root-owned QSP1 snapshots are read; no V4L2
open, sensor reconfiguration, browser or debug-service dependency is introduced.
Both eye frames must be younger than 150 ms. Missing/stale pairs render a black
background instead of freezing an old camera view. Runtime status distinguishes
interface availability, current camera freshness and successfully submitted
camera frames. A failed image wait/render poisons the adapter session and returns
an error; the session owner performs the normal stop/restore path.

Optional dequeue-rotation compensation uses observations located at converted
CLOCK_MONOTONIC times through `XR_KHR_convert_timespec_time`. If the application
has not enabled that extension, compensation is inactive. Recenter events reset
history at the change time. This is dequeue timing, not exposure synchronization;
translation/depth reprojection is not implemented. Physical camera extrinsics,
colour calibration, motion comfort and display scanout still require device tests.

The ARM64 package has been deployed on MIX. HyperSpace exercised alpha/opaque
switching, stopped snapshot reads while opaque, and resumed advancing distinct
source counters for both eyes. A stock Khronos `hello_xr` Vulkan2/AlphaBlend client
also submitted advancing stereo camera backgrounds and exited normally. That
sample does not enable the clock-conversion extension, so its rotation
compensation correctly remained inactive. The wearer confirmed the real scene
and floating desktop were visible with no obvious abnormality in a brief check.

Normal scene/runtime shutdown and desktop restoration passed, including after
path-only session configuration replaced digest-pinned startup. Head observation,
controller services and the optional hand service remained active; physical
controller/gesture accuracy was not reaccepted. The existing tracking producer
still exits nonzero on shutdown (`tracking_capture`), despite successful scene
cleanup and zero remaining FastRPC objects; the overall session validation gate
is therefore not passed. Native seated/standing reset requests completed but no
OpenXR reference-space event was observed in HyperSpace, so recenter acceptance
remains open. Stale-source fault behavior, physical calibration, exposure alignment
and long-duration motion comfort have not been validated on this combination.
