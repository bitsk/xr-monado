# Godot native application adapter

Apply `integration.patch` to the exact official demo directory pinned in
`upstream.json`. Substitute `@QIYU_NATIVE_CONTROL@` with the target's app-local
control-file path when materializing the demo. Keep the upstream MIT license.
The engine is a separate checksum-locked input; it is not vendored into this
patch stack. The selected native example uses the maintenance build in
`../godot-engine/` with `xr/openxr/headset_only=true` to avoid desktop mirror
images. The original official engine remains a comparison/rollback input.

The adapter uses Godot's actual 3D viewport scale and OpenXR foveation APIs.
Control is a JSON object with a new integer `serial` per update: `scale`,
`ffr`, `adaptive`, `target_ms`, `motion`, and `quit`. The default scale is 1,
FFR is off and subsampled images are explicitly disabled. Feedback changes
scale in 0.05 steps with hysteresis. `motion` drives scripted scene-origin yaw
for repeatable rendering checks; it is not a physical tracking measurement.

DRS returned to an identical full-resolution scanout in the tested static
scene and reduced application GPU time. The tested FFR setting had no net
gain and remains off. Per-frame rendering timers and KMS callbacks do not
establish panel FPS. Wearer checks for clarity, real head movement and optics
remain separate. The compatibility profile is required for the tested
SteamVR ordinary mutable-image path.

Place `godot-native` beside the materialized `project.godot`. `launch.py`
reads an adjacent `launch.json` with `compatibility_profile`, `openxr_runtime`,
`vulkan_icd` and `control_file` absolute paths, plus an optional `log_directory`.
The control path must match the substituted demo control path. The launcher
clears stale control commands and execs the engine in the same PID, allowing
a SteamVR binary application manifest to identify the actual XR process.
It does not select a global renderer or start/stop the VR session.

Before running a source project with the release template, import its resources
with the matching editor on the target (`--headless --editor --import
--xr-mode off --path /path/to/project`). Alternatively use a complete exported
PCK. Generated `.import` descriptions and `.godot/imported` textures must match;
copying only the cache can leave the scene unable to load while XR still runs.
