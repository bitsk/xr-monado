# Godot XR maintenance patches

`upstream.json` pins the official engine source archive and the patch hashes.
This directory maintains a patch stack, not another copy of Godot.

The teardown patch requests session exit and consumes the runtime's actual
state-change events before calling `xrEndSession` in STOPPING. The wait is
bounded to 500 ms and only runs during destruction. A timeout is reported and
falls back to session destruction; no state or API success is synthesized.
It also records interaction-profile RIDs in the owning cleanup list and only
disconnects the spatial-marker signal when that capability connected it.
The existing application-scoped Vulkan lifetime guard is still required.

The second patch adds the opt-in `xr/openxr/headset_only` project setting,
defaulting to false. On the Linux X11 path it avoids mapping the main desktop
window, drawing the desktop boot splash and blitting an XR eye to the desktop.
The Vulkan renderer, XR viewport and frame submission remain active. This
avoids creating desktop mirror images rather than releasing another process's
live references. The editor and `--xr-mode off` keep their desktop output.
Other window systems have not been accepted for this setting.

Build the checksum-locked archive with SCons and an AArch64 GCC toolchain:

```sh
python3 build.py --archive /path/to/godot-source.tar.gz --output /new/output \
  --target-libm /path/to/target-sdk/libm.so.6
```

`--baseline` builds the same source and compiler options without the patches.
The optional target `libm` keeps the link compatible when the cross toolchain's
default math library is newer than the device's glibc. Its checksum and the
compiler command are recorded in the output manifest; it is not shipped in
the resulting engine archive. Other toolchain library requirements must also
match the target system.

The output includes the engine, upstream license/third-party notices and a
build manifest. A release template without path overrides must sit beside
`project.godot` or its exported PCK; launch it without `--path`. Select it only
for the intended application with the native-interop profile and compatible
X11 Vulkan ICD. This build does not replace SteamVR's renderer or enable
DRS/FFR. Validate image output, motion and teardown separately on the device.
