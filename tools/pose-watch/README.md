# Packaged pose observer

The canonical Debian build includes the read-only observer in
`xr-monado-steamvr-driver` at
`/usr/lib/aarch64-linux-gnu/xr/monado/pose-watch`.

Set `XR_POSE_WATCH_SOURCE` in the component build environment to an external
cache file matching `original_source.sha256` in the source lock. Register the
same file and hash in the component builder environment's `inputs` list. The
build verifies the external source, both patches, the prepared source, and the
canonical OpenVR header/license. It does not consume an old deployment binary.
The external full source retains its existing redistribution boundary.

`XR_DEVICE=vr700` selects the exact VR700 DT model, first compatible, installed
root marker, and native QVR model/serial properties. An absent device selection
retains MIX identity. UID1000, the private OpenVR registry, real finite pose
checks, and post-stop invalidation criteria remain required. This helper does
not generate poses or accept simulated tracking.
