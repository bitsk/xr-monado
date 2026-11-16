# Independent XR Monado adaptation

Actual downstream source is committed on the pinned upstream history. Public pose and controller headers are provided by libxr-head-dev and libxr-controller-dev. Build-Depends express the owner of each interface; source files do not maintain duplicate copies.

`dpkg-buildpackage -us -uc -b` builds the SteamVR driver into a private multiarch path. The checked-in CMake profile explicitly disables the OpenXR backend; an OpenXR package must be implemented and validated separately. Installation does not select an active runtime or alter the standard Monado packages.


The SteamVR provider supports explicit `XR_DEVICE=a081` (the default) and
`XR_DEVICE=vr700`. Device creation checks the running device-tree model against
that selection and still requires `A081_BRINGUP=1`. The legacy `a081` builder
identifier and `A081_QVR_*` / `A081_CONTROLLER_*` environment names are shared
interface names, so the existing session manager can retain the same head,
controller and hand producer sockets. `qvr-pose` requires the producer's
`android-portrait-v1` coordinate contract; it does not manufacture tracking data.

VR700 selects a distinct HMD/controller identity and the optical constants in
`devices/vr700/optics-profile.json`. Its panel layout is 1920 x 3664 at 72 Hz,
with two rotated 1832 x 1920 views. The shared SteamVR path retains real QVR head
poses, physical controller poses/buttons, optical hand joints and automatic
controller/hand switching. `XR_HAND_STATE_SOCKET` enables the existing unified
input pair. `XR_CONTROLLER_MODEL_DIR` selects that device's grip/aim anchors;
without an override, the model-specific directory is
`/usr/share/xr/devices/<device>/controllers`. Missing VR700 anchors do not fall
back to MIX anchors. Per-device camera/IMU calibration belongs to the producer
and is never embedded in this runtime.

Build with the existing `driver_monado` target and the public development
packages declared in `debian/control`. OpenVR headers are already in the pinned
upstream tree; SteamVR itself and the vendor tracking SDKs are separate session
inputs. Building this plugin does not install those inputs or validate their
device compatibility. Parameter extraction and compilation do not establish
live tracking, physical distortion, current wearer IPD or end-to-end latency.
