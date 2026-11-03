# Independent XR Monado adaptation

Actual downstream source is committed on the pinned upstream history. Public pose and controller headers are provided by libxr-head-dev and libxr-controller-dev. Build-Depends express the owner of each interface; source files do not maintain duplicate copies.

`dpkg-buildpackage -us -uc -b` builds the SteamVR driver into a private multiarch path. The checked-in CMake profile explicitly disables the OpenXR backend; an OpenXR package must be implemented and validated separately. Installation does not select an active runtime or alter the standard Monado packages.
