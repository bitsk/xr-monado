# Gamescope OpenVR startup

The deferred OpenVR backend initializes while holding its exclusive init lock.
Calling `UpdateFeatures()` there acquires the Wayland mutex, while a simultaneous
`gamescope_control` bind holds the Wayland mutex and waits for the init read lock.
Steam then stalls in a Wayland roundtrip before creating its VR main interface.

The patch marks feature information dirty during initialization and sends it from
the first completed frame. Later refresh-rate changes still notify clients.
The backend, control protocol and rendering settings remain enabled.

`upstream.json` locks the exact upstream commit, dependency archives and patch.
Use a native ARM64 SteamOS build environment with its graphics development packages:

```sh
python3 build.py --cache /path/to/verified-source-cache --output /path/to/new-build
```

The cache contains the archive filenames listed in the lock. The build verifies
every digest and disables additional source downloads and Gamescope tests.
`--reference` builds the identical source and options without this patch for a
controlled comparison. Only the executable is packaged; matching installed
SteamOS data, scripts and gamescope-reaper remain required.

Select the private executable through qiyu-vr-session's root-owned
`/etc/mix-vr-shell-gamescope.json`, with format `mix-vr-gamescope-v1`, absolute
`binary` path under `/opt`, and `sha256`. Removing that file selects the system
package. Gamescope uses the system desktop WSI ICD; ready.py separately selects
the private compositor ICD. A build alone does not establish device acceptance.
