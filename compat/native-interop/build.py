#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build explicit app-scoped Vulkan/OpenXR compatibility layers."""
import argparse
import ast
import configparser
import hashlib
import gzip
import json
from pathlib import Path
import subprocess
import tarfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--openxr-include', type=Path, required=True)
p.add_argument('--cross-file', type=Path)
p.add_argument('--cxx', default='c++')
p.add_argument('--cxx-option', action='append', default=[])
p.add_argument('--prefix', default='/opt/mix-native-interop')
a = p.parse_args()
source = Path(__file__).resolve().parent
out = a.output.resolve()
out.mkdir(parents=True, exist_ok=False)
flags = []
compiler = a.cxx
if a.cross_file:
    cross = configparser.ConfigParser(interpolation=None)
    cross.read(a.cross_file)
    compiler = ast.literal_eval(cross['binaries']['cpp'])
    for key in ('cpp_args', 'cpp_link_args'):
        flags += ast.literal_eval(cross['built-in options'].get(key, '[]'))
flags += a.cxx_option
root = out / 'root'
lib = root / 'lib'
lib.mkdir(parents=True)
commands = []
for name, library in [('interop-layer.cpp', 'libqiyu-interop.so'),
                      ('lifecycle-layer.cpp', 'libqiyu-xr-lifecycle.so')]:
    cmd = [compiler, *flags, '-std=c++17', '-O2', '-shared', '-fPIC',
           '-fvisibility=hidden', '-pthread']
    if name.startswith('lifecycle'):
        cmd += ['-I' + str(a.openxr_include.resolve())]
    cmd += [str(source / name), '-o', str(lib / library)]
    with (out / (name + '.log')).open('w') as log:
        subprocess.run(cmd, check=True, stdout=log, stderr=subprocess.STDOUT)
    commands.append(cmd)
prefix = Path(a.prefix)
if not prefix.is_absolute():
    p.error('--prefix must be an absolute target path')
vk = root / 'share/vulkan/explicit_layer.d'
xr = root / 'share/openxr/1/api_layers/explicit.d'
vk.mkdir(parents=True)
xr.mkdir(parents=True)
(vk / 'qiyu-interop.json').write_text(json.dumps({
    'file_format_version': '1.2.0', 'layer': {
        'name': 'VK_LAYER_QIYU_native_interop', 'type': 'GLOBAL',
        'library_path': str(prefix / 'lib/libqiyu-interop.so'),
        'api_version': '1.3.0', 'implementation_version': '1',
        'description': 'Qiyu native XR shared-image compatibility',
        'functions': {'vkNegotiateLoaderLayerInterfaceVersion':
                      'vkNegotiateLoaderLayerInterfaceVersion'}}}, indent=2) + '\n')
(xr / 'qiyu-lifecycle.json').write_text(json.dumps({
    'file_format_version': '1.0.0', 'api_layer': {
        'name': 'XR_APILAYER_QIYU_lifecycle',
        'library_path': str(prefix / 'lib/libqiyu-xr-lifecycle.so'),
        'api_version': '1.0', 'implementation_version': '1',
        'description': 'Qiyu native XR storage teardown compatibility'}}, indent=2) + '\n')
profile = {
    'VK_LAYER_PATH': str(prefix / 'share/vulkan/explicit_layer.d'),
    'VK_INSTANCE_LAYERS': 'VK_LAYER_QIYU_native_interop',
    'QIYU_VK_SHARED_MUTABLE_LAYOUT': '1',
    'XR_API_LAYER_PATH': str(prefix / 'share/openxr/1/api_layers/explicit.d'),
    'XR_ENABLE_API_LAYERS': 'XR_APILAYER_QIYU_lifecycle',
    'QIYU_XR_WAIT_IDLE_BEFORE_DESTROY': '1'}
(root / 'profile.json').write_text(json.dumps(profile, indent=2) + '\n')
(root / 'LICENSE-MIT').write_bytes((source / 'LICENSE-MIT').read_bytes())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = {'prefix': str(prefix), 'commands': commands,
            'sources': {name: sha(source / name) for name in
                        ('interop-layer.cpp', 'lifecycle-layer.cpp', 'build.py', 'LICENSE-MIT')},
            'openxr_headers': {name: sha(a.openxr_include / 'openxr' / name) for name in
                               ('openxr.h', 'openxr_platform.h', 'openxr_loader_negotiation.h')},
            'files': {str(f.relative_to(root)): sha(f) for f in sorted(root.rglob('*')) if f.is_file()},
            'activation': 'Explicit per-process opt-in; no automatic service or global ICD change',
            'scope': 'Validated Godot Vulkan2 ordinary mutable sRGB stereo images on A081 SteamVR; other apps/drivers require their own acceptance'}
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
with (out / 'native-interop.tar.gz').open('wb') as target:
    with gzip.GzipFile(filename='', fileobj=target, mode='wb', mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as archive:
            for f in sorted(root.rglob('*')):
                if not f.is_file():
                    continue
                info = archive.gettarinfo(str(f), arcname=str(f.relative_to(root)))
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                info.mtime = 0
                with f.open('rb') as stream:
                    archive.addfile(info, stream)
print(json.dumps({'archive_sha256': sha(out / 'native-interop.tar.gz'),
                  'libraries': {f.name: sha(f) for f in lib.iterdir()}}, indent=2))
