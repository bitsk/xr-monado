#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build a native ARM64 Gamescope from checksum-locked source archives."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--cache', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--reference', action='store_true', help='Build without the OpenVR fix')
p.add_argument('--jobs', type=int, default=2)
a = p.parse_args()
if os.uname().machine != 'aarch64':
    p.error('Use the matching native ARM64 SteamOS SDK/build environment')
recipe = Path(__file__).resolve().parent
lock = json.loads((recipe / 'upstream.json').read_text())
out = a.output.resolve()
out.mkdir(parents=True, exist_ok=False)
source = out / 'gamescope-source'
build = out / 'gamescope-build'
sha = lambda f: hashlib.sha256(f.read_bytes()).hexdigest()

def extract(entry, destination):
    archive = a.cache / entry['archive']
    if sha(archive) != entry['archive_sha256']:
        raise RuntimeError('Source archive checksum mismatch: ' + archive.name)
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive) as tar:
        for member in tar:
            parts = Path(member.name).parts
            if member.name.startswith('/') or '..' in parts:
                raise RuntimeError('Unsafe source archive path')
            if len(parts) < 2 or not (member.isfile() or member.isdir()):
                continue
            # OpenVR samples/prebuilt assets do not participate in this build.
            if destination.name == 'openvr' and parts[1] in ('samples', 'bin', 'lib', 'controller_callouts'):
                continue
            member.name = str(Path(*parts[1:]))
            tar.extract(member, destination)

extract(lock, source)
for entry in lock['dependencies']:
    extract(entry, source / entry['path'])
for name in ('glm', 'stb'):
    for f in (source / 'subprojects/packagefiles' / name).iterdir():
        if f.is_file():
            shutil.copyfile(f, source / 'subprojects' / name / f.name)
commands = []

def run(label, cmd):
    commands.append(cmd)
    with (out / (label + '.log')).open('w') as log:
        subprocess.run(cmd, check=True, stdout=log, stderr=subprocess.STDOUT)

if not a.reference:
    for entry in lock['patches']:
        patch = recipe / entry['path']
        if sha(patch) != entry['sha256']:
            raise RuntimeError('Patch checksum mismatch')
        run('patch', ['patch', '-d', str(source), '-p1', '-i', str(patch)])
options = [
    '--prefix=/usr', '--buildtype=debugoptimized', '--wrap-mode=nodownload',
    '-Denable_gamescope_wsi_layer=false', '-Denable_tests=false',
    '-Ddrm_backend=enabled', '-Dsdl2_backend=enabled', '-Davif_screenshots=enabled',
    '-Dpipewire=enabled', '-Dinput_emulation=enabled', '-Drt_cap=enabled',
    '-Dbenchmark=disabled', '-Dforce_fallback_for=libliftoff,vkroots,libdisplay-info',
    '-Dcpp_args=-ffile-prefix-map=' + str(out) + '=/usr/src/qiyu-gamescope',
]
run('configure', ['meson', 'setup', str(build), str(source), *options])
run('compile', ['ninja', '-C', str(build), '-j' + str(a.jobs), 'src/gamescope'])
package = out / 'package'
(package / 'bin').mkdir(parents=True)
run('strip', ['strip', '--strip-debug', '-o', str(package / 'bin/gamescope'), str(build / 'src/gamescope')])
(package / 'bin/gamescope').chmod(0o755)
shutil.copyfile(source / 'LICENSE', package / 'LICENSE')
shutil.copyfile(recipe / 'upstream.json', package / 'source-lock.json')
manifest = {'format': 'qiyu-gamescope-build-v1', 'source': lock['commit'],
            'reference': a.reference, 'commands': commands,
            'binary_sha256': sha(package / 'bin/gamescope'),
            'data_dependency': 'Matching SteamOS /usr/share/gamescope and gamescope-reaper',
            'activation': 'Explicit root-owned /etc/mix-vr-shell-gamescope.json selection'}
(out / 'build.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(json.dumps({'binary': str(package / 'bin/gamescope'), 'sha256': manifest['binary_sha256']}))
