#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build the locked upstream Godot archive with optional XR maintenance patches."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--archive', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--baseline', action='store_true')
p.add_argument('--cc', default='aarch64-linux-gnu-gcc')
p.add_argument('--cxx', default='aarch64-linux-gnu-g++')
p.add_argument('--scons', default='scons')
p.add_argument('--jobs', type=int, default=min(os.cpu_count() or 1, 12))
p.add_argument('--target-libm', type=Path,
               help='Optional libm.so.6 from the target SDK/system for linking')
a = p.parse_args()
recipe = Path(__file__).resolve().parent
lock = json.loads((recipe / 'upstream.json').read_text())
if sha256(a.archive) != lock['archive_sha256']:
    p.error('Upstream source archive checksum differs from the lock')
if a.jobs < 1:
    p.error('--jobs must be positive')
out = a.output.resolve()
out.mkdir(parents=True, exist_ok=False)
unpack = out / 'source'
unpack.mkdir()
with tarfile.open(a.archive) as archive:
    members = archive.getmembers()
    roots = set()
    for member in members:
        path = PurePosixPath(member.name)
        if path.is_absolute() or '..' in path.parts or not path.parts:
            raise ValueError('Unsafe archive path')
        if not (member.isfile() or member.isdir()):
            raise ValueError('Source archive contains a non-file entry')
        roots.add(path.parts[0])
    if len(roots) != 1:
        raise ValueError('Expected one upstream source directory')
    archive.extractall(unpack)
source = unpack / roots.pop()
applied = []
if not a.baseline:
    for patch in lock['patches']:
        patch_path = recipe / patch['file']
        if sha256(patch_path) != patch['sha256']:
            raise ValueError('Maintenance patch checksum differs from the lock')
        subprocess.run(['patch', '-p1', '--fuzz=0', '--forward',
                        '-i', str(patch_path)], cwd=source, check=True)
        applied.append(patch)
command = [a.scons, 'platform=linuxbsd', 'arch=arm64',
           'target=template_release', 'CC=' + a.cc, 'CXX=' + a.cxx,
           'LINK=' + a.cxx, 'use_static_cpp=no', 'lto=none',
           'debug_symbols=no', 'vulkan=yes', 'opengl3=no', 'wayland=no',
           'speechd=no', 'accesskit=no', 'optimize=speed', '-j' + str(a.jobs)]
target_libm = None
if a.target_libm:
    lib = out / 'target-lib'
    lib.mkdir()
    shutil.copy2(a.target_libm, lib / 'libm.so.6')
    (lib / 'libm.so').symlink_to('libm.so.6')
    command += ['linkflags=-L' + str(lib)]
    target_libm = {'file': 'libm.so.6', 'sha256': sha256(lib / 'libm.so.6')}
environment = dict(os.environ)
environment.setdefault('SOURCE_DATE_EPOCH', '0')
with (out / 'build.log').open('w') as log:
    subprocess.run(command, cwd=source, env=environment, stdout=log,
                   stderr=subprocess.STDOUT, check=True)
root = out / 'root'
root.mkdir()
binary = root / 'godot-native'
shutil.copy2(source / 'bin/godot.linuxbsd.template_release.arm64', binary)
for filename in ('LICENSE.txt', 'COPYRIGHT.txt'):
    shutil.copy2(source / filename, root / filename)
manifest = {'upstream': lock['commit'], 'archive_sha256': lock['archive_sha256'],
            'patches': applied,
            'command': [part.replace(str(out), '@OUTPUT@') for part in command],
            'target_libm': target_libm,
            'binary_sha256': sha256(binary),
            'compiler': subprocess.check_output([a.cxx, '--version'], text=True).splitlines()[0]}
(root / 'build-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
with tarfile.open(out / 'godot-native.tar.gz', 'w:gz') as archive:
    for path in sorted(root.iterdir()):
        archive.add(path, arcname=path.name)
print(json.dumps(manifest, indent=2))
