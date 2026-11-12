#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Apply the locked root marker and optional private-session display guard to an external observer."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--compiler-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tracking-display-guard', action='store_true')
    args = parser.parse_args()
    recipe = Path(__file__).resolve().parent
    lock = json.loads((recipe / 'pose-watch-openkylin.source.json').read_text())
    original = args.source.resolve()
    manifest = args.compiler_manifest.resolve()
    patch = recipe / lock['patch']['path']
    for path, expected in ((original, lock['original_source']['sha256']),
                           (manifest, lock['compiler_template']['sha256']),
                           (patch, lock['patch']['sha256'])):
        if not path.is_file() or digest(path) != expected:
            parser.error('Input does not match the fixed build record: ' + str(path))
    command = json.loads(manifest.read_text())[lock['compiler_template']['field']]
    if len(command) < 4 or command[-2] != '-o':
        parser.error('Unexpected compiler command shape')
    headers = [Path(argument[2:]) / 'openvr.h' for argument in command
               if argument.startswith('-I') and (Path(argument[2:]) / 'openvr.h').is_file()]
    if len(headers) != 1 or digest(headers[0]) != lock['openvr_sdk']['header_sha256']:
        parser.error('Keep the original fixed OpenVR SDK header')
    license_file = headers[0].parent / 'LICENSE'
    if not license_file.is_file() or digest(license_file) != lock['openvr_sdk']['license_sha256']:
        parser.error('Keep the original OpenVR SDK license')
    output = args.output.resolve()
    if output.exists():
        parser.error('Choose a new output directory to retain earlier build evidence')
    inputs = output / 'inputs'
    inputs.mkdir(parents=True)
    shutil.copy2(original, inputs / 'pose-watch.original.cpp')
    source = inputs / 'pose-watch.cpp'
    shutil.copy2(original, source)
    subprocess.run(['patch', '--batch', '--fuzz=0', '--no-backup-if-mismatch', '-p1',
                    '--input', str(patch)], cwd=inputs, check=True)
    if digest(source) != lock['patch']['patched_source_sha256']:
        raise RuntimeError('Patched source differs from the exact marker-only change')
    guard = lock.get('optional_display_guard') if args.tracking_display_guard else None
    if guard:
        guard_patch = recipe / guard['patch']
        header = recipe / guard['header']
        if digest(guard_patch) != guard['patch_sha256'] or digest(header) != guard['header_sha256']:
            raise RuntimeError('Display guard inputs differ from the fixed record')
        shutil.copy2(header, inputs / guard['header'])
        subprocess.run(['patch', '--batch', '--fuzz=0', '--no-backup-if-mismatch', '-p1',
                        '--input', str(guard_patch)], cwd=inputs, check=True)
        if digest(source) != guard['patched_source_sha256']:
            raise RuntimeError('Display guard patched source differs from the fixed record')
    sdk = inputs / 'openvr'
    sdk.mkdir()
    shutil.copy2(headers[0], sdk / 'openvr.h')
    shutil.copy2(license_file, sdk / 'LICENSE')
    binary = output / 'pose-watch'
    command[-3] = str(source)
    command[-1] = str(binary)
    with (output / 'build.log').open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    if int.from_bytes(binary.read_bytes()[18:20], 'little') != 183:
        raise RuntimeError('Expected an AArch64 observer ELF')
    receipt = {'display_guard': guard, 'original_source_sha256': digest(original), 'patched_source_sha256': digest(source),
               'patch_sha256': digest(patch), 'compiler_template_sha256': digest(manifest),
               'openvr_header_sha256': digest(headers[0]), 'openvr_license_sha256': digest(license_file),
               'binary_sha256': digest(binary), 'compiler_argv': command,
               'reference_binary_matches': digest(binary) == lock['reference_binary']['sha256'],
               'device_executed': False, 'installed': False}
    (output / 'build-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({key: value for key, value in receipt.items() if key != 'compiler_argv'}, indent=2))


if __name__ == '__main__':
    main()
