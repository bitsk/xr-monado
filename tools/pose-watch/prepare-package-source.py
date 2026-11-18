#!/usr/bin/env python3
"""Prepare the pinned external observer for the canonical Debian package."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
recipe = Path(__file__).resolve().parent
lock = json.loads((recipe / 'pose-watch-openkylin.source.json').read_text())
if not args.source.is_file() or sha(args.source) != lock['original_source']['sha256']:
    parser.error('XR_POSE_WATCH_SOURCE must provide the exact pinned external observer')
sdk = recipe.parents[1] / 'src/external/openvr_includes'
for name, key in (('openvr.h', 'header_sha256'), ('LICENSE', 'license_sha256')):
    if sha(sdk / name) != lock['openvr_sdk'][key]:
        parser.error('Canonical OpenVR SDK input differs from the observer lock')
args.output.mkdir(parents=True, exist_ok=True)
source = args.output / 'pose-watch.cpp'
shutil.copyfile(args.source, source)
for entry in (lock['patch'],):
    patch = recipe / entry.get('path', entry.get('patch', ''))
    if sha(patch) != entry.get('sha256', entry.get('patch_sha256')):
        parser.error('Observer patch differs from its lock')
    subprocess.run(['patch', '--batch', '--fuzz=0', '--no-backup-if-mismatch',
                    '-p1', '--input', str(patch)], cwd=args.output, check=True)
    if sha(source) != entry['patched_source_sha256']:
        parser.error('Prepared observer source differs from its lock')
guard = lock['display_guard']
header = recipe / guard['header']
if sha(header) != guard['header_sha256']:
    parser.error('Display guard header differs from its lock')
shutil.copyfile(header, args.output / guard['header'])
(args.output / 'inputs.json').write_text(json.dumps({
    'source_sha256': sha(args.source), 'prepared_sha256': sha(source),
    'openvr_header_sha256': sha(sdk / 'openvr.h'),
    'device_executed': False}, indent=2) + '\n')
