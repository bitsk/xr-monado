#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exec the materialized native example without changing its SteamVR PID."""
import json
import os
from pathlib import Path
import time

directory = Path(__file__).resolve().parent
config = json.loads((directory / 'launch.json').read_text())
environment = dict(os.environ)
environment.update(json.loads(Path(config['compatibility_profile']).read_text()))
environment.update(config.get('environment', {}))
environment.update({
    'XR_RUNTIME_JSON': config['openxr_runtime'],
    'VK_DRIVER_FILES': config['vulkan_icd'],
    'DISABLE_GAMESCOPE_WSI': '1',
    'DISABLE_VULKAN_FDM_INJECTION_LAYER': '1',
})
environment.pop('ENABLE_GAMESCOPE_WSI', None)
control = Path(config['control_file'])
control.parent.mkdir(parents=True, exist_ok=True)
control.unlink(missing_ok=True)
if config.get('log_directory'):
    logs = Path(config['log_directory'])
    logs.mkdir(parents=True, exist_ok=True)
    filename = time.strftime('%Y%m%dT%H%M%S-', time.gmtime()) + str(os.getpid()) + '.log'
    with (logs / filename).open('w') as log:
        os.dup2(log.fileno(), 1)
        os.dup2(log.fileno(), 2)
binary = directory / 'godot-native'
arguments = [str(binary), '--rendering-method', 'mobile',
             '--rendering-driver', 'vulkan', '--display-driver', 'x11',
             '--xr-mode', 'on']
os.execve(binary, arguments, environment)
