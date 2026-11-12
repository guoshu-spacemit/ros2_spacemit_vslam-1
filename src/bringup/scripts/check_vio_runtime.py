#!/usr/bin/env python3
"""Resolve application components before hardware startup, not at first Track."""
import argparse
import ctypes
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mapping', choices=('on', 'off'), default='off')
    args = parser.parse_args()
    names = ['vio', 'esdf_drm']
    # Vendor EP uses initial-exec TLS and is preloaded by its actual launch.
    # Do not late-dlopen it into this Python process as a false ABI test.
    if args.mapping == 'on':
        path = Path(get_package_prefix('las2')) / 'lib/liblas2_component.so'
        result = subprocess.run(['ldd', str(path)], text=True, capture_output=True, check=True)
        if 'not found' in result.stdout:
            raise RuntimeError(result.stdout)
    handles = []
    for name in names:
        path = Path(get_package_prefix(name)) / f'lib/lib{name}_component.so'
        handles.append(ctypes.CDLL(str(path), mode=os.RTLD_NOW | os.RTLD_LOCAL))
    loaded = {Path(line.split()[-1]).resolve()
              for line in Path('/proc/self/maps').read_text().splitlines()
              if len(line.split()) >= 6 and line.split()[-1].startswith('/')}
    sdk_prefix = os.environ.get('SPACEMIT_ROBOT_SDK_PREFIX')
    sdk_library_dir = str(Path(sdk_prefix) / 'lib') if sdk_prefix else None
    expected = os.environ.get('SPACEMIT_VSLAM_LIBRARY_DIR', sdk_library_dir)
    if expected:
        library_name = 'libspacemit_vslam.so'
        selected = {p for p in loaded if p.name == library_name}
        if selected != {(Path(expected) / library_name).resolve(strict=True)}:
            raise RuntimeError(f'Unexpected spacemit_vslam resolution: {selected}')
    expected_mapping = os.environ.get('SPACEMIT_MAPPING_LIBRARY_DIR', sdk_library_dir)
    if expected_mapping:
        selected = {p for p in loaded
                    if p.name == 'libspacemit_mapping.so' or p.name.startswith('libspacemit_mapping_')}
        if not selected or any(p.parent != Path(expected_mapping).resolve() for p in selected):
            raise RuntimeError(f'Unexpected spacemit_mapping resolution: {selected}')
    strict_root = os.environ.get('SPACEMIT_ROBOT_RUNTIME_STRICT_ROOT')
    if strict_root:
        allowed = [Path(strict_root).resolve(), Path('/usr'), Path('/opt/ros'), Path('/lib')]
        outside = [str(p) for p in loaded if not any(p.is_relative_to(root) for root in allowed)]
        if outside:
            raise RuntimeError(f'Runtime still depends on external workspaces: {outside}')
    print('Runtime symbol/library resolution: PASS (no hardware opened)')


if __name__ == '__main__':
    main()
