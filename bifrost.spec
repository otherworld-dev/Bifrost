# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec file for Bifrost robot control application."""

import sys
from pathlib import Path

block_cipher = None
src_dir = Path(SPECPATH)

# Read-only resources bundled inside the exe (extracted to temp dir)
# AND default configs (copied to Bifrost_Data/ on first launch)
datas = [
    # STL mesh files (read-only)
    (str(src_dir / 'STLs'), 'STLs'),
    # Default config files (copied to data dir on first run)
    (str(src_dir / 'dh_parameters.json'), '.'),
    (str(src_dir / 'gripper_calibration.json'), '.'),
    (str(src_dir / 'coordinate_frames.json'), '.'),
]

# Add optional config files if they exist
for optional in ['home_position.json', 'park_position.json']:
    if (src_dir / optional).exists():
        datas.append((str(src_dir / optional), '.'))

a = Analysis(
    ['bifrost.py'],
    pathex=[str(src_dir)],
    binaries=[],
    datas=datas,
    hiddenimports=[
        'OpenGL.platform.win32',
        'OpenGL.arrays.vbo',
        'OpenGL.arrays.numpymodule',
        'OpenGL.arrays.arraydatatype',
        'OpenGL.arrays.formathandler',
        'OpenGL.converters',
        'scipy.spatial.transform._rotation',
        'scipy._cyutility',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'OpenGL_accelerate',
        'scipy.integrate',
        'scipy.stats',
        'scipy.optimize',
        'scipy.interpolate',
        'scipy.signal',
        'scipy.ndimage',
        'scipy.io',
        'scipy.fft',
        'pytest',
        'setuptools',
        '_pytest',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='bifrost',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
