# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for the ORDLAV desktop application.

Build on the platform you are targeting — PyInstaller does not cross-compile,
so a Windows .exe must be produced on Windows (locally, or by the
build-windows GitHub Actions workflow in this repository):

    pip install -r requirements.txt pyinstaller
    pyinstaller packaging/ORDLAV.spec --noconfirm
"""

import os

block_cipher = None
project_root = os.path.abspath(os.path.join(os.getcwd()))

# python-can resolves interfaces by name at runtime, so the backends have to be
# named explicitly or they are left out of the bundle.
can_backends = [
    'can.interfaces.kvaser',
    'can.interfaces.kvaser.canlib',
    'can.interfaces.virtual',
    'can.interfaces.pcan',
    'can.interfaces.pcan.pcan',
    'can.interfaces.vector',
    'can.interfaces.ixxat',
    'can.interfaces.slcan',
    'can.interfaces.socketcan',
    'can.interfaces.serial.serial_can',
    'can.interfaces.udp_multicast',
]

a = Analysis(
    ['../ordlav_app.py'],
    pathex=[project_root],
    binaries=[],
    datas=[],
    hiddenimports=can_backends + [
        'can_frame_processor',
        'orion_radar_visualizer',
        'can_source',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Keep the bundle lean: the analysis stage is not part of the app.
    excludes=['matplotlib', 'pandas', 'tkinter', 'PyQt5.QtWebEngineWidgets'],
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
    name='ORDLAV',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    runtime_tmpdir=None,
    # Windowed app: no console window behind the Qt interface. Use
    # `--mode processor` from a terminal if you want the capture logs.
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
