# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_all

datas = []
binaries = []
hiddenimports = []
tmp_ret = collect_all('torch')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('torchvision')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('torchaudio')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]

tmp_ret = collect_all('chumpy')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]


block_cipher = None


a = Analysis(
    ['axio_live_test.py'],
    pathex=[],
    binaries = binaries + [
    ('C:\\Users\\ipop1\\anaconda3\\envs\\PIP_SMPL\\Lib\\site-packages\\torch\\lib\\*.dll', '.')
],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(

    pyz,

    a.scripts,

    [],

    exclude_binaries=True,   # 🔹 이 부분이 onedir용 포인트

    name='axio_live_test',

    debug=False,

    bootloader_ignore_signals=False,

    strip=False,

    upx=False,               # 🔹 우선 False 권장

    console=True,

)



coll = COLLECT(

    exe,

    a.binaries,

    a.zipfiles,

    a.datas,

    strip=False,

    upx=False,

    name='axio_live_test'

)