"""Offline contract test: actual PGM bytes -> PNG, route shapes, patch idempotence."""
import importlib.util
import io
import os
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

from fastapi import FastAPI
from fastapi.testclient import TestClient
from PIL import Image

ROOT = Path(__file__).resolve().parent
MOD = ROOT / 'backend/app/api/physical.py'

spec = importlib.util.spec_from_file_location('physical_under_test', MOD)
physical = importlib.util.module_from_spec(spec)
spec.loader.exec_module(physical)

with tempfile.TemporaryDirectory() as d:
    temp = Path(d)
    pgm = temp / '1f_map.pgm'
    pgm.write_bytes(b'P5\n# test map\n3 2\n255\n' + bytes([0, 127, 255, 255, 90, 0]))
    yml = temp / '1f_map.yaml'
    yml.write_text('image: 1f_map.pgm\nresolution: 0.01\norigin: [0.0, 0.0, 0.0]\n')
    os.environ['FMS_PHYSICAL_MAP_YAML'] = str(yml)

    app = FastAPI()
    class Settings:
        physical_ssh_targets='pinky-1=pinky@192.168.0.83'
        physical_ssh_timeout_seconds=1
    app.state.settings = Settings()
    app.include_router(physical.router)
    with TestClient(app) as client:
        m = client.get('/api/physical/map')
        assert m.status_code == 200, m.text
        assert m.json()['width_m'] == 0.03
        assert m.json()['height_m'] == 0.02
        assert m.json()['source'] == '1f_map.yaml'
        pic = client.get('/api/physical/map/image')
        assert pic.status_code == 200, pic.text
        im = Image.open(io.BytesIO(pic.content))
        assert im.size == (3, 2)
        assert im.getpixel((0,0)) == 0
        assert im.getpixel((2,0)) == 255
        assert im.getpixel((1,1)) == 90
        assert physical.parse_targets(Settings.physical_ssh_targets) == [('pinky-1','pinky@192.168.0.83')]

    checkout = temp / 'repo'
    for name, source in {
        'backend/app/core/config.py': 'class Settings:\n    auto_tick: bool = True\n',
        'backend/app/main.py': 'from app.api.fleet import router as fleet_router\n\ndef app():\n    app.include_router(fleet_router)\n',
        'frontend/src/components/fleet-console.tsx': 'old SSH-only dashboard',
        'frontend/package.json': '{"name":"stub"}',
    }.items():
        p = checkout / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(source)
    script = ROOT / 'apply_update.py'
    for _ in range(2):
        subprocess.run([sys.executable, str(script), str(checkout)], check=True, capture_output=True)
    assert (checkout/'frontend/src/components/fleet-console.tsx').read_text() == (ROOT/'frontend/src/components/fleet-console.tsx').read_text()
    assert (checkout/'backend/app/api/physical.py').exists()
    assert (checkout/'backend/app/main.py').read_text().count('app.include_router(physical_router)') == 1
    assert (checkout/'backend/app/core/config.py').read_text().count('physical_ssh_targets:') == 1
print('PASS: Map PNG encodes original PGM pixels; API/map metadata; safe patch apply and rerun')
