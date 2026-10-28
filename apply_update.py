#!/usr/bin/env python3
"""Restore original full FMS dashboard layout, feed it physical SSH/map data.

Preserves local Nav2 files, DB, .env and every existing modified file via backup.
Does not reset Git or install dependencies. Run against existing FMS project root.
"""
from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project_root', type=Path)
    args = parser.parse_args()
    root = args.project_root.expanduser().resolve()
    source = Path(__file__).resolve().parent

    config_path = root / 'backend/app/core/config.py'
    main_path = root / 'backend/app/main.py'
    ui_path = root / 'frontend/src/components/fleet-console.tsx'
    api_path = root / 'backend/app/api/physical.py'
    for required in (config_path, main_path, ui_path):
        if not required.is_file():
            raise SystemExit(f'ERROR: required FMS file not found: {required}')
    if not (root / 'frontend/package.json').is_file():
        raise SystemExit('ERROR: frontend/package.json missing. Restore project frontend before applying.')

    config = config_path.read_text()
    main = main_path.read_text()

    if 'physical_ssh_targets:' not in config:
        needle = '    auto_tick: bool = True\n'
        if needle not in config:
            raise SystemExit('ERROR: cannot safely extend backend config; expected auto_tick setting missing')
        config = config.replace(
            needle,
            '    physical_ssh_targets: str = ""\n'
            '    physical_ssh_timeout_seconds: float = 3.0\n'
            + needle,
            1,
        )
    elif 'physical_ssh_timeout_seconds:' not in config:
        config = config.replace(
            '    physical_ssh_targets: str = ""\n',
            '    physical_ssh_targets: str = ""\n'
            '    physical_ssh_timeout_seconds: float = 3.0\n',
            1,
        )
        if 'physical_ssh_timeout_seconds:' not in config:
            raise SystemExit('ERROR: SSH timeout setting cannot be inserted safely')

    if 'from app.api.physical import router as physical_router' not in main:
        needle = 'from app.api.fleet import router as fleet_router\n'
        if needle not in main:
            raise SystemExit('ERROR: expected fleet import not found in backend/app/main.py')
        main = main.replace(needle, needle + 'from app.api.physical import router as physical_router\n', 1)
    if 'app.include_router(physical_router)' not in main:
        needle = '    app.include_router(fleet_router)\n'
        if needle not in main:
            raise SystemExit('ERROR: expected fleet router registration not found')
        main = main.replace(needle, needle + '    app.include_router(physical_router)\n', 1)

    planned = {
        config_path: config,
        main_path: main,
        ui_path: (source / 'frontend/src/components/fleet-console.tsx').read_text(),
        api_path: (source / 'backend/app/api/physical.py').read_text(),
    }
    suffix = datetime.now().strftime('.backup_before_real_fleet_%Y%m%d_%H%M%S')
    for path, content in planned.items():
        if path.exists() and path.read_text() == content:
            print('UNCHANGED:', path.relative_to(root))
            continue
        if path.exists():
            backup_path = path.with_name(path.name + suffix)
            if backup_path.exists():
                raise SystemExit(f'ERROR: backup already exists: {backup_path}')
            shutil.copy2(path, backup_path)
            print('BACKUP:', backup_path.relative_to(root))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        print('UPDATED:', path.relative_to(root))

    print('SUCCESS: original dashboard sections restored with physical SSH and physical-site map data.')
    print('No data files, task history, existing ROS/Nav2 configuration, or .env were changed.')


if __name__ == '__main__':
    main()
