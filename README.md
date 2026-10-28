# Care-Move FMS — Full dashboard restoration (real Pinky + original site map)

This patch **restores all original dashboard sections** (robot cards, map, task input,
queue, zones, history, events). It **does not display legacy mock robots, mock positions,
mock tasks, fake ward zone geometry, or artificial completed results**. It shows only
robots that pass noninteractive SSH authentication.

The physical-site map is loaded from:
`<FMS root>/config/maps/physical_site/1f_map.yaml`, with its `.pgm` image. The backend
converts the actual PGM bytes to browser-readable PNG. No replacement/demo map is created.

## Apply on swPC

```bash
python3 /path/to/extracted/fms_restore_full_dashboard_patch/apply_update.py /home/sw/DevSpace/FMS
```

The script backs up modified files using a `.backup_before_real_fleet_...` suffix.
It does **not** delete database rows, reset Git, touch Nav2/map files, or modify `.env`.

## Backend terminal

```bash
cd /home/sw/DevSpace/FMS
export FMS_PHYSICAL_SSH_TARGETS='pinky-1=pinky@192.168.0.83'
export FMS_PHYSICAL_MAP_YAML='/home/sw/DevSpace/FMS/config/maps/physical_site/1f_map.yaml'
export FMS_SEED_FLEET=false
export FMS_AUTO_TICK=false
./scripts/run_api.sh
```

SSH must already work noninteractively from the account **running the backend**:

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes pinky@192.168.0.83 true
```

## Frontend terminal

```bash
cd /home/sw/DevSpace/FMS
./scripts/run_dashboard.sh
```

No need to reinstall packages if the existing frontend is running.

## Verify

```bash
curl -fsS http://127.0.0.1:8741/api/physical/robots | python3 -m json.tool
curl -fsS http://127.0.0.1:8741/api/physical/map | python3 -m json.tool
curl -fsS http://127.0.0.1:8741/api/physical/map/image -o /tmp/fms_actual_map.png
file /tmp/fms_actual_map.png
```

The dashboard is on http://localhost:8742.

## Important boundaries

- SSH login is not ROS / Nav2 / physical motor readiness.
- No fake pose or battery values are fabricated. Robot markers are hidden until
  verified live `map`-frame pose telemetry is integrated.
- Task input and stop controls retain their places in the UI, but are **disabled**
  until a real Nav2 action bridge and a real hardware-stop API are implemented.
  This avoids accidentally submitting jobs to the legacy mock scheduler.
- Existing task/zone/history panels are preserved, but mock content is not read.
- Map pixels are original Nav2 PGM pixel data; selecting a point only fills the X/Y inputs.
- This patch does not alter your ROS_DOMAIN_ID, Nav2 launch, AMCL, or the loaded map.
