# Pinky RobotStatus Pose — 배포·Rollback (승인 후)

**승인 없이 Receiver 재시작/소스 교체를 하지 않는다.**

## 2026-10-29 실측 요약

| 항목 | Pinky (`192.168.0.83`) | swPC | 결과 |
|---|---|---|---|
| ROS Domain | 24 (조사 시) | 24 | 동일 |
| RMW | `rmw_fastrtps_cpp` (관례) | `rmw_fastrtps_cpp` | 동일 |
| RobotStatus 필드 | 7 (pose 없음) | 10 (pose/pose_valid/pose_source) | **불일치** |
| RobotStatus.json sha256 | `107617ad…` | `8a3e0460…` | **불일치** |
| Receiver 프로세스 | **미실행** (마지막 감사 07:40Z) | Gateway/API 미기동 | LIVE 통신 없음 |
| `/tf` `/tf_static` | 없음, `map` 프레임 없음 | 동일 | TF **없음** |
| Nav2/AMCL | 프로세스 없음 | — | Localization **비활성** |

## 배터리 데이터 소스 (실측)

- Domain 24에 `/battery*` Topic **미실행** (bringup/sensor 노드 없음)
- Pinky SDK `pinkylib.Battery` I2C 읽기 가능 (LIVE: ~8.13V → SDK 100%)
- bringup 시 `battery/percent` (Float32 0..100), `battery/voltage`
- sensor_adc 시 `batt_state` (`sensor_msgs/BatteryState`, **percentage=NaN**, voltage만)
- Receiver: Topic 우선, 없으면 `pinkylib:i2c` fallback (전압→SOC 임의 변환 금지, SDK 함수만 사용)

## Dry-run (승인 전, 쓰기 없음)

```bash
# swPC
cd /home/sw/DevSpace/FMS
rsync -avn \
  ros_ws/src/fms_msgs/msg/RobotStatus.msg \
  robot_receiver/ros_task_node.py \
  robot_receiver/ros_messages.py \
  robot_receiver/pose_report.py \
  robot_receiver/battery_report.py \
  robot_receiver/battery_adapter.py \
  robot_receiver/ipc_atomic.py \
  pinky@192.168.0.83:/tmp/fms_pose_preview/
```

## 승인 후 배포 순서

1. Pinky 백업
```bash
ssh pinky@192.168.0.83 'ts=$(date +%Y%m%d_%H%M%S); mkdir -p ~/fms_backup_$ts
  cp -a ~/FMS/ros_ws/src/fms_msgs/msg/RobotStatus.msg ~/fms_backup_$ts/
  cp -a ~/FMS/robot_receiver/*.py ~/fms_backup_$ts/
  echo backup=$HOME/fms_backup_$ts'
```

2. 파일 동기화 (Nav2/Localization 프로세스는 건드리지 않음)
```bash
rsync -av ros_ws/src/fms_msgs/msg/RobotStatus.msg \
  pinky@192.168.0.83:~/FMS/ros_ws/src/fms_msgs/msg/
rsync -av robot_receiver/ros_task_node.py robot_receiver/ros_messages.py \
  robot_receiver/pose_report.py robot_receiver/battery_report.py \
  robot_receiver/battery_adapter.py \
  pinky@192.168.0.83:~/FMS/robot_receiver/
```

3. Pinky에서 fms_msgs만 재빌드
```bash
ssh pinky@192.168.0.83 'bash -lc "
  cd ~/FMS/ros_ws && source /opt/ros/jazzy/setup.bash
  rm -rf build/fms_msgs install/fms_msgs
  colcon build --packages-select fms_msgs --symlink-install
  source install/setup.bash
  ros2 interface show fms_msgs/msg/RobotStatus | tee /tmp/robotstatus_after.txt
"'
```

4. Receiver만 재시작 (실행 방식 확인 후). Nav2/AMCL/bringup은 유지.

5. 검증
```bash
# 양쪽 Domain 24
ros2 interface show fms_msgs/msg/RobotStatus   # pose 필드 일치
timeout 10 ros2 topic echo /fms/robot/status --once
timeout 10 ros2 run tf2_ros tf2_echo map base_footprint
# swPC IPC
cat data/ros_ipc/pinky-1/robot_status.json
```

## Rollback

```bash
ssh pinky@192.168.0.83 'bash -lc "
  B=~/fms_backup_TIMESTAMP   # 배포 시 출력된 경로
  cp -a \$B/RobotStatus.msg ~/FMS/ros_ws/src/fms_msgs/msg/
  cp -a \$B/*.py ~/FMS/robot_receiver/
  cd ~/FMS/ros_ws && source /opt/ros/jazzy/setup.bash
  rm -rf build/fms_msgs install/fms_msgs
  colcon build --packages-select fms_msgs --symlink-install
  # 구버전 Receiver 재기동
"'
```

## 선행 조건 (실위치 LIVE_PASS)

1. swPC/Pinky RobotStatus 타입 일치 (위 배포)
2. Receiver 실행
3. Localization/Nav2가 `map→base_footprint` TF 발행 (현재 **부재** — 무단 재시작 금지)
4. swPC Gateway + Backend API 기동
