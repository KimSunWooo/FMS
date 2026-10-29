# FMS ROS 2 Topic 전환 보고서

기준일: 2026-10-29. 실물 Pinky-1에 Task를 보내지 않았다. Nav2 Goal과 `/cmd_vel`도 발행하지 않았다.

## 1. 기존 SSH Task 전달

`FleetTaskService`가 payload를 만들고 `FleetDispatcher`가 Paramiko SSH로 `pinky_task_receiver.py`를 실행했다. JSON을 stdin에 넣고 stdout 마지막 줄을 ACK로 봤다. 기본 `FMS_FLEET_DISPATCH_MODE`는 `ssh`였다. Publish와 저장 ACK를 구분하는 Topic은 없었다.

## 2. 변경 후 ROS Topic 구조

```text
FastAPI
  Task Manager / Scheduler / Reservation Manager
  ros_task_gateway  (파일 IPC, rclpy 없음)
        |
        v
pinky-1 Domain Worker
  ROS_DOMAIN_ID=24
  fms_task_gateway
        |  /fms/task/command, /fms/task/cancel
        v
fms_task_receiver
        |  저장 후 /fms/task/ack, /fms/task/cancel_ack, /fms/robot/status
        v
Gateway result 파일
        v
ACKNOWLEDGED 또는 CANCELLED
```

Pinky-2·3은 virtual이다. Gateway를 띄우지 않고 `channel=simulated`로만 수신을 기록한다. 이 값은 DDS ACK가 아니다.

`ssh`와 `local`은 남는다. `ros_topic`이 실패해도 SSH로 다시 보내지 않는다.

## 3. 수정 파일

- `backend/app/core/config.py` — 기본 모드 `ros_topic`, Topic 이름과 timeout
- `backend/app/services/fleet_task_service.py` — 모드 분기, Cancel ACK의 `ros_topic`, 화면용 `transport`
- `backend/app/services/domain_supervisor.py` — Worker 환경에 모드와 Topic 이름
- `backend/app/services/domain_worker.py` — 실물 로봇일 때만 Gateway 프로세스
- `backend/app/services/ros_task_gateway.py` — API 쪽 IPC
- `robot_receiver/pinky_task_receiver.py` — 같은 Task의 재ACK에 `command_id`만 갱신
- `robot_receiver/task_codec.py`, `ros_messages.py`, `ros_task_node.py`, `ros_gateway_node.py`
- `scripts/run_ros_gateway.sh`, `scripts/run_task_receiver.sh`
- `frontend/src/components/fleet-task-panel.tsx`
- `README.md`, `docs/FMS_Operation_Guide.md`, `docs/FMS_Communication_API_Specification.md`
- `backend/tests/test_ros_topic_contract.py`, `backend/tests/test_ros_topic_dds.py`
- `backend/tests/test_static_isolation.py` — ROS 진입점은 subprocess로 두고 API import는 금지

## 4. ROS Interface Package

`ros_ws/src/fms_msgs`. Jazzy에서 `colcon build --packages-select fms_msgs` 성공.

메시지: `TaskCommand`, `TaskAck`, `TaskCancel`, `TaskCancelAck`, `RobotStatus`, `Waypoint`.

Hash 원문은 `canonical_payload` JSON이다. `command_id`는 hash에 넣지 않는다. Waypoint heading이 없으면 `orientation_is_planned=false`이고 quaternion identity는 계획된 방향이 아니다.

## 5. Publisher / Subscriber

| 노드 | 발행 | 구독 |
|---|---|---|
| `fms_task_gateway` | command, cancel | ack, cancel_ack, robot status |
| `fms_task_receiver` | ack, cancel_ack, status 1 Hz | command, cancel |

QoS는 RELIABLE, VOLATILE, KEEP_LAST, depth 10이다.

## 6. Domain Worker

Worker 프로세스가 Registry의 Domain을 자식 환경에 넣는다. API의 `ROS_DOMAIN_ID`는 바꾸지 않는다. `kind=physical`이고 연결이 켜져 있고 모드가 `ros_topic`일 때만 `run_ros_gateway.sh`를 실행한다. Pinky-2·3 상태는 `physical_transport_disabled`다. `receiver_ready`는 RobotStatus 파일이 ready이고 `task_execution_enabled=false`일 때만 참이다.

## 7. Receiver ACK

로컬 DDS Domain 80에서 `fms_task_receiver`가 command를 저장하고 ACK를 냈다. `receiver_id`는 `pinky-fms-receiver`, `execute`는 false였다.

IPC 시험은 Gateway 없이 codec이 ACK 파일을 쓰는 경로다. Domain 80 시험과 구분한다.

Domain 24에서 `ros2 topic info /fms/task/ack`는 `Unknown topic`이었다. 실물 ACK는 BLOCKED다.

## 8. Cancel ACK

Domain 82에서 저장 직후 cancel을 보냈다. ACK `outcome=cancel_ack`, inbox `executable=false`, payload hash는 유지됐다. FMS 통합 시험은 `CANCELLED`와 `ack_status=CANCEL_ACKNOWLEDGED`를 확인했다. Cancel ACK 전에는 `CANCELLED`로 두지 않는다.

## 9. QoS

RELIABLE은 유실을 줄인다. VOLATILE은 늦게 붙은 노드가 지난 Task를 다시 받지 않게 한다. TRANSIENT_LOCAL은 쓰지 않는다. 성공은 Receiver 저장과 ACK로만 판정한다.

## 10. 재시도, 중복, Timeout

- `command_id`로 ACK를 짝짓는다. 재시도는 새 command id와 같은 canonical payload다.
- 같은 Task와 같은 hash는 파일을 다시 쓰지 않고 ACK를 다시 낸다.
- 같은 Task ID와 다른 hash는 `rejected`이며 재시도하지 않는다.
- Robot ID, hash, `execute=true` 거절도 재시도하지 않는다.
- Subscriber가 없으면 Gateway가 timeout을 쓴다. 기본 대기 8초, 최대 2회.
- Timeout이면 `FAILED`, `ack_status=NOT_RECEIVED`, 예약 해제. SSH fallback 없음.
- 프로세스 재시작 때 `DISPATCHING`은 `FAILED`이고 사유는 `dispatch interrupted by process restart`다.
- `CANCEL_PENDING`은 재시작만으로 `CANCELLED`가 되지 않는다.

## 11. Backend 테스트

`cd backend && env -u ROS_DISTRO -u ROS_DOMAIN_ID -u RMW_IMPLEMENTATION -u FMS_PHYSICAL_MAP_YAML PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv/bin/pytest -q -p pytest`

152 PASS. 여기에는 기존 예약, 취소, 보관, Domain Worker, SSH 비밀번호 없음 실패, 가상 로봇 simulated 수신이 포함된다.

DDS 시험 Domain은 80, 81, 82, 83이다. 24, 25, 26에 시험 명령을 발행하지 않았다.

## 12. Frontend 테스트

`frontend/node_modules/.bin/tsc --noEmit` PASS.

`FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack` PASS. 빌드가 넣은 `.next-verify/types/**/*.ts` include는 `tsconfig.json`에서 제거했다.

Task 상세에 전송 방식, Domain, Topic, Publish 상태, ACK, Receiver, 실행, 완료를 나눴다. ACK 없이 전달 성공으로 보이지 않는다.

## 13. 기존 기능

A*, Yen, UTC 예약, 재계획, Task 취소, 보관, 지도 `1f_map_f`, 로봇별 Domain 24·25·26을 유지했다. 테스트 DB와 운영 DB의 Task 행을 지우지 않았다. `execute=true`는 전달 전에 거절한다. Collision Monitor와 5cm Goal 코드는 바꾸지 않았고 5cm POST도 하지 않았다.

## 14. 실물 Pinky 연결

Domain 24에 `/fms/task/ack` Topic이 없었다. 실물 Receiver ACK는 BLOCKED다. 로컬 Domain 80 ACK와 구분한다.

## 15. 미해결

- Pinky-1에 Receiver를 올리지 않았다. 실물 ACK는 로봇 쪽 `run_task_receiver.sh`가 Domain 24에서 뜬 뒤에만 확인할 수 있다.
- Gateway와 Receiver를 운영 PC에서 재시작했을 때의 DDS 복구는 상태 규칙과 중복 ACK 시험으로 확인했다. 실물 프로세스 재시작 시험은 BLOCKED다.
- `builtin_interfaces/Time`과 inbox ISO 시각은 같은 순간이지만 문자열 반올림은 다를 수 있다. Hash는 이 시각을 포함하지 않는다.

## 16. 이후 Nav2 Action에 필요한 인터페이스

이번 단계의 ACK는 수신과 저장이다. 이동 완료가 아니다.

다음에 필요한다.

- `execute=true`를 허용하는 별도 승인. 지금은 Gateway가 false로 덮어쓴다.
- Receiver가 Nav2 `NavigateToPose`를 호출하는 지점. FMS는 Goal을 직접 보내지 않는 현재 경계를 유지할 수 있다.
- 진행과 완료를 ACK와 다른 Topic 또는 Action result로 받을 것. `ACKNOWLEDGED`를 완료로 쓰지 않는다.
- 취소 시 이미 실행 중인 Goal의 cancel. 지금은 저장본의 `executable=false`만 바꾼다.
- Pinky-2·3을 physical로 바꿀 때의 Domain 25·26 Receiver. 가상 simulated 수신과 섞지 않는다.
