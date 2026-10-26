# Phase 2.1 E2E 보고

전체 판정: **PASS**. 단일 Gazebo Pinky가 Nav2 goal을 받아 움직였고, map 프레임 pose를 관측한 뒤 FMS task가 `COMPLETED`가 된 로그가 있다.

판정에 쓴 주행은 두 번째 시도다. 첫 시도는 Nav2가 성공했다고 보고했지만 관측 오차가 0.257 m라 FMS가 `COMPLETED`로 올리지 않았다. 그 기록을 성공으로 바꾸지 않는다.

시각은 KST(UTC+9)다. 원문 타임스탬프는 UTC다.

## 1. 시뮬레이터

| 항목 | 값 |
| --- | --- |
| 월드 | `pinky_factory.world` |
| 스폰 | `ros2 launch pinky_gz_sim launch_sim.launch.xml`. 엔티티 생성 성공. Gazebo가 모델 이름 `pinky`를 만듦 |
| Gazebo | Sim Server / GUI 8.15.0. 서버 인자는 업스트림 그대로 `-r -s` |
| 브리지 | `/odom`, `/scan`, `/tf`, `/cmd_vel`, `/clock` |
| 스폰 직후 odom | `frame_id=odom`, `child_frame_id=base_footprint`, xy 약 0, 0 |
| scan | `frame_id=rplidar_link`, 거리 샘플 수신 |
| TF, Nav2 전 | `odom -> base_footprint -> base_link` 와 센서 프레임. `map` 없음 |
| Nav2 | `ros2 launch pinky_navigation gz_bringup_launch.xml`. 맵 기본값 `my_map.yaml` (원점 -2.05, -1.62, 공장 벽 x=±2, y=±1.5와 맞음) |
| Lifecycle | `/amcl`, `/map_server`, `/bt_navigator`, `/controller_server`, `/planner_server`, `/velocity_smoother` 모두 `active [3]` |
| 액션 서버 | `/navigate_to_pose` |
| map TF | `map -> base_footprint` 가 초기 pose 0, 0 근처로 관측됨 |
| 격리 | 도메인 73, localhost only, Cyclone DDS. API 프로세스는 DDS에 없음 |
| 실물 런치 | 실행하지 않음 |

업스트림 런치는 네임스페이스가 비어 있고 `/cmd_vel` 이 Gazebo 브리지로 간다. 에이전트는 절대 이름 `/navigate_to_pose` 로 goal을 보낸다. FMS robot id를 ROS 네임스페이스로 쓰지 않는다. `pinky-sim` 은 ROS 이름 규칙을 통과하지 않고, 그렇게 나누면 cmd_vel이 브리지와 어긋난다.

FMS는 `/cmd_vel` 을 발행하지 않는다. 속도는 Nav2 velocity smoother가 브리지로 보낸다.

## 2. 완료된 주행

로그: [phase2-1-logs/agent-attempt2.json](phase2-1-logs/agent-attempt2.json), [phase2-1-logs/fms-task-completed.json](phase2-1-logs/fms-task-completed.json), [phase2-1-logs/nav2-result-lines.txt](phase2-1-logs/nav2-result-lines.txt).

| 필드 | 값 |
| --- | --- |
| Task ID | `20a239aa-4de1-4f0c-8869-c3c6862c400b` |
| Execution ID | `4d176a73-025e-41cc-b400-087f85a3dbf7` |
| Owner epoch | 1 |
| FMS goal ID | `738ed8b4-80e3-40ce-9373-464b80318ea5` |
| Nav2 action goal UUID | `6ddc0c69-9f9b-48cc-9147-9e9747a6f736` |
| 목표 | map, x=0.7, y=0.2, yaw=0, label `factory-east` |
| Goal 전송 | 2026-10-26 14:17:09.081 KST |
| Goal accepted | 2026-10-26 14:17:09.082 KST |
| Goal result | 2026-10-26 14:17:27.504 KST. Nav2 status 4, outcome `succeeded`. 피드백 1639회 |
| 관측 pose | frame `map`, child `base_footprint`, x=0.466968, y=0.116200, yaw=0.095473. stamp 382.820 s |
| 거리 오차 | 0.247642 m. 허용 0.25 m |
| 이동량 | 등록 pose (-0.059, 0.509)에서 관측 pose까지 약 0.656 m |
| FMS 최종 상태 | `COMPLETED`. disposition `matched_pose_lease_and_owner` |
| 에이전트 종료 | 0. `task_completed=true`. 그 다음 세션 stop |

Nav2 로그: `bt_navigator: Goal succeeded` (로그 줄 283).

이 UUID는 테스트 fixture `fixture-action-id` 가 아니다. FMS goal id와 Nav2 UUID는 다르다.

완료 조건은 Nav2 status만이 아니다. pose가 `map`에서 읽혔고, 목표와의 평면 거리가 `pose_tolerance_m` 0.25 이하다.

## 3. 첫 시도. 완료로 세지 않음

로그: [phase2-1-logs/agent-attempt1.json](phase2-1-logs/agent-attempt1.json).

| 필드 | 값 |
| --- | --- |
| Task ID | `ebe61c3b-62a1-4be1-abbf-923b1504f966` |
| Execution ID | `0986e4b3-e795-404f-8b3f-ab2848fa15fd` |
| Owner epoch | 1 |
| FMS goal ID | `72dbb25b-76ec-46b9-8b0a-06a147059f20` |
| Nav2 UUID | `3bf5d1a2-df98-4ba6-bd38-c700547a8c1d` |
| 목표 | map (0.0, 0.8) |
| 전송 / accepted / result | 14:12:42.512 / 14:12:42.518 / 14:12:59.666 KST |
| Nav2 | status 4, `Goal succeeded` (로그 줄 263), 피드백 1714회 |
| 관측 pose | map, base_footprint, x=-0.034990, y=0.545009 |
| 거리 오차 | 0.257380 m |
| FMS | goal disposition `observation_mismatch`. 이후 `RECOVERY_REQUIRED`, 이유 `heartbeat_timeout`. `completion_evidence` 없음 |

오차가 허용을 넘어서 `COMPLETED`가 아니다. 결과 직후 샘플이 아직 이동 중이었고, 나중에 odom y는 0.59 근처까지 갔다. 그 나중 위치를 소급해서 완료로 쓰지 않았다. 두 번째 시도는 결과 이후 2초 안의 마지막 TF를 쓴다.

## 4. 장애

| 시나리오 | 판정 | 근거 |
| --- | --- | --- |
| Nav2 goal 거부 | MOCK_PASS. 라이브 NOT_RUN | pytest가 reject를 `FAILED`로 고정. 라이브에서 맵 밖 목표는 거부가 아니라 abort였다 |
| Nav2 goal 실패 | 라이브 PASS | task `dba3779d-284f-42b6-8255-3b82b99262ed`, 목표 (3, 0). Nav2 UUID `cd751524-b5f6-4277-9f13-3f5dee555b9a`. status 6 `aborted`. FMS `FAILED` `nav2_goal_aborted`. 로그 [agent-abort.json](phase2-1-logs/agent-abort.json). `Goal failed` |
| 같은 goal 중복 claim | 라이브 PASS | Nav2 클라이언트가 떠 있는 동안 두 번째 claim `send_navigate_to_pose=false`, `duplicate_send`, goal 행 1 |
| 에이전트 세션이 프로세스보다 오래 남음 | 라이브에서 재현 후 수정 | 완료 에이전트가 stop 하지 않아 다음 프로세스가 409. 그 task `645f96dd-bdc9-463a-a92d-5c79d9a4a871` 는 claim만 되고 Nav2 goal은 없었다. `RECOVERY_REQUIRED` `heartbeat_timeout`. 운영자가 `FAILED`로 닫았고 goal을 다시 보내지 않았다. 에이전트는 이제 종료 시 stop 한다 |
| Nav2 goal 취소 | MOCK_PASS. 라이브 NOT_RUN | pytest: 취소 요청 후 canceled와 신뢰 pose면 `CANCELED`. 요청 없는 cancel은 `RECOVERY_REQUIRED` |
| FMS 재시작이 goal 종료로 추정하지 않음 | MOCK_PASS. 주행 중 재시작 NOT_RUN | pytest `test_mock_fms_restart_does_not_assume_the_goal_finished` |
| stale epoch | MOCK_PASS. 라이브 NOT_RUN | pytest. 상태 유지 |
| lease 만료 후 success | MOCK_PASS. 라이브 NOT_RUN | pytest는 `RECOVERY_REQUIRED`. 라이브 첫 시도의 `heartbeat_timeout`은 pose 불일치 뒤에 난 복구이고, 만료 후 success를 완료로 받은 사례가 아니다 |
| 통신 단절 | MOCK_PASS. 주행 중 kill NOT_RUN | pytest는 disconnected를 `RECOVERY_REQUIRED`로 두고 goal 행을 늘리지 않는다 |

실행 중인 goal의 종료를 확인하지 못한 상태에서는 재전송하지 않았다. 맵 밖 실패와 완료 주행은 Nav2가 종료 상태를 돌려준 뒤에만 FMS에 올렸다.

Abort 주행의 DB `nav2_action_goal_id` 는 비어 있다. 그 실행은 accept 이벤트 없이 result만 올렸고, 당시 코드는 UUID를 accept에서만 저장했다. 에이전트 JSON에 실제 UUID가 있다. 이후 코드는 검증을 통과한 result에도 같은 UUID를 저장한다. 그 경로는 단위 테스트로 확인했고, 같은 abort를 다시 주행하지는 않았다.

## 5. 테스트

unsourced `cd backend && pytest`: **50 passed**.

- 기존 49개 회귀와 Phase 2 계약: PASS
- 추가 단위 테스트 `test_unit_action_uuid_and_yaw_are_not_invented`: 빈 UUID는 None. 목적지 좌표를 pose로 만들지 않는 헬퍼만 검사
- GAZEBO_E2E: 위 2절의 task가 PASS. pytest 수집 항목은 아니다

## 6. Phase 3

단일 Pinky의 Nav2 goal UUID, 이동, pose, FMS `COMPLETED`가 같은 주행 로그에 있다. Phase 2 보고가 두었던 “단일 로봇 로그가 생기기 전에는 Gazebo 다음 단계로 가지 않는다”는 조건은 해소되었다.

Phase 3 자체를 이 단계에서 실행하지 않았다. 로봇 3대 프로세스 격리와 다중 Gazebo 주행은 여전히 미실행이다. 실물 Pinky, 모터, 승강기, 자동문은 BLOCKED다.
