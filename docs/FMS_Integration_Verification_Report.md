# FMS 1~5단계 통합 검증

검증일: 2026-10-28. 경로와 예약은 계획 시뮬레이션이다. 실측 주행으로 안전이 검증된 결과가 아니다. 이번 검증에서 Nav2 Goal, DriveOnHeading, cmd_vel은 보내지 않았다.

## 결과 요약

Backend pytest **104 passed**. 이전에 깨지던 `test_sources_do_not_import_rmf_or_ros`는 이제 통과한다. Frontend 프로덕션 빌드는 개발 서버의 `.next`와 분리된 `.next-verify`에서 통과했다.

5cm 상태 조회는 HTTP 200이다. 주행 POST는 실행하지 않았다.

## 수정한 파일

| 파일 | 이유 |
|---|---|
| `backend/app/services/fleet_dispatcher.py` | 로컬 receiver와 SSH 응답이 시간 초과되면 예외가 500으로 나가지 않고 `ack_timeout`으로 실패한다. |
| `backend/app/services/fleet_task_service.py` | 같은 idempotency key의 동시 삽입이 무결성 오류로 끝나면 기존 task를 반환한다. 이미 수신 기록이 있는 `DELIVERED` task는 다시 보내지 않는다. |
| `backend/tests/test_static_isolation.py` | 끊어진 심볼릭 링크에서 테스트가 죽지 않게 하고, FMS 서버가 ROS worker를 import하지 않는지 검사한다. |
| `backend/tests/test_fleet_tasks.py` | 충돌 5종, ACK 시간 초과, 잘못된 hash, 재전송, 재시작, 5cm GET을 분리해 검증한다. |
| `frontend/next.config.ts` | `FMS_NEXT_DIST_DIR`가 있으면 그 디렉터리에 빌드한다. 기본값은 `.next`다. |
| `frontend/.gitignore` | `.next-verify/` |

`config/physical_fleet.json`은 이번 검증 스크립트가 쓰지 않았다. 다만 재시작 전 서버 로그에 `POST /api/fleet-tasks/robots/pinky-2/home`와 `pinky-3/home`이 200으로 남아 있다. 현재 파일의 home은 그 승인 결과인 `(0.425, 1.925)`와 `(0.425, 0.575)`다.

## 1. API와 데이터 모델

기존 mock `/api/tasks`와 fleet `/api/fleet-tasks`는 테이블이 다르다. `POST /api/planning/routes/preview`는 조회 전용이다. 검증 중 preview는 200과 경로 3개를 반환했다.

재시작 후 저장된 task는 완료로 바뀌지 않았다.

| task | 재시작 후 |
|---|---|
| `5b3bc030-0b45-4688-bd97-7618c1102fde` | `ACKNOWLEDGED`, channel `physical`, `actual_motion_verified` false |
| `4ae06120-8ece-4ae4-be43-c4bc9e495b7a` | `DELIVERED`, `SIMULATED_RECEIPT` |
| `e8ee09bc-b889-4158-9697-cce5fa9687d6` | `RESERVED`, 전달되지 않음 |
| `f8c7961f-2acb-4d3d-be71-cd8181c3eb46` | `FAILED`, 사유 `tf_unavailable` |

## 2. 충돌 테스트

각각 별도 pytest로 실행했고 통과했다.

| 종류 | 테스트 | 결과 |
|---|---|---|
| Vertex | `test_vertex_conflict` | PASS. 같은 셀의 겹치는 시간은 충돌. 끝점만 맞닿으면 충돌 아님 |
| Edge | `test_edge_conflict` | PASS. 반대 방향만 충돌. 같은 방향은 충돌 아님 |
| Crossing | `test_crossing_conflict` | PASS. `e-0,0>1,1`와 `e-1,0>0,1` |
| Footprint | `test_footprint_conflict` | PASS. 중심 거리 0.05 m, radius 0.06 m에서 충돌. radius 0이면 아님 |
| Waiting | `test_waiting_conflict` | PASS. 같은 셀에서 기존 점유가 더 길면 vertex와 waiting |

두 로봇이 같은 출발·목적지를 예약하는 `test_second_robot_does_not_share_the_same_reservation`도 PASS다. confirmed 예약 사이의 conflict report는 비어 있고 `collision_safety_verified`는 false다.

## 3. 동시 생성과 예약 원자성

| 테스트 | 결과 |
|---|---|
| `test_concurrent_plans_do_not_duplicate_a_reservation` | PASS. 두 thread가 서로 다른 로봇을 예약. conflict report가 비었다 |
| `test_concurrent_idempotency_key_keeps_one_task` | PASS. 같은 key 4회 동시 생성은 task id 1개 |

SQLite `BEGIN IMMEDIATE` 안에서 기존 예약을 읽고 확정한다. 무결성 오류가 나면 그 key의 기존 행을 돌려준다.

## 4. Pinky-2, Pinky-3 clearance

현재 소스와 `1f_map_f`로 원래 등록 좌표를 계산한 결과다. 이 계산은 파일을 쓰지 않는다.

| 로봇 | 원래 좌표 | 판정 | 가장 가까운 점유 | 후보 (suggestion_only) |
|---|---|---|---|---|
| pinky-2 | `(0.40, 1.90)` | clearance | 0.055 m | `(0.425, 1.925)`, `(0.375, 1.925)`, `(0.325, 1.925)` |
| pinky-3 | `(0.40, 0.50)` | clearance | 0.025 m | `(0.425, 0.575)`, `(0.375, 0.575)`, `(0.325, 0.575)` |

`test_invalid_registry_home_is_not_rewritten`와 `test_home_approval_writes_only_after_validation`은 PASS다. `approve: false`는 422이고, 승인은 검사에 통과한 뒤에만 임시 registry 복사본에 기록된다. 물리 로봇 home 변경은 404다.

라이브 `config/physical_fleet.json`은 위 표의 원래 좌표가 아니다. 재시작 전 프로세스 로그의 승인 POST 200 이후 값은 pinky-2 `(0.425, 1.925)`, pinky-3 `(0.425, 0.575)`다. 이번 검증의 조회는 그 파일을 다시 쓰지 않았다. 현재 좌표는 clearance를 통과한다.

## 5. Pinky-1 receiver

로봇의 `~/fms_task_receiver/inbox/5b3bc030-0b45-4688-bd97-7618c1102fde.json`을 대상으로 수신 프로그램만 다시 실행했다.

| 확인 | 결과 |
|---|---|
| 같은 payload 재전송 | PASS. exit 0, `accepted` true, hash 일치, `execute` false, receiver `pinky-fms-receiver` |
| 같은 task id, 다른 goal | PASS. exit 2, `accepted` false, detail에 different payload. 저장된 hash는 그대로 |
| 재시작 후 task 상태 | PASS. `ACKNOWLEDGED` 유지, 주행 완료로 바뀌지 않음 |

## 6. 중복, 시간 초과, hash, 재시작

| 항목 | 결과 | 근거 |
|---|---|---|
| 실물 중복 전달 | PASS | `test_local_physical_receiver_ack_matches_hash_and_does_not_move`. 두 번째 dispatch의 `received_at`이 같다. 라이브 receiver 재전송도 위 5절 |
| 가상 중복 전달 | PASS | `test_repeated_virtual_dispatch_does_not_send_twice`. 상태는 `DELIVERED`이고 실물 ACK가 아니다 |
| ACK timeout | PASS | `test_ack_timeout_does_not_acknowledge`. 상태는 `FAILED`, `ack_status`는 `NOT_RECEIVED`, 예약은 해제. 실물 SSH를 일부러 멈추는 실험은 NOT RUN |
| 잘못된 hash | PASS | `test_wrong_ack_hash_is_rejected`. receiver stdout의 hash가 다르면 `FAILED`. 라이브 충돌 응답은 5절 |
| 서버 재시작 | PASS | `test_restart_does_not_treat_dispatching_as_delivered`는 중단된 `DISPATCHING`을 `FAILED`로 둔다. `test_acknowledged_task_survives_recovery_without_completion`는 ACK를 유지한다. 라이브 프로세스 재시작 후에도 같은 구분이 유지됐다 |
| 지도 revision | PASS | `test_stale_map_revision_requires_replan` |

## 7. 5cm GET

| 확인 | 결과 |
|---|---|
| GET `/api/physical/tests/pinky-1/forward-5cm` | PASS. HTTP 200. `_map_files` import는 소스에 없다 |
| 재시작 전 body | `state: failed`, detail `Nav2 result status 6`. HTTP 오류가 아니라 이전 주행 시도의 메모리 상태 |
| 재시작 후 body | `state: idle` |
| pytest에서 자동 기동을 끈 경우 | PASS. `test_five_cm_status_get_does_not_start_motion`는 `disabled` |
| 주행 POST | NOT RUN |

## 8. Frontend 빌드

`FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack` 통과. 산출 경로는 `frontend/.next-verify`다. 라우트 `/`와 `/api/physical-5cm`가 포함됐다. 개발 서버의 `.next`는 쓰지 않았다.

## 9. test_static_isolation.py

이전 실패는 테스트가 `simulation/ros_ws/install/.../view_small_house.launch.py`를 읽다 `FileNotFoundError`로 중단된 것이다. 링크 대상 `simulation/ros_ws/src/pinky_pro/src/aws-robomaker-small-house-world/launch/view_small_house.launch.py`는 저장소에 없다. ROS 패키지를 만들어 링크를 고치지 않았다.

수정 내용은 다음과 같다.

- 읽을 수 없는 경로와 `simulation/ros_ws`는 검사에서 제외한다. 그 트리는 Pinky ROS 워크스페이스와 생성된 install 트리라서 `rclpy` import가 정상이다.
- `backend/app`, `robot_agent`, `robot_receiver`, `simulation`의 나머지 Python은 계속 검사한다.
- `nav2_forward_5cm_worker.py`와 `live_tf_worker.py`는 subprocess 진입점이다. API 모듈이 이 둘을 import하면 실패한다. 현재 import는 없다.
- `test_static_isolation_failure_is_a_dangling_symlink`는 그대로 통과한다. 링크는 여전히 끊어져 있다.

결과: `tests/test_static_isolation.py` 3건 PASS.

## 10. 항목별 판정

| 항목 | 판정 |
|---|---|
| 1~2단계 preview, 지도, 로봇 구분 | PASS |
| Vertex / Edge / Crossing / Footprint / Waiting | PASS |
| 동시 task와 예약 원자성 | PASS |
| 가상 home 후보 계산, 무승인 자동 변경 없음 | PASS. 단위 테스트. 라이브 파일은 이미 승인 API로 바뀐 상태 |
| Pinky-1 receiver ACK와 hash 충돌 | PASS |
| 중복 전송, 잘못된 hash, 재시작 복구 | PASS |
| 실물 SSH를 멈춘 ACK timeout | NOT RUN. 코드 경로는 단위 테스트 PASS |
| 5cm GET | PASS |
| 5cm 주행 POST | NOT RUN |
| Frontend 분리 빌드 | PASS |
| 정적 격리 테스트 | PASS. 끊어진 ROS 링크 자체는 미복구 |
| 실물 다중 로봇 안전 | 해당 없음. `collision_safety_verified`는 false |

## 남은 결함

- `view_small_house.launch.py` 대상 파일이 없어 심볼릭 링크는 여전히 깨져 있다. FMS 테스트는 그 경로를 읽지 않는다.
- 라이브 registry는 원래 `(0.40, 1.90)`, `(0.40, 0.50)`이 아니다. 승인 API 호출이 먼저 기록돼 있다. 원래 좌표로 되돌리려면 별도의 승인이 필요하다.
- ACK timeout을 실제 Pinky SSH 세션에서 멈추는 방식으로는 재현하지 않았다.
- 5cm POST의 수정된 분기(지도 경계 검사)는 주행이 발생하므로 라이브에서 호출하지 않았다.
- 계획 예약은 시간이 지나도 실제 통과로 바꾸지 않는다. `RESERVED` task는 재시작 후에도 자동 전달되지 않는다.
