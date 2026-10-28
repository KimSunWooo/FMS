# FMS 3·4·5단계 구현 보고서

작성 기준: 2026-10-28. 경로 계획과 예약은 계획 시뮬레이션이다. 실측 주행으로 안전이 검증된 결과가 아니다. `collision_safety_verified`는 false다. 계획 속도 0.15 m/s는 설정값이며 Pinky의 측정 속도가 아니다.

## 1. 기존 프로젝트 상태

1·2단계에서 이미 있던 구성은 유지했다.

- 지도 `config/maps/physical_site/1f_map_f.yaml`와 `1f_map_f.pgm`. 186×236 px, 0.01 m/px, origin `(0, 0)`.
- 로봇 등록 `config/physical_fleet.json`. pinky-1은 physical이고 home이 없다. pinky-2 home `(0.40, 1.90)`, pinky-3 home `(0.40, 0.50)`.
- 경로 탐색은 `occupancy_grid.py`, `waypoint_graph.py`, `route_planner.py`, `route_preview.py`를 그대로 호출한다. Planning grid는 0.05 m, robot radius 설정값은 0.06 m, 8방향, corner cutting 금지, Yen 후보 기본 3개, unknown 차단.
- `POST /api/planning/routes/preview`는 조회 전용이다. Task와 예약을 만들지 않는다.
- 기존 mock Task API `POST /api/tasks`와 `tasks` 테이블은 그대로 두었다. 새 작업은 `fleet_tasks` 테이블과 `/api/fleet-tasks`를 사용한다.

작업 전 기준 테스트는 2단계 이후 전체 pytest에서 `test_static_isolation.py::test_sources_do_not_import_rmf_or_ros` 1건만 실패했다. 원인은 `simulation/ros_ws/install/.../view_small_house.launch.py`의 끊어진 심볼릭 링크다. 이번 작업에서 그 테스트를 지우거나 ROS 트리를 수정하지 않았다.

5cm 버튼의 HTTP 500은 `forward_5cm_service.py`가 삭제된 `app.api.physical._map_files`를 import하던 것이 원인이었다. 경계 검사를 `load_geometry`와 `world_to_grid`로 바꿨다. 5cm 주행 POST는 실행하지 않았다.

## 2. 전체 시스템 아키텍처

```
Fleet Console (8742)
  map-frame.ts 좌표 변환
  /fms-api  →  FastAPI (8741)
        ├─ /api/planning/routes/preview     조회 전용 A* / Yen
        ├─ /api/planning/points/inspect     클릭 좌표 검사
        ├─ /api/fleet-tasks                 Task 생성·계획·예약·전달
        └─ /api/physical                    SSH, Nav2, TF, 5cm (기존)
              │
              ▼
        FleetTaskService
          ├─ WaypointGraph / k_shortest_paths   기존 planner 재사용
          ├─ schedule_candidates               시간 슬롯 예약
          └─ fleet_dispatcher
                ├─ virtual  → fms-virtual-receiver (로컬 파일)
                └─ physical → SSH python3 ~/fms_task_receiver/pinky_task_receiver.py
                              저장 후 ACK. Nav2 / cmd_vel 호출 없음
```

예약 시각은 계획 시각이다. 시간이 지났다는 이유만으로 로봇이 그 칸을 통과했다고 보지 않는다. ACK는 수신 확인이지 주행 완료가 아니다. `ACKNOWLEDGED`를 `COMPLETED`로 바꾸지 않는다.

## 3. 수정 및 생성 파일

| 경로 | 목적 |
|---|---|
| `backend/app/models/tables.py` | `fleet_tasks`, `fleet_task_events`, `fleet_reservations` |
| `backend/app/services/fleet_task_machine.py` | 상태 전이 검증 |
| `backend/app/services/fleet_task_service.py` | 생성, 배정, 계획, 예약, 취소, 전달, 재시작 복구 |
| `backend/app/services/reservation_manager.py` | vertex / edge / crossing / footprint / waiting |
| `backend/app/services/fleet_dispatcher.py` | 가상 수신, 로컬 receiver, SSH receiver |
| `backend/app/services/fleet_registry.py` | 승인된 가상 home만 저장 |
| `backend/app/api/fleet_tasks.py` | `/api/fleet-tasks` |
| `backend/app/api/planning.py` | inspect, reservations, conflicts. preview는 유지 |
| `backend/app/main.py` | fleet task router 등록 |
| `backend/app/core/config.py` | 계획 속도, dwell, dispatch mode, facilities 경로 |
| `backend/app/services/forward_5cm_service.py` | 삭제된 `_map_files` import 제거 |
| `backend/tests/conftest.py` | 테스트 dispatch mode를 `local`로 고정 |
| `backend/tests/test_fleet_tasks.py` | 3·4·5단계 검증 |
| `config/facilities.json` | 화장실, 식당, 대기 구역. 좌표는 null |
| `robot_receiver/pinky_task_receiver.py` | 수신 전용 저장과 ACK |
| `frontend/src/components/fleet-task-panel.tsx` | Task 생성, 대기열, 전달 |
| `frontend/src/components/fleet-console.tsx` | 경로 overlay, 시설, 기존 5cm/TF 유지 |

`config/physical_fleet.json`의 등록 좌표는 바꾸지 않았다.

## 4. Task Manager

상태:

`CREATED → PLANNING → PLANNED → RESERVED → DISPATCHING → DELIVERED → ACKNOWLEDGED`

그 외 `WAITING`, `REPLANNING`, `CANCELLED`, `FAILED`, `UNREACHABLE`.

허용되지 않은 전이는 `FleetTransitionError`로 거부한다. `ACKNOWLEDGED`는 `DELIVERED`에서만 가고, 그 전이는 physical receiver ACK의 task id와 payload hash가 맞을 때만 수행한다. 가상 수신은 `DELIVERED`와 `ack_status=SIMULATED_RECEIPT`에서 멈춘다.

SQLite `BEGIN IMMEDIATE` 안에서 예약 확정과 상태 변경을 같이 커밋한다. 선택 경로가 충돌하면 기존 confirmed 예약을 유지한 채 409를 반환한다. 취소와 전달 실패는 confirmed 예약을 `released`로 바꾼다.

재시작 복구:

- `DISPATCHING`은 완료로 보지 않고 `FAILED`로 내린다. 사유는 `dispatch interrupted by process restart`.
- 지도 revision이 저장된 값과 다르면 `REPLANNING`으로 내리고 예약을 `stale`로 만든다.
- `ACKNOWLEDGED`를 주행 완료로 바꾸지 않는다.

`map_revision`은 yaml 바이트, pgm 바이트, origin, resolution의 SHA-256 앞 16자다.

## 5. Scheduler

수동 배정은 요청한 robot id만 사용한다. 로봇이 없거나 출발지가 막혀 있거나 경로가 없으면 `FAILED` 또는 `UNREACHABLE`이다. 예약이 시간 한도 안에서 맞지 않으면 `WAITING`이다.

자동 배정은 robot id를 받으면 422다. 비용은 낮을수록 우선이다.

```
score = (travel_s + depart_delay_s + queue_length * goal_dwell_s) / priority
```

`travel_s`는 선택 경로 길이 / 0.15 m/s다. 출발지가 traversable하지 않은 로봇은 후보에서 빠진다. pinky-1은 60초 이내의 TF가 없으면 임의 좌표로 대체하지 않는다.

가상 home `(0.40, 1.90)`, `(0.40, 0.50)`은 clearance 실패다. 자동 배정에서 제외하고, 화면에는 이동 불가로 표시한다. 대체 좌표는 `approve: true`일 때만 registry에 저장한다. 테스트의 명시적 start `(0.9, 1.2)`는 task의 `start_source=request`이며 등록 home이 아니다.

같은 로봇의 이전 예약도 충돌로 본다. 그래서 한 로봇의 다음 task는 앞 task의 계획 점유가 끝난 뒤의 출발 시각을 찾는다. 이 지연은 계획 시간이고, 실제 이동이 끝난 뒤 다음 task를 자동 실행하는 기능은 아니다.

## 6. 경로 충돌 방지

시간 슬롯은 `planning_resolution / speed` = 0.05 / 0.15 초. 출발 지연은 0부터 최대 8슬롯. 그 안에서 후보 경로를 순서대로 검사하고, 넘으면 `WAITING` 또는 계획 실패다.

| 종류 | 판정 |
|---|---|
| vertex | 다른 task가 같은 시각에 같은 셀 |
| waiting | 같은 셀이고 기존 점유 시간이 더 김 |
| edge | 같은 간선을 반대 방향으로 통과 |
| crossing | 한 사각형의 두 대각선이 같은 시각에 겹침 |
| footprint | 중심 거리가 `2 * robot_radius`보다 가까움 |

구간은 반열린 구간이라 끝점이 맞닿기만 하면 충돌이 아니다. 같은 task의 자기 구간은 충돌에서 제외한다. 예약 상태 이름은 `confirmed`이고 `occupancy_class`는 `planned`다. `actual_motion_verified`는 false다.

## 7. Frontend

관제 화면은 기존 Fleet Console이다.

- pinky-1 실물: SSH, Nav2, TF.
- pinky-2, pinky-3: 가상. SSH 연결과 Nav2 READY를 표시하지 않는다. 초기 위치가 clearance 실패이면 제안 위치 승인 버튼을 보여 준다.
- 지도 클릭은 `map-frame.ts`의 `pixelToWorld` 후 `POST /api/planning/points/inspect`로 검사한다. 지도 밖이나 장애물이면 목적지로 받지 않는다.
- 시설 드롭다운의 화장실, 식당, 대기 구역은 좌표가 null이라 비활성화다.
- `경로 계산 및 예약`은 생성과 계획만 한다. `Task 전달`은 `RESERVED`일 때만 켜진다.
- 선택 경로는 실선, 다른 후보는 점선이다. 좌표는 backend waypoint의 x, y를 `worldToPixel`로 옮긴다. 시작점과 목적지의 직선 연결은 쓰지 않는다.
- 색은 pinky-1 `#007d82`, pinky-2 `#c47b00`, pinky-3 `#5b4db7`.
- 5cm 버튼은 `/api/physical-5cm`로만 가고 fleet dispatcher와 연결하지 않았다.

브라우저에서 `http://127.0.0.1:8742/`를 열었다. 지도 `1f_map_f`, pinky-1 TF 약 `(1.697, 0.110)`, Nav2 연결 문구, 가상 로봇 2대, 비활성 시설 3개, 대기열의 실물 ACK와 가상 수신이 보였다. pinky-1 task를 선택하면 path-1부터 path-3와 15개 waypoint가 표시되고, 지도에 teal 경로가 그려졌다. 이미 `ACKNOWLEDGED`인 task의 전달 버튼은 꺼져 있었다. 5cm 상태는 idle이었다.

`npx tsc --noEmit`는 통과했다. 개발 서버가 `.next`를 사용 중이라 `next build`는 실행하지 않았다. 이전에 build와 dev를 같이 돌리면 `.next`가 깨진 적이 있다.

## 8. Pinky Task Receiver

전송 방식은 SSH로 고른다. 기존 `FMS_PHYSICAL_SSH_TARGETS`와 비밀번호 설정을 재사용하고, 호스트를 새 코드에 하드코딩하지 않는다. ROS 2 action보다 수신 확인이 단순하고, Nav2 프로세스와 분리된다.

원격 명령은 다음 한 줄이다.

`python3 ~/fms_task_receiver/pinky_task_receiver.py ~/fms_task_receiver/inbox`

스크립트는 stdin JSON을 검증하고 `~/fms_task_receiver/inbox/{task_id}.json`에 저장한 뒤 ACK를 stdout으로 낸다. `execute: true`는 거부한다. 같은 task id와 같은 hash는 기존 ACK를 다시 준다. 같은 id에 다른 payload면 오류다. SSH 종료 코드만으로 성공 처리하지 않고, ACK의 task id와 payload hash를 전송본과 비교한다.

실물 확인:

- task `5b3bc030-0b45-4688-bd97-7618c1102fde`
- 출발은 당시 TF `(map → base_footprint)`, 목적지는 `(1.2, 0.8)`
- 상태 `ACKNOWLEDGED`, channel `physical`, receiver `pinky-fms-receiver`
- ACK hash와 저장 hash가 일치, `execute`는 false
- SFTP로 다시 읽은 inbox 파일과 업로드된 스크립트 SHA-256이 로컬 receiver와 같음
- 전달 뒤 5cm 상태는 계속 idle

가상 task는 `fms-virtual-receiver`, `SIMULATED_RECEIPT`, channel `simulated`다. 실물 ACK로 표시하지 않았다. 가상 로봇 좌표는 갱신하지 않았다.

## 9. API 명세

기존 `POST /api/tasks`와 `POST /api/planning/routes/preview`는 유지한다.

### `POST /api/fleet-tasks`

```json
{
  "assignment_mode": "manual",
  "robot_id": "pinky-1",
  "start": {"x": 0.9, "y": 1.2},
  "goal": {"x": 1.2, "y": 0.8},
  "priority": 5,
  "task_type": "navigate",
  "label": "",
  "idempotency_key": null
}
```

자동 배정은 `robot_id`를 비운다. 응답에는 `task_id`, `status`, `assigned_robot_id`, `planned_path`, `candidate_paths`, `delivery_status`, `ack_status`, `ack_channel`, `ack`, `map_revision`, `schedule`가 있다. 생성 직후 status는 `CREATED`이고 경로는 비어 있다.

### 그 외

| Method | Path | 동작 |
|---|---|---|
| GET | `/api/fleet-tasks` | 목록 |
| GET | `/api/fleet-tasks/queue` | 종료되지 않은 task, 우선순위 내림차순 |
| GET | `/api/fleet-tasks/facilities` | 시설. 좌표 null이면 `selectable: false` |
| GET | `/api/fleet-tasks/availability` | 배정 가능 여부, 가상 home 검사 |
| GET | `/api/fleet-tasks/{task_id}` | 상세 |
| POST | `/api/fleet-tasks/{task_id}/plan` | 배정, 경로, 예약 |
| POST | `/api/fleet-tasks/{task_id}/select` | body `{ "path_id": "path-2" }`. 다시 예약 검사 |
| POST | `/api/fleet-tasks/{task_id}/dispatch` | 수신 전달. 주행 명령 없음 |
| POST | `/api/fleet-tasks/{task_id}/cancel` | 취소 및 예약 해제 |
| POST | `/api/fleet-tasks/robots/{robot_id}/home` | `{x, y, yaw, approve}`. approve가 true이고 traversable일 때만 가상 home 저장 |
| POST | `/api/planning/points/inspect` | `{x, y}` 검사. task를 만들지 않음 |
| GET | `/api/planning/reservations` | confirmed 계획 예약 |
| GET | `/api/planning/conflicts` | 현재 confirmed 예약 사이의 충돌. 안전 검증 플래그는 false |

ACK 예:

```json
{
  "task_id": "task-uuid",
  "robot_id": "pinky-1",
  "receiver": "pinky-fms-receiver",
  "accepted": true,
  "received_at": "2026-10-28T09:49:02+00:00",
  "payload_hash": "sha256",
  "execute": false,
  "channel": "physical"
}
```

가상 수신의 receiver는 `fms-virtual-receiver`, channel은 `simulated`다.

## 10. 테스트 결과

Backend: `cd backend && ../.venv/bin/pytest -q`. 수집 93건. **92 passed, 1 failed**. 실패는 아래 11.8의 기존 심볼릭 링크다. `tests/test_fleet_tasks.py` 20건과 `tests/test_route_planning.py` 13건은 그 안에 포함되어 통과했다.

| 항목 | 결과 | 근거 |
|---|---|---|
| TEST 1 기존 회귀 | PASS | 전체 pytest 92 passed. 라이브 preview 200, 경로 3개. 5cm GET 200 `idle`. 대시보드에 SSH, Nav2, TF, 지도 표시. 5cm POST는 실행하지 않음 |
| TEST 2 수동 배정 | PASS | `test_manual_assignment_reserves_requested_robot`. 라이브 pinky-1 TF 출발로 `RESERVED`, waypoint 15 |
| TEST 3 자동 배정 | PASS | `test_automatic_assignment_reports_cost`. 사유에 score, travel, delay, queue가 포함됨 |
| TEST 4 가상 home | PASS | 등록 좌표 변경 없음. availability에서 pinky-2·3 `assignable: false`. 명시 start로는 경로 성공. 승인 API는 임시 registry 복사본에만 기록 |
| TEST 5 vertex | PASS | 같은 셀, 더 긴 기존 점유에서 `vertex`와 `waiting` |
| TEST 6 edge | PASS | `e-0,0>1,0`와 `e-1,0>0,0` |
| TEST 7 crossing | PASS | `e-0,0>1,1`와 `e-1,0>0,1` |
| TEST 8 footprint / waiting | PASS | 0.05 m 떨어진 셀, radius 0.06 m에서 footprint. waiting은 TEST 5와 같은 케이스 |
| TEST 9 재계획 | PASS | 두 번째 로봇은 `RESERVED`이면서 출발 지연 또는 다른 path, 혹은 `WAITING`. confirmed 예약의 conflict report는 빈 목록. 선택 API는 schedule이 실패하면 409이고 기존 예약 id를 유지 |
| TEST 10 동시 생성 | PASS | 두 thread가 pinky-1, pinky-2 plan. conflict report 빈 목록 |
| TEST 11 취소 | PASS | 취소 후 confirmed 예약 0 |
| TEST 12 Frontend 경로 | PASS | 브라우저에서 3대, 후보 3개, 선택 경로 15 waypoint, 지도 polyline. 좌표는 backend waypoint를 `worldToPixel`로 변환. 클릭 거부는 inspect API와 단위 테스트. 브라우저에서 지도 픽셀 클릭까지 재현하지는 않음 |
| TEST 13 시설 | PASS | 3개 시설 `selectable: false`. 화면에서 disabled. 임의 좌표를 넣지 않음 |
| TEST 14 가상 전달 | PASS | 단위 테스트와 라이브 모두 `DELIVERED` / `SIMULATED_RECEIPT` / `fms-virtual-receiver`. `ACKNOWLEDGED` 아님 |
| TEST 15 실물 전달 | PASS | 위 8절. 원격 inbox 재조회, hash 일치, execute false, 5cm idle |
| TEST 16 실패와 재시도 | PASS | 비밀번호 없는 SSH 설정은 `FAILED`, ACK 없음, 예약 해제. 같은 payload 재수신은 같은 ACK. 다른 payload는 ValueError. `CREATED`에서 dispatch는 409. 라이브 로봇의 연결 끊김과 timeout은 일부러 만들지 않아 그 두 경우는 NOT RUN |
| TEST 17 복구 | PASS | DB에서 `DISPATCHING`으로 바꾼 뒤 재조회하면 `FAILED`. 완료로 올리지 않음 |
| TEST 18 지도 변경 | PASS | 저장된 revision을 `stale-revision`으로 바꾸면 `REPLANNING`, confirmed 예약 없음 |
| TEST 19 통합 | PASS | 라이브에서 등록 3대, 수동 pinky-1 계획·예약·실물 ACK, 수동 pinky-2 가상 수신, 화면에서 두 결과를 구분해 표시. 자동 배정은 pytest. 실물 이동 없음 |

Frontend `tsc --noEmit`: PASS. `next build`: NOT RUN. 이유: 8742 개발 서버가 `.next`를 사용 중이며, build가 그 디렉터리를 깨뜨린 전례가 있다. 개발 서버 GET `/`는 200이고 브라우저에서 새 UI를 조작했다.

## 11. 기존 기능과의 충돌 검사

- `/api/tasks` mock scheduler와 `/api/planning/routes/preview`는 경로와 테이블을 분리했다. preview 뒤 fleet task 수는 0이었다.
- 지도 변환은 기존 `map_frame.py`와 `frontend/src/lib/map-frame.ts`다. overlay도 그 함수를 쓴다.
- pinky-1 pose는 TF만 사용한다. 가상 로봇에 SSH 연결 성공이나 Nav2 READY를 붙이지 않았다.
- A*와 Yen은 복제하지 않고 `k_shortest_paths`를 호출한다.
- 5cm 서비스의 주행 함수는 dispatcher에서 호출하지 않는다. 원격 명령 문자열에 `ros2` 또는 `cmd_vel`이 들어가면 전송 전에 거부한다.
- 5cm 500의 코드 경로는 수정했다. 수정 후 서버를 재시작했고 GET 상태는 `idle`이다. POST start는 로봇이 움직이므로 다시 호출하지 않았다. 따라서 수정된 POST 분기의 라이브 HTTP 응답은 NOT RUN이다.
- `test_static_isolation.py::test_sources_do_not_import_rmf_or_ros`는 이번 변경 전부터 실패하는 끊어진 심볼릭 링크다. 이번 pytest에서도 그 1건만 실패했다.

## 12. 미해결 문제

- 가상 로봇 등록 좌표는 여전히 clearance 실패다. 승인 없이는 바꾸지 않았다.
- 시설 좌표는 등록되어 있지 않다.
- 계획 예약은 실제 점유가 아니다. 주행이 끝나도 예약을 자동으로 완료 처리하지 않는다. 슬롯 한도를 넘으면 `WAITING`이다.
- 같은 로봇의 다음 task를 실제 도착 뒤에 자동 실행하지 않는다.
- 5cm POST의 라이브 재검증은 하지 않았다.
- SSH timeout과 연결 중단은 실물 로봇에서 일부러 재현하지 않았다. 단위 테스트의 실패 폐쇄만 확인했다.
- 정적 격리 테스트 1건은 기존 심볼릭 링크 때문에 실패한다.
- `next build`는 개발 서버를 유지하기 위해 실행하지 않았다.

## 13. 다음 단계

실제 Nav2 Goal은 이번 receiver에 넣지 않는다. 이후 순서는 다음과 같다.

1. 저장된 inbox payload를 읽는 별도 executor를 둔다. receiver의 ACK와 실행을 한 프로세스에 합치지 않는다.
2. 실행 전 조건은 `ACKNOWLEDGED`, 같은 `map_revision`, 현재 TF와 계획 출발점의 거리, Nav2 READY다. 조건이 아니면 goal을 보내지 않는다.
3. Goal 결과는 별도 상태(`RUNNING`, `COMPLETED`, `FAILED`)로 남긴다. 수신 ACK를 완료로 바꾸지 않는다.
4. 완료가 관측된 뒤에만 해당 task의 planned reservation을 해제하고 다음 queue task를 다시 계획한다.
5. 가상 로봇은 executor에 넣지 않는다.
6. 시설 좌표는 실측 뒤에만 `config/facilities.json`에 넣는다.
