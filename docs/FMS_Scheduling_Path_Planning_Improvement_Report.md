# FMS Scheduling / Path Planning 개선 보고서

작성 시각: 2026-10-29. 대상은 `FleetTaskService`와 `reservation_manager`이다. `backend/app/scheduler/scheduler.py`의 Mock Scheduler는 변경하지 않았다. 로봇을 이동시키지 않았고, 5cm POST, Collision Monitor 변경, DB 삭제, git reset, force push는 하지 않았다.

계획 충돌 검사는 통과했다. 실물 로봇이 그 경로를 주행하지 않았으므로 물리적 충돌 회피가 검증됐다고 보지 않는다.

## 1. 수정 전 시스템 아키텍처

```text
Fleet Task API
    |
    +-- FleetTaskService
    |       |-- A* / Yen (route_planner, 유지)
    |       |-- 단일 snapshot_provider  (모든 실물 로봇이 같은 TF를 볼 수 있음)
    |       |-- schedule_candidates
    |       |       depart = slot_index * slot     # 상대 초
    |       |       첫 번째로 충돌이 없는 Yen 경로를 반환
    |       +-- fleet_reservations.t_start/t_end   # 상대 초로 저장
    |
    +-- 화면 ETA만 now + 상대 초
    |
    +-- Mock scheduler (runtime.tick)  # fleet 예약과 분리, 이번에도 그대로
```

같은 시각에 만든 예약만 숫자로 겹쳤다. 나중에 만든 Task의 `depart_s`는 0부터 다시 시작해, DB에 이미 있는 예약과 실제 시계가 달랐다. 우선순위는 모든 로봇 후보에 같은 제수로 들어가 배정 순서를 바꾸지 않았다. 같은 로봇의 다음 Task는 이전 목적지에서 출발하지 않았다.

## 2. 수정 후 시스템 아키텍처

```text
Fleet Task API
    |
    +-- FleetTaskService
    |       |-- robot_state(robot_id)     # 해당 로봇의 domain worker / snapshot만
    |       |-- 로봇별 queue anchor        # 이전 Task 목적지, motion_verified 전 전달 금지
    |       |-- schedule_candidates
    |       |       origin = max(지금 UTC, 이전 Task 도착)
    |       |       예약 시각 = origin + 상대 지연
    |       |       충돌 없는 후보 중 가장 이른 도착
    |       +-- plan_pending()             # Mock tick과 별도. 전달하지 않음
    |
    +-- fms-fleet-replan thread            # FMS_FLEET_REPLAN_ENABLED, 기본 켜짐
    |
    +-- Mock scheduler                     # 변경 없음
```

새 예약의 `t_start`/`t_end`는 UTC epoch 초다. `depart_s`는 그 원점으로부터의 지연이다. `ACKNOWLEDGED`는 이동 완료가 아니다.

## 3. 상대시간 예약 문제 및 해결

문제: `schedule_candidates()`가 `depart = slot_index * slot`만 저장하고, 화면 시각만 `_now() + timedelta(seconds=...)`로 만들었다.

해결:

- 계획 원점은 UTC epoch다. 후속 Task는 `max(현재 시각, 이전 Task의 예상 도착)`이다.
- 구간 시각은 `origin + 상대 시간`으로 저장한다.
- `estimated_start_time` / `estimated_arrival_time`은 그 epoch의 UTC ISO 시각이다.
- 비교는 모두 epoch 초다. 로컬 타임존과 섞지 않는다.
- 예약 시각이 지났다는 이유만으로 이동 완료로 보지 않는다. 절대시각 예약의 `t_end`가 현재보다 과거이고 아직 `motion_verified`가 아니면 `delayed_motion`으로 공간을 계속 점유한다.

## 4. 기존 Reservation 데이터 마이그레이션

라이브 DB 조사(변경 전):

| 항목 | 값 |
|---|---|
| fleet_tasks | 22 |
| fleet_reservations | 381 |
| confirmed | 65, `t_start` 0.0–13.57 |
| released | 316, 같은 상대시간 |

`t_start < 1_000_000_000`이면 UTC로 복원할 원점이 없다. 숫자를 바꾸거나 행을 지우지 않았다.

- 스키마 버전 6. `fleet_reservations.time_basis`, `fleet_tasks.predecessor_task_id`, `fleet_tasks.motion_verified`를 추가한다.
- 마이그레이션은 `t_start < 1e9`인 행만 `time_basis='legacy'`로 표시한다.
- legacy 구간은 시각이 달라도 같은 기하와 충돌한다 (`legacy_time_basis`). 빈 공간으로 통과시키지 않는다.
- 그 예약을 가진 비종료 Task는 `needs_replan`이 되고, `RESERVED`/`PLANNED`이면 `REPLANNING`으로 옮긴다. ACK 이력은 상태를 되돌리지 않는다.

API 재시작 후 실제 결과:

| 항목 | 값 |
|---|---|
| fleet_tasks | 22 (유지) |
| fleet_reservations | 446 |
| legacy / released | 381 (기존 행, 시각 그대로) |
| absolute / confirmed | 65 (재계획으로 새로 쓴 행) |

활성 Task `e9afb44b-9c95-4fc3-bfc5-973b7f3797e9`는 삭제되지 않았다. 재시작 직후 재계획이 상대시간 예약을 `released`로 바꾸고, Pinky-1의 당시 TF를 시작으로 절대시간 예약 65건을 다시 썼다. 상태는 `RESERVED`이고 로봇에 전달하지 않았다. 시작 출처는 `tf:map->base_footprint` (약 1.69, 0.12)이다.

## 5. 로봇별 TF 분리 구조

`robot_state(robot_id)`는 그 로봇의 registry와 `data/domain_workers/{robot_id}.json`만 읽는다.

| Robot | Domain | 계획 시작 |
|---|---|---|
| pinky-1 | 24 | 그 로봇의 fresh TF. 없으면 임의 좌표를 만들지 않는다. 요청 시작과 0.05 m 넘게 다르면 `start_mismatch`. |
| pinky-2 | 25 | registry home `(0.425, 1.925)`. TF를 만들지 않는다. |
| pinky-3 | 26 | registry home `(0.425, 0.575)`. TF를 만들지 않는다. |

Nav2 snapshot은 `snapshot["robot_id"]`가 요청한 로봇과 같을 때만 쓴다. 가상 로봇의 실물 전환은 기존 registry 규칙 그대로다. `kind=virtual`인 로봇은 `physical_connection_enabled`를 true로 둘 수 없다.

TF가 아직 없는 실물 로봇에 운영자가 시작 좌표를 넣으면 출처는 `request`이고 `schedule.pose_unverified=true`다. 다른 로봇 TF의 대체가 아니다.

## 6. Task Queue 및 Priority 정책

- `predecessor_task_id`로 같은 로봇의 직전 미완료 Task를 연결한다.
- 후속 계획의 시작은 이전 목적지(`predecessor_goal`)이고, 출발 원점은 이전 예상 도착 이후다.
- `ACKNOWLEDGED`와 시뮬레이션 수신(`DELIVERED`)은 `motion_verified`가 아니다. 후속 `dispatch`는 409다.
- `POST /api/fleet-tasks/{id}/simulated-motion`은 `ack_channel=simulated`이고 `DELIVERED`일 때만 `motion_verified`를 세운다. 실물 ACK에는 409다.
- 이전 Task가 취소되면 후속 `RESERVED`/`PLANNED`/`WAITING`은 `REPLANNING`이 되고 그 예약은 해제된다.
- 자동 배정 점수는 `travel + delay + queue`다. 우선순위로 나누지 않는다.
- `plan_pending()` 순서는 높은 우선순위, 같으면 먼저 만든 Task다. 1시간마다 최대 +3까지 오래 기다린 Task의 우선순위를 올린다.
- 이미 `RESERVED`이거나 전달된 Task는 빼앗지 않고, 자동으로 취소하지 않는다.

## 7. WAITING 자동 재계획

`FleetTaskService.plan_pending()`만 다시 계획한다. Mock `runtime.tick`에 넣지 않았다.

대상은 `CREATED`, `WAITING`, `REPLANNING`이다. 한 번에 최대 3건. 실패하면 `replan_attempts`를 올리고 2초×시도(최대 60초) 동안 같은 Task를 다시 잡지 않는다. 기본 최대 5회(`FMS_FLEET_REPLAN_MAX_ATTEMPTS`). 성공하면 횟수를 0으로 둔다.

호출 시점: `POST /api/fleet-tasks/replan`, Task 취소 직후, 그리고 `FMS_FLEET_REPLAN_ENABLED`(기본 true)일 때 API 프로세스의 `fms-fleet-replan` 스레드. 테스트 설정은 이 스레드를 끈다.

재계획은 예약과 경로만 갱신한다. 실물 Task 전달, Nav2 Goal, 기존 ACK 재전송은 하지 않는다.

## 8. 연속 궤적 충돌 검사

기존 vertex, waiting, opposite edge, diagonal crossing, vertex footprint는 유지한다.

추가된 계획 검사:

| 종류 | 조건 |
|---|---|
| edge_vertex | 간선과 정지 정점의 최단 거리가 `2 * corner` 미만 |
| edge_proximity | 서로 다른 간선의 중심선 거리가 `2 * corner` 미만이고 같은 방향이 아님 |
| following | 같은 간선이거나, 같은 방향인데 중심선 거리가 `2 * corner` 미만 |
| delayed_motion | 절대시각 예약이 현재보다 과거에 끝났는데 이동 확인이 없음 |
| legacy_time_basis | 상대시간 예약. 시각과 무관하게 기하가 겹치면 충돌 |

`corner = planning_robot_radius_m * sqrt(2)`. 설정값 0.06 m이면 약 0.0849 m이고, Nav2 footprint 정사각형 `(±0.06, ±0.06)`의 모서리와 같다. Nav2 `footprint_padding` 0.01 m는 계획 반경에 더하지 않았다. 기존 vertex footprint 임계값 `2 * 0.06 m`는 기존 테스트와 맞추기 위해 유지한다. 새 간선 검사는 모서리 반경을 쓴다.

간선 사이 거리는 두 선분의 해석적 최단 거리다. 시간 축은 예약 구간이 겹칠 때 중심선이 그 구간 동안 점유된다고 보는 보수적 모델이다. 차체를 시간마다 샘플링한 스윕이 아니므로, 계획상 충돌 회피이지 실측 안전 증명이 아니다. `collision_safety_verified`는 항상 false다.

## 9. 후보 경로 선택 비용 함수

Yen이 준 각 경로에 대해 충돌이 없는 가장 이른 출발 슬롯만 본다. 더 늦은 슬롯은 이동 시간이 고정이면 도착만 늦춘다. 그 후보들 중 `arrival_epoch`가 최소인 경로를 고른다. 탐색 상한은 `max_depart_slots * 후보 수`다.

예: 이동 4초 + 지연 3초(도착 7초)보다 이동 5초 + 지연 0초(도착 5초)가 선택된다. 충돌이 없는 경로가 없으면 직선을 만들지 않고 `WAITING` 또는 실패다. A*와 Yen 구현은 수정하지 않았다.

로봇 점수는 `travel_s + depart_s + queue_length * dwell`이다. 선택 이유는 `assignment_reason`과 `schedule.alternatives`에 남긴다. 배터리 값은 만들지 않는다.

## 10. 수정한 파일

- `backend/app/services/reservation_manager.py`
- `backend/app/services/fleet_task_service.py`
- `backend/app/services/robot_state.py` (신규)
- `backend/app/models/tables.py`
- `backend/app/core/database.py`
- `backend/app/core/config.py`
- `backend/app/api/fleet_tasks.py`
- `backend/app/main.py`
- `backend/tests/conftest.py`
- `backend/tests/test_schedule_phase8.py` (신규)
- `frontend/src/components/fleet-task-panel.tsx`
- `frontend/src/components/fleet-console.tsx`

공식 지도 `config/maps/physical_site/1f_map_f.yaml`과 domain 24/25/26 설정은 바꾸지 않았다.

## 11. 실제 API 변경

- `POST /api/fleet-tasks/replan` — 대기 Task를 다시 계획한다. 전달하지 않는다.
- `POST /api/fleet-tasks/{task_id}/simulated-motion` — 시뮬레이션 수신에만 이동 확인을 기록한다.
- Task JSON에 `predecessor_task_id`, `motion_verified`, `schedule.time_basis`, `origin_epoch`, `arrival_epoch`, `options`, `needs_replan`, `replan_attempts`, `pose_unverified`가 추가된다.
- Reservation JSON에 `time_basis`가 추가된다.
- `estimated_start_time` / `estimated_arrival_time`은 UTC (`+00:00`)다.
- 기존 생성, 계획, 선택, 전달, 취소, ACK, archive API 경로는 유지한다.

## 12. 변경 전후 테스트 결과

변경 전 전체 pytest는 이전 지도 복구 시점 기준 132건 PASS였다. 이번 변경 후 전체 pytest는 143건 PASS다. 명령:

```text
cd backend && env -u ROS_DISTRO -u ROS_DOMAIN_ID -u RMW_IMPLEMENTATION -u FMS_PHYSICAL_MAP_YAML \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv/bin/pytest -q -p pytest
```

Frontend `tsc --noEmit` PASS. `FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack` PASS. 빌드가 넣은 `.next-verify` include는 `tsconfig.json`에서 제거했다.

| 묶음 | 결과 | 근거 |
|---|---|---|
| A. 공식 지도, A*, Yen, 장애물, unknown, corner cutting | PASS | 기존 `test_route_planning.py` 13건이 전체 스위트에 포함되어 통과. 지도 파일은 변경하지 않음. |
| B. 절대시간, 생성 시각, vertex/edge/crossing/waiting/footprint | PASS | 기존 충돌 단위 테스트 + `test_schedule_phase8.py` |
| B. edge–vertex, following, edge proximity, delayed motion, legacy | PASS | `test_schedule_phase8.py` |
| B. 취소 후 해제, 동시 계획 | PASS | 기존 fleet 테스트 |
| B. 서버 재시작 후 시각 유지 | PASS | 새 서비스로 같은 DB를 열었을 때 epoch 예약이 남음. 라이브 재시작에서 381건은 legacy로 남고 삭제되지 않음. |
| C. 수동/자동 배정, 우선순위, 선점 없음 | PASS | `test_priority_plans_before_an_older_lower_task_without_preemption` |
| C. 같은 로봇 다중 Task, 후속 시작, 미완료 전달 차단 | PASS | `test_same_robot_queue_waits_for_verified_motion` |
| C. WAITING 재계획 | PASS | legacy 점유를 취소한 뒤 대기 Task가 `RESERVED`가 됨 |
| D. Domain 24/25/26, TF 분리 | PASS | 기존 domain worker 테스트 + `test_pinky_tf_is_not_shared_across_domains` |
| D. 연결되지 않은 로봇의 TF 생성 | PASS | 가상 로봇 state는 `registry_home`이고 pose를 TF로 만들지 않음. 화면에서도 Pinky-2·3는 등록 위치만 표시. |
| E. ACK, cancel ACK, archive, map/task API | PASS | 기존 cancel/browse 테스트 |
| E. 5cm 실주행 | NOT RUN | POST 하지 않음. 버튼 코드와 Collision Monitor는 변경하지 않음. |
| E. Frontend 지도·Task 상세 | PASS | 브라우저에서 `1f_map_f.yaml` 186×236, 경로 polyline 3개, Task 상세의 UTC 시각·TF 출처·우선순위를 확인. |
| 물리적 충돌 회피 | NOT RUN | 로봇을 움직이지 않음. |

## 13. 기존 FMS 기능 회귀 검사

유지됨: 공식 지도, domain 24/25/26, domain worker, Pinky-1 SSH 카드, Pinky-2·3 가상 상태, A*/Yen, Task 생성·배정, 예약, receiver ACK, cancel ACK, archive/restore, 지도 표시. Mock scheduler 테스트 3건도 통과했다.

라이브 화면(2026-10-29 12:00 KST): Pinky-1 domain 24, Pinky-2 domain 25, Pinky-3 domain 26. 가상 로봇은 “등록 위치는 실제 TF가 아닙니다”. 활성 Task 1건은 `RESERVED`, ACK 없음. Task 전달 버튼은 누르지 않았다.

확인 도중 Pinky-1 카드가 TF fresh / Nav2 READY로 바뀌며 5cm 버튼의 disabled가 풀린 순간이 있었다. 이 작업은 그 게이트와 StopZone을 수정하지 않았고, 5cm POST도 하지 않았다. 버튼이 보인다고 주행이 실행된 것은 아니다.

## 14. 미해결 문제

- 계획 충돌과 실제 차체 충돌은 다르다. 중심선 거리와 예약 구간의 검사이지, 실측 스캔이나 StopZone 검증이 아니다.
- vertex footprint는 여전히 반폭 0.06 m의 두 배다. 모서리 0.085 m는 새 간선 검사에만 쓴다.
- 저장된 간선의 끝점은 DB에 없고, 검사 때 그래프에서 다시 만든다. 그래프를 열 수 없는 행은 중점만 남는다.
- 자동 선점은 범위에서 제외했다. 높은 우선순위가 와도 확정 예약을 취소하지 않는다.
- 배터리는 신뢰할 값이 없어 점수에 넣지 않았다.
- 실물 Task의 이동 완료 이벤트가 없다. 후속 실물 Task는 전달되지 않은 채로 남는다.
- 재시작 때 상대시간 활성 Task가 한 번 재계획되었다. 이전 계획 좌표와 예약 시각은 `released` + `legacy`로 남아 있다.

## 15. 실제 Nav2 Task 실행에 더 필요한 기능

1. 로봇이 보고하는 주행 완료·실패·정지. 수신 ACK와 구분되는 `motion_verified`의 실물 근거.
2. 그 완료 이후에만 후속 Task를 전달하는 실행기. 지금은 전달 자체가 Nav2 Goal을 보내지 않는다.
3. 주행 중 위치가 예약을 벗어나면 남은 예약을 무효로 보고 다시 계획하는 피드백. `delayed_motion`은 계획 시각이 지난 점유를 유지할 뿐, 실제 편차를 측정하지 않는다.
4. Nav2 footprint와 padding을 계획 격자 반경과 맞춰 재검증하는 절차. 이번 변경은 이미 잡힌 경로를 새 반경으로 자동 승인하지 않는다.
5. 5cm와 StopZone은 이번 범위 밖이다. Collision Monitor를 바꾸거나 주행 POST를 하기 전에는 실물 이동을 시작하지 않는다.
