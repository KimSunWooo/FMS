# FMS 통신 및 API 명세

기준 코드는 FastAPI 라우터, `fleet_dispatcher.py`, `robot_receiver/pinky_task_receiver.py`, `nav2_forward_5cm_worker.py`다. 여기 없는 경로와 필드는 구현되어 있지 않다.

인증 값이 없는 엔드포인트는 별도 로그인 헤더를 요구하지 않는다. 5cm 시작과 취소만 로컬 주소와 `X-FMS-Operator-Token`을 요구한다.

## 통신 세 갈래

```text
브라우저  --HTTP-->  Next.js :8742
                         |
                         | /fms-api/*  ->  /api/*
                         v
                    FastAPI :8741
                         |
                         | SSH, JSON stdin, ACK stdout
                         v
                    Pinky-1 Receiver
                    execute=false, Nav2를 호출하지 않음

FastAPI 5cm 워커 --ROS 2 Action--> /drive_on_heading
```

Task Dispatcher는 로봇에 HTTP POST를 보내지 않는다. Pinky-1 전달은 Paramiko SSH다. 5cm 주행은 Task payload와 별도이며, 운영자가 시험을 시작한 뒤에만 Nav2 Goal을 보낸다.

`scripts/run_api.sh`는 `0.0.0.0:8741`에 바인딩한다. Fleet Task API는 토큰이 없으므로 외부 인터페이스에 이 포트를 열면 조회와 상태 변경이 모두 가능하다.

## HTTP API

공통 실패는 FastAPI `{"detail": "..."}`다. Fleet Task 비즈니스 오류는 서비스의 status code를 그대로 쓴다. 경합과 잘못된 전이는 409다.

### 시스템

| Method | URL | 기능 | 로봇 동작 | DB |
|---|---|---|---|---|
| GET | `/api/system/health` | 프로세스와 SQLite `SELECT 1` | 없음 | 읽기 |
| GET | `/api/system/readiness` | health에 `ready`를 추가. DB가 아니면 503 | 없음 | 읽기 |

health 본문에는 `service`, `status`, `phase`, `database`, `physical_control`, `ros_bridge`, `execution_mode`가 있다. 현재 코드의 `phase` 값은 2다.

### 물리 로봇과 지도

| Method | URL | 기능 | 로봇 동작 | DB |
|---|---|---|---|---|
| GET | `/api/physical/robots` | Registry, Domain, SSH, Nav2 | 없음 | 없음 |
| GET | `/api/physical/domain-workers` | 로봇별 Worker 상태 | 없음 | 없음 |
| GET | `/api/physical/nav2/status` | Nav2 모니터 스냅샷 | 없음 | 없음 |
| GET | `/api/physical/nav2/pose` | `map`→`base_footprint` | 없음 | 없음 |
| GET | `/api/physical/map` | 지도 메타데이터 | 없음 | 없음 |
| GET | `/api/physical/map/image` | 지도 이미지 | 없음 | 없음 |
| POST | `/api/physical/map/coordinates` | 지도 좌표 변환 | 없음 | 없음 |

좌표 변환은 `map` 또는 `grid` 중 하나만 보낸다.

```json
{"map": {"x": 1.0, "y": 1.0, "yaw": 0.0}}
```

```json
{"grid": {"col": 10, "row": 20}}
```

### 경로

`POST /api/planning/routes/preview`는 계산만 한다. Task, 예약, Nav2 Goal을 만들지 않는다.

```json
{
  "robot_id": "pinky-2",
  "goal": {"x": 1.0, "y": 1.0},
  "start": null,
  "max_candidates": 3,
  "allow_diagonal": true
}
```

`start`를 생략하면 Registry 홈 또는 Pinky-1의 신선한 TF를 쓴다. 도달 불가면 HTTP 422이고 `reachable: false`, `paths: []`다.

| Method | URL | 기능 | 로봇 동작 | DB |
|---|---|---|---|---|
| GET | `/api/planning/starts` | 시작점 후보 | 없음 | 읽기 |
| POST | `/api/planning/points/inspect` | 점유 여부 | 없음 | 없음 |
| GET | `/api/planning/reservations` | 예약 목록 | 없음 | 읽기 |
| GET | `/api/planning/conflicts` | 충돌 | 없음 | 읽기 |
| POST | `/api/planning/routes/preview` | A*와 Yen 미리보기 | 없음 | 없음 |

점 검사 body는 `{"x": 1.0, "y": 1.0}`이다.

### Fleet Task

`/api/tasks`는 모의 스케줄러다. 실제 관제 Task는 `/api/fleet-tasks`다.

| Method | URL | Body | 로봇 동작 | DB |
|---|---|---|---|---|
| POST | `/api/fleet-tasks` | 아래 생성 예 | 없음 | 생성 |
| GET | `/api/fleet-tasks` | 없음 | 없음 | 읽기. 보관 포함 |
| GET | `/api/fleet-tasks/browse` | query | 없음 | 읽기 |
| GET | `/api/fleet-tasks/queue` | 없음 | 없음 | 읽기 |
| GET | `/api/fleet-tasks/facilities` | 없음 | 없음 | 읽기 |
| GET | `/api/fleet-tasks/availability` | 없음 | 없음 | 읽기 |
| GET | `/api/fleet-tasks/{task_id}` | 없음 | 없음 | 읽기 |
| POST | `/api/fleet-tasks/{task_id}/plan` | 없음 | 없음 | 경로 저장 |
| POST | `/api/fleet-tasks/{task_id}/select` | `{"path_id":"..."}` | 없음 | 예약 |
| POST | `/api/fleet-tasks/{task_id}/dispatch` | 없음 | SSH 또는 가상 수신 | ACK 저장 |
| POST | `/api/fleet-tasks/{task_id}/cancel` | `{"reason":"operator"}` 또는 빈 body | 수신된 Task면 cancel JSON | 상태 변경 |
| POST | `/api/fleet-tasks/{task_id}/archive` | `{"archived_by":"operator"}` 또는 빈 body | 없음 | 보관 플래그 |
| POST | `/api/fleet-tasks/{task_id}/restore` | archive와 동일 | 없음 | 보관 해제 |
| POST | `/api/fleet-tasks/robots/{robot_id}/home` | `{"x":0.4,"y":1.9,"yaw":0,"approve":true}` | 없음 | `config/physical_fleet.json`의 가상 로봇 홈 |

생성:

```json
{
  "assignment_mode": "manual",
  "robot_id": "pinky-2",
  "goal": {"x": 1.0, "y": 1.0},
  "start": null,
  "priority": 5,
  "task_type": "navigate",
  "label": "",
  "idempotency_key": null
}
```

`assignment_mode`는 `manual` 또는 automatic 할당이다. `priority`는 1 이상 10 이하다. 좌표 단위는 지도 미터다.

browse query: `scope`는 `active`, `history`, `archived`. `page_size`는 10, 20, 50만 허용한다. `status`, `robot_id`, `q`는 선택이다.

dispatch는 로봇을 움직이라는 명령이 아니다. 수신 저장과 ACK만 수행한다. Pinky-1에 대한 dispatch는 운영자 승인 후에만 호출한다.

### 5cm

| Method | URL | 인증 | 로봇 동작 |
|---|---|---|---|
| GET | `/api/physical/tests/pinky-1/forward-5cm/diagnosis` | 없음 | 구독만. Goal 없음 |
| GET | `/api/physical/tests/pinky-1/forward-5cm` | 없음 | 메모리 상태 조회 |
| POST | `/api/physical/tests/pinky-1/forward-5cm` | localhost와 토큰 | 진단 통과 시에만 Goal |
| POST | `/api/physical/tests/pinky-1/forward-5cm/cancel` | localhost와 토큰 | 진행 중 Goal 취소 |

POST 성공은 202다. 기능이 꺼져 있거나 토큰이 다르면 403이다. StopZone 진단이 막히면 409이고 `goal_sent`는 false다. 모니터가 없으면 503이다.

화면은 Backend에 직접 토큰을 보내지 않는다. `POST http://127.0.0.1:8742/api/physical-5cm` body는 `{"action":"start"}` 또는 `{"action":"cancel"}`이고, 헤더 `x-fms-ui-intent: physical-5cm`가 필요하다. Origin은 그 호스트와 같아야 한다.

### 모의 API

`/api/robots`, `/api/tasks`, `/api/agent`, `/api/fleet`, `/api/traffic`는 Phase 1 모의 런타임이다. Fleet Task와 Pinky Receiver를 대체하지 않는다. 로봇 생성은 `POST /api/robots`, 모의 Task는 `POST /api/tasks`다. 응답에는 `motion_completed: false`가 들어간다.

2026-10-28에 `GET /openapi.json`이 반환한 경로 전체는 54개다. 위에 적은 Fleet, 물리, 계획 경로 외에 다음이 있다.

- `POST /api/agent/session`, `POST /api/agent/session/replace`, `POST /api/agent/stop`, `POST /api/agent/heartbeat`, `POST /api/agent/claim`, `GET /api/agent/assignment`, `POST /api/agent/goal-event`, `POST /api/agent/pose`
- `GET /api/fleet/status`, `GET /api/fleet/events`
- `GET /api/robots`, `POST /api/robots`, `GET /api/robots/{robot_id}`, `GET /api/robots/{robot_id}/status`, `POST /api/robots/{robot_id}/offline`, `POST /api/robots/{robot_id}/online`, `POST /api/robots/{robot_id}/emergency-stop`, `POST /api/robots/{robot_id}/estop/clear`, `POST /api/robots/{robot_id}/mock-observe`
- `POST /api/tasks`, `GET /api/tasks`, `GET /api/tasks/{task_id}`, `POST /api/tasks/{task_id}/cancel`, `POST /api/tasks/{task_id}/resolve`
- `GET /api/traffic/zones`, `GET /api/traffic/reservations`, `POST /api/traffic/reservations/{reservation_id}/release`

모의 emergency-stop은 Nav2 `/cmd_vel`을 0으로 발행하지 않는다. Pinky-1 수신 취소와 5cm Goal 취소와 별개다.

## Task payload

`canonical_payload`가 만드는 JSON이다. 좌표는 지도 미터이고 `start`에는 yaw가 없다. `execute`는 항상 false다. 이 값이 false인 것은 수신만 하라는 뜻이지, Nav2가 이미 실행 중이라는 뜻이 아니다.

```json
{
  "task_id": "uuid",
  "robot_id": "pinky-1",
  "task_type": "navigate",
  "map_id": "1f_map_f",
  "map_revision": "16자리 sha256",
  "start": {"x": 0.0, "y": 0.0},
  "goal": {"x": 1.0, "y": 1.0, "label": ""},
  "waypoints": [{"x": 0.0, "y": 0.0, "waypoint_id": "w1"}],
  "execute": false,
  "payload_hash": "sha256 hex"
}
```

`payload_hash`는 `payload_hash` 키를 뺀 객체를 `sort_keys`와 구분자 `,` `:`로 직렬화한 SHA-256이다. `map_revision`은 지도 YAML, PGM, origin, resolution의 해시 앞 16자다. waypoint는 선택된 계획 경로다.

수신 ACK는 stdout 마지막 줄이다.

```json
{
  "task_id": "uuid",
  "robot_id": "pinky-1",
  "receiver": "pinky-fms-receiver",
  "accepted": true,
  "received_at": "2026-10-28T00:00:00+00:00",
  "payload_hash": "같은 해시",
  "execute": false
}
```

FMS는 exit code 0, `accepted`, task id, payload hash가 같을 때만 성공으로 본다. 같은 task id에 다른 해시가 있으면 Receiver는 `accepted: false`와 exit 2를 낸다. `execute: true`는 거부한다.

취소 요청은 다른 구조다.

```json
{
  "action": "cancel",
  "task_id": "uuid",
  "robot_id": "pinky-1",
  "cancellation_request_id": "uuid",
  "requested_at": "2026-10-28T00:00:00+00:00",
  "reason": "operator",
  "execute": false
}
```

Cancel ACK:

```json
{
  "kind": "cancel_ack",
  "task_id": "uuid",
  "robot_id": "pinky-1",
  "receiver": "pinky-fms-receiver",
  "accepted": true,
  "cancelled": true,
  "cancellation_request_id": "uuid",
  "cancelled_at": "2026-10-28T00:00:00+00:00",
  "reason": "operator",
  "execute": false
}
```

저장 파일은 `executable: false`가 되고 수신 ACK의 payload hash는 유지된다. 같은 취소를 다시 보내면 저장된 cancel ACK를 돌려준다. 가상 로봇은 receiver가 `fms-virtual-receiver`이고 `channel`이 `simulated`다. SSH 성공 ACK에는 FMS가 `channel: "physical"`을 붙인다.

## SSH

1. `FMS_PHYSICAL_SSH_TARGETS`에서 `pinky-1`의 `user@host`를 찾는다.
2. 시스템 host key로 접속한다. 비밀번호는 `FMS_PHYSICAL_SSH_PASSWORD`다. 연결 timeout 기본값은 3초다.
3. `~/fms_task_receiver`가 없으면 만들고 로컬 `robot_receiver/pinky_task_receiver.py`를 업로드한다.
4. 원격 명령은 `python3 ~/fms_task_receiver/pinky_task_receiver.py ~/fms_task_receiver/inbox`다. 명령 문자열에 `ros2` 또는 `cmd_vel`이 있으면 실행하지 않는다.
5. JSON 하나를 stdin에 쓰고 stdout 마지막 줄을 ACK로 파싱한다. `exec_command` timeout은 20초다.
6. 로컬 테스트 모드 `FMS_FLEET_DISPATCH_MODE=local`은 SSH 없이 같은 Receiver를 로컬 inbox에서 실행한다. 기본 모드는 `ssh`다.

## ROS 2

로봇별 Worker가 자기 Domain에서만 읽는다. Topic 이름이 같아도 Domain이 다르면 섞지 않는다.

| Robot | Domain | Worker |
|---|---:|---|
| pinky-1 | 24 | 실물 관측. 5cm Action도 이 Domain |
| pinky-2 | 25 | 가상. 실물 미연결이면 ROS에 붙지 않고 pose는 null |
| pinky-3 | 26 | 가상. 실물 미연결이면 ROS에 붙지 않고 pose는 null |

Task의 소유자는 `assigned_robot_id`다. 응답의 `assigned_ros_domain_id`는 Registry 조회 결과이며 payload hash에는 들어가지 않는다. Domain 번호로 Task 소유자를 고르지 않는다.

FMS가 구독하거나 호출하는 인터페이스만 적는다. 아래 이름은 Pinky-1 Domain 24 기준이다.

| 이름 | 타입 | FMS 역할 | 이동 명령 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 진단 구독 | 아님 |
| `/odom` | `nav_msgs/Odometry` | 5cm 거리 측정 구독 | 아님 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `map`→`odom`→`base_footprint`, LiDAR TF | 아님 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 진단 구독 | 아님. Nav2가 발행 |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | 진단 구독 | 아님. velocity_smoother 출력 |
| `/cmd_vel` | `geometry_msgs/Twist` | 진단 구독 | Collision Monitor 출력. FMS는 발행하지 않음 |
| `/drive_on_heading` | `nav2_msgs/action/DriveOnHeading` | 5cm 워커만 Action client | 승인된 5cm 시험일 때만 |
| `/set_initial_pose` | Nav2 AMCL 서비스 | `physical_nav2_auto.py`가 초기 위치를 보낼 때 | 주행 Goal 아님 |
| lifecycle get/set | map_server, amcl, controller_server, planner_server, bt_navigator, velocity_smoother, collision_monitor | 자동 기동과 상태 확인 | 직접 속도 아님 |

`/navigate_to_pose` [`nav2_msgs/action/NavigateToPose`]는 준비 확인용으로 action list에서 찾는다. Fleet Task가 이 Action을 보내지는 않는다. FMS 코드는 `/amcl_pose`를 구독하지 않는다. 위치 필드는 TF에서 온다.

5cm Goal 필드:

- `target.x` = 0.05 m. 헤딩 방향 거리다.
- `target.y` = 0, `target.z` = 0
- `speed` = 0.04 m/s
- `time_allowance` = 10 s

성공은 Action 상태만으로 정하지 않는다. map TF 전진과 odometry 전진이 모두 0.035 m 이상 0.10 m 이하이고 횡방향 편차가 0.04 m 이하여야 한다. 측정이 없으면 성공으로 기록하지 않는다.

속도 체인:

```text
DriveOnHeading
  -> behavior / controller
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel
  -> pinky_bringup
```

`launch/pinky1_navigation.launch.py`는 behavior, controller, smoother의 `cmd_vel`을 `cmd_vel_nav`로 remap한다. Collision Monitor는 remap하지 않는다. StopZone은 `base_footprint` 기준 원, 반경 0.18 m, `min_points` 4다.

## 환경 변수

| 변수 | 목적 | 코드 기본값 | 현재 개발 | 재시작 |
|---|---|---|---|---|
| `FMS_ROS_DOMAIN_ID` | API 설정의 단일 도메인. 로봇 Domain은 Registry가 우선 | 73 | 24. Pinky-1 Worker는 Registry의 24 | 필요 |
| `FMS_DOMAIN_WORKERS_ENABLED` | 로봇별 Worker supervisor | true | true | 필요 |
| `FMS_API_HOST` | 설정값. 스크립트는 0.0.0.0을 직접 전달 | 0.0.0.0 | 0.0.0.0 | 필요 |
| `FMS_API_PORT` | 문서상 포트. 스크립트는 8741을 직접 전달 | 8741 | 8741 | 필요 |
| `FMS_DATABASE_URL` | SQLite | `data/fms.sqlite3` | `.env` 경로 | 필요 |
| `FMS_PHYSICAL_SSH_TARGETS` | robot id와 `user@host` | 빈 문자열 | 설정됨 | 필요 |
| `FMS_PHYSICAL_SSH_PASSWORD` | Paramiko 비밀번호 | 빈 문자열 | 설정됨. 값 비공개 | 필요 |
| `FMS_PHYSICAL_SSH_TIMEOUT_SECONDS` | 접속 timeout | 3 | 기본 또는 `.env` | 필요 |
| `FMS_PHYSICAL_MAP_YAML` | map_server와 플래너 지도 | `1f_map_f.yaml` | 같은 파일 | 필요 |
| `FMS_PHYSICAL_NAV2_AUTO_START` | Nav2 모니터 자동 기동 | false | 개발 환경에서 사용 | 필요 |
| `FMS_PHYSICAL_5CM_TEST_ENABLED` | 5cm POST 허용 | false | 시험 시 true | 필요 |
| `FMS_PHYSICAL_5CM_TEST_TOKEN` | 5cm POST 헤더 | 빈 문자열 | 설정됨. 값 비공개 | 필요 |
| `FMS_LEASE_SECRET` | 모의 lease | 없음 | `.env`에만 | 필요 |
| `FMS_FLEET_DISPATCH_MODE` | `ssh` 또는 `local` | ssh | ssh | 필요 |

계획 파라미터:

| 필드 | 환경 접두 | 값 | 의미 |
|---|---|---|---|
| `planning_resolution_m` | `FMS_PLANNING_RESOLUTION_M` | 0.05 m | 계획 격자 |
| `planning_robot_radius_m` | `FMS_PLANNING_ROBOT_RADIUS_M` | 0.06 m | 계획용 반경. 검증된 안전 반경이 아님 |
| `planning_max_candidates` | `FMS_PLANNING_MAX_CANDIDATES` | 3 | Yen 후보 수 |
| `planning_allow_diagonal` | `FMS_PLANNING_ALLOW_DIAGONAL` | true | 대각 이동 |
| `planning_speed_m_s` | `FMS_PLANNING_SPEED_M_S` | 0.15 m/s | 예약 시간 추정. 실측 속도가 아님 |
| `planning_max_depart_slots` | `FMS_PLANNING_MAX_DEPART_SLOTS` | 8 | 출발 슬롯 |
| `planning_goal_dwell_s` | `FMS_PLANNING_GOAL_DWELL_S` | 0.5 s | 목표 체류 |

지도 파일 해상도는 0.01 m/pixel이다. 계획 격자와 다르다.
