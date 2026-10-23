# Phase 2 보고 — Gazebo / Nav2 단일 로봇

전체 판정: **BLOCKED**. Gazebo Pinky 주행 로그가 없으므로 PASS가 아니다.

| 항목 | 상태 |
| --- | --- |
| Phase 0–1 Mock 회귀 (기존 34) | PASS |
| Phase 2 계약 UNIT / MOCK (15) | PASS |
| GAZEBO_E2E | NOT_RUN |
| 실물 /cmd_vel, 모터, 승강기, 자동문 | BLOCKED |
| 다중 로봇 충돌 방지 검증 | 선언하지 않음. Phase 1 구역 예약은 유지 |
| Phase 3 Gazebo 진입 | 불가. 단일 로봇 Gazebo 로그가 생기기 전 |

실행 시각의 호스트는 Ubuntu 24.04.5 LTS, 커널 6.12.94+, 메모리 약 15Gi 중 available 약 3.7Gi, GPU 없음, 디스크 여유 약 245G였다. ROS apt 소스는 `/etc/apt/sources.list.d`에 없고 `/opt/ros`도 없다.

## 1. Phase 0–1 코드 확인

보고 문장과 코드를 대조했다.

- FastAPI 앱은 `backend/app/main.py`. 포트 기본값 8741.
- Task 권위 저장소는 SQLite `data/fms.sqlite3`. 스키마는 `backend/app/models/tables.py`.
- 소유권은 `owner_epoch`, HMAC lease, 프로세스 메모리의 gateway boot id. `backend/app/ownership/service.py`, `backend/app/robot/mock_gateway.py`.
- 로봇당 활성 Task 1개는 partial unique index `uq_tasks_one_active_per_robot`.
- Mock 3대는 `config/fleet.json`과 `MockFleet`. 완료는 `complete_mock` → `apply_result`이며 HTTP 202 자체가 완료가 아니다.
- `physical_control_enabled`, `ros_bridge_enabled`가 참이면 프로세스가 기동을 거부한다.
- 기존 `send_navigate_to_pose`는 예외만 낸다. 이번에도 그 진입점은 그대로 닫혀 있다.
- 소스 AST와 `pyproject.toml` 의존성 블록에 rclpy, nav2, rmf import가 없다. Nav2 클라이언트는 함수 안의 `importlib.import_module`만 사용한다.
- Open-RMF 런타임, 기존 DB, 기존 프로세스에는 연결하지 않았다. 이 머신에는 그 트리가 없다.

상태 이름은 기존 모델을 유지한다. 취소 완료는 `CANCELED`이다. 요청문의 `CANCELLED`로 바꾸지 않았다.

`PENDING`, `ASSIGNED`, `DISPATCHING`, `RUNNING`, `COMPLETED`, `FAILED`, `CANCEL_REQUESTED`, `CANCELED`, `RECOVERY_REQUIRED`.

회귀: `cd backend && pytest` → **49 passed**. 그중 기존 34개는 그대로 통과했다.

## 2. 수정한 파일

- `backend/app/core/config.py` — `execution_mode` (`mock` 기본, `physical`은 기동 거부)
- `backend/app/core/database.py` — 스키마 2, `nav2_action_goal_id` 열
- `backend/app/models/tables.py` — 같은 열
- `backend/app/robot/safety.py` — mock / gazebo / physical 분리. `/cmd_vel`은 계속 거부
- `backend/app/scheduler/scheduler.py` — gazebo는 실행 모드가 gazebo일 때만 후보
- `backend/app/services/task_manager.py` — claim 1회, Nav2 결과 매핑, 취소-성공 경합
- `backend/app/services/runtime.py` — 외부 에이전트 API, mock pump는 mock 로봇만
- `backend/app/services/read_model.py` — goal에 Nav2 action id
- `backend/app/api/agent.py`, `backend/app/api/schemas.py`, `backend/app/main.py`
- `backend/tests/conftest.py`, `backend/tests/test_phase2_nav2_contract.py`
- `robot_agent/nav2_bridge.py`, `robot_agent/preflight.py`, `robot_agent/gazebo_nav2_client.py`, `robot_agent/agent_main.py`
- `scripts/run_agent.sh`
- `frontend/src/components/fleet-console.tsx` — 모드와 Gazebo preflight 배지
- `README.md`, `docs/isolation-plan.md`, `docs/roadmap.md`, `docs/verification-report.md`
- 이 문서와 `docs/phase2-gazebo-preflight.json`

## 3. 아키텍처와 데이터 흐름

```text
API (execution_mode=mock|gazebo, ROS 노드 없음)
  Task 생성 → Scheduler → Ownership/Lease → NavigationGoal 1행 (status=pending)
  mock 로봇: 프로세스 안 MockFleet가 accept 후 위치 오차로 완료
  gazebo 로봇: pump가 완료시키지 않음
        │
        ▼
로봇별 에이전트 프로세스 (scripts/run_agent.sh)
  preflight 실패 → 종료 코드 2, goal 미전송, COMPLETED 미보고
  preflight 통과 시에만 session → claim 1회 → NavigateToPose
  관측한 액션 결과만 FMS에 게시
  측정한 pose가 없으면 trustworthy=false → COMPLETED 불가
```

실행 모드:

| 모드 | 동작 |
| --- | --- |
| mock (기본) | 기존 테스트와 Mock 완료. gazebo 로봇 등록은 403 |
| gazebo | 외부 에이전트 HTTP만 허용. API는 Nav2를 열지 않음. preflight 불일치면 에이전트가 전송 거부 |
| physical | 프로세스 기동 거부. 로봇 mode=physical은 403 |

완료 조건은 그대로다. 같은 task id, execution id, owner epoch, session, 유효 lease, goal status succeeded, 위치 오차 이내, 취소·실패·복구와 충돌 없음. Goal accepted는 `RUNNING`일 뿐 `COMPLETED`가 아니다.

결과 매핑:

| 관측 | Task |
| --- | --- |
| accept | RUNNING |
| rejected, 아직 accept 전 | FAILED. goal 행은 1개 |
| aborted, fence 일치 | FAILED |
| canceled, 취소 요청이 있고 pose가 신뢰됨 | CANCELED |
| canceled, 요청 없음 | RECOVERY_REQUIRED |
| succeeded 동안 CANCEL_REQUESTED | RECOVERY_REQUIRED. COMPLETED 아님 |
| disconnected, 서버 단절, 상태 미관측 | RECOVERY_REQUIRED. 자동 재전송 없음 |
| stale epoch, 다른 goal/execution/session | 상태 유지 |
| lease 만료 후 success | RECOVERY_REQUIRED |

Goal 전송은 `navigation_goals.status`가 `pending`일 때 한 번 `send_claimed`로 바뀐다. 두 번째 claim은 `send_navigate_to_pose=false`이고 행을 추가하지 않는다.

에이전트를 멈추면 그 로봇의 in-flight 작업만 `RECOVERY_REQUIRED`가 된다. 재시작 후에도 goal이 끝났다고 추정하지 않는다. `goal_may_be_in_flight`이면 복구 격리이고 goal 행 수는 유지된다.

## 4. ROS 2 / Gazebo 환경

| 점검 | 결과 |
| --- | --- |
| Ubuntu | 24.04.5 LTS. Jazzy 대상 OS와 일치 |
| `/opt/ros`, `ros2`, `ROS_DISTRO` | 없음 |
| `gz`, `gazebo`, `ign` | 없음 |
| ROS apt source | `ubuntu.sources`, `google-chrome.sources`만 있음 |
| Nav2 인터페이스 (`/opt/ros/jazzy/share/nav2_msgs`) | 없음 |
| Pinky 시뮬 패키지 | 없음. `/usr/bin/pinky`는 Unix finger 유틸이라 로봇 패키지로 세지 않음 |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` | 프로세스 환경에 없음. 기대 도메인 73과 불일치로 fail-closed |
| NVIDIA GPU | 없음 |
| DDS 통신 시험 | 노드를 만들지 않음. NOT_RUN |
| 설치 가능성 | OS와 디스크는 Jazzy+Gazebo 설치를 막지 않음. 패키지 미설치, Pinky 월드 부재, GPU 없음, available 메모리 약 3.7Gi. 대체 시뮬레이터를 Pinky로 부르지 않기 위해 설치와 실행을 하지 않음 |

에이전트 실행:

```text
./scripts/run_agent.sh --robot-id pinky-sim
exit 2
nav2_goal_sent=false
task_completed=false
```

원문은 [phase2-gazebo-preflight.json](phase2-gazebo-preflight.json).

## 5. 단일 로봇 E2E 로그

**없음.** Gazebo를 기동하지 않았고 Nav2 goal을 전송하지 않았다. 따라서 이 절을 주행 성공으로 채우지 않는다.

## 6. Nav2 Goal ID와 FMS Task ID

주행으로 연결된 쌍은 없다.

MOCK 계약에서는 FMS가 만든 `goal_id` 한 개와 테스트가 넣은 표식 `nav2_action_goal_id=fixture-action-id`를 같은 task 행에 저장한다. 이 표식은 Nav2 액션 서버가 발급한 id가 아니다. claim 이벤트 `goal_send_claimed`에는 `completed: false`가 들어간다. 두 번째 claim은 같은 `goal_id`를 돌려주고 행을 늘리지 않는다.

실제 클라이언트가 결과를 관측하면 `handle.goal_id`만 `nav2_action_goal_id`로 올린다. 관측 전에는 그 값을 만들지 않는다. 에이전트는 목적지 좌표를 관측 pose로 복사하지 않는다.

## 7. 테스트

구분: UNIT / MOCK / GAZEBO_E2E.

| 검증 | 구분 | 상태 |
| --- | --- | --- |
| 기존 34개 회귀 | MOCK / UNIT | PASS |
| preflight 전 조건, 라이브 호스트 거절 | UNIT | PASS |
| 잘못된 ROS 도메인 거절 | UNIT | PASS |
| physical 모드와 `/cmd_vel` 차단, `send_navigate_to_pose` 거절 | UNIT | PASS |
| 에이전트 프로세스가 goal 없이 종료 코드 2 | UNIT | PASS |
| mock 모드에서 gazebo 등록 403 | MOCK | PASS |
| 성공 결과는 RUNNING 이후에만, fence와 오차가 맞을 때 COMPLETED | MOCK | PASS. 입력은 테스트 fixture |
| reject → FAILED, abort → FAILED, 요청된 cancel → CANCELED, 단절 → RECOVERY | MOCK | PASS |
| 중복 claim이 goal 행을 늘리지 않음 | MOCK | PASS |
| stale epoch 결과 유지, 취소와 success 경합은 COMPLETED가 아님 | MOCK | PASS |
| lease 만료 후 success → RECOVERY | MOCK | PASS |
| FMS 재시작이 goal 종료로 추정하지 않음 | MOCK | PASS |
| 새 에이전트 세션이 이전 goal을 재개하지 않음 | MOCK | PASS |
| 한 에이전트 종료가 다른 로봇 작업을 바꾸지 않음 | MOCK | PASS |
| claim 직후 에이전트 종료 → RECOVERY, 재전송 없음 | MOCK | PASS |
| Gazebo에서 Pinky가 목표점까지 주행 | GAZEBO_E2E | NOT_RUN |

pytest 합계: **49 passed**.

## 8. 장애와 남은 위험

- Gazebo, Nav2, Pinky 패키지가 없어 단일 로봇 E2E가 막혀 있다. 설치는 OS상 가능하지만, Pinky 월드 없이 TurtleBot 등을 Pinky로 보고하지 않았다.
- DDS를 붙이지 않았다. 도메인 73이 현장 Care-Move 도메인과 같은지는 이 머신에서 확인할 수 없다. 같으면 에이전트를 켜면 안 된다.
- MOCK의 `succeeded`는 테스트가 넣은 결과다. 시뮬레이터 성공이 아니다.
- 에이전트는 아직 AMCL/odom pose를 읽지 않는다. Nav2가 성공해도 pose가 없으면 FMS는 COMPLETED로 올리지 않는다. 주행이 가능해진 뒤에 pose 관측을 붙여야 한다.
- `config/cyclonedds.fms.xml`은 여전히 로드하지 않는다.
- 구역 예약은 Phase 1 그대로다. 이번 작업으로 다중 로봇 충돌 안전을 검증하지 않았다.
- 실물 Pinky, 모터, 승강기, 자동문은 코드 경로가 거부한다.

## 9. Phase 3

Gazebo 다중 로봇으로 넘어갈 수 없다. 단일 Pinky의 Nav2 goal id와 FMS task id가 같은 주행 로그에 남기 전에는 Phase 3의 시뮬레이터 항목을 시작하지 않는다.

Mock 3대의 작업 격리와 구역 예약은 Phase 1 결과로 남아 있다. 그것은 교통 안전 검증이 아니다.
