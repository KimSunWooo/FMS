# Phase 2.2.1 회귀

시각은 KST(UTC+9)다. 원본은 `docs/phase2-2-1-logs/regression/`다. 이 항목들은 Phase 2.2에서 이미 통과한 동작의 재확인이다. 기능을 새로 만들지 않았다.

unsourced pytest는 `env -i`로 `backend/tests`를 실행해 55 passed다. ROS 환경을 넣지 않았다.

## 요약

| 항목 | 결과 |
| --- | --- |
| 물리 등록, cmd_vel, mock-observe, health | PASS |
| 정상 세션 종료 | PASS |
| Heartbeat 만료 | PASS |
| 종료 세션 재배정 | PASS. assignment 409 `STALE_OWNER` |
| 실행 중 두 번째 세션 | PASS. HTTP 409 |
| 중복 claim | PASS. `send_navigate_to_pose` false, goal 행 1 |
| Stale owner epoch | PASS. disposition `rejected_stale_owner`, deadbeef UUID 미저장 |
| 정상 완료 | 반복 주행 11회 `COMPLETED`. stale 케이스도 주입 후 원래 goal이 완료됨 |
| Nav2 취소 | PASS. HTTP 202, FMS `CANCELED`, 에이전트 Nav2 status 5 |
| Nav2 abort | PASS. `FAILED`, `nav2_goal_aborted` |
| Agent SIGKILL | PASS. `RECOVERY_REQUIRED`, `agent_session_lost` |
| FMS 재시작 | PASS. `process_restart`. 재기동 health preflight `not_ready` |
| Lease 만료 | PASS. `lease_invalid`. 늦은 결과는 `rejected_task_not_running` |
| UUID 보존 | 위 실패·복구 케이스에서 수락 UUID가 유지됨 |
| 네트워크만 단절 | NOT_RUN |
| 강제 DB 쓰기 실패 | NOT_RUN |

## 식별자

| 케이스 | task | Nav2 UUID | 최종 |
| --- | --- | --- | --- |
| stale | `c0e28417-4b90-4444-9393-61c5099a5ca8` | `6b6347fd-5324-4128-ac67-9d6eeb5f1f4f` | COMPLETED. 주입 UUID는 거부 |
| cancel | `e334f7da-5429-428b-936b-c27bd863e58b` | `14dc955c-96fd-45cd-b002-12b9da3713a8` | CANCELED |
| abort | `de558446-2f6c-4aba-87bd-079cc2c91c0a` | `bdaaa2ae-8e6f-4eeb-a53e-d35f108fed3f` | FAILED. 목표 (3, 0) |
| kill | `9b546580-660d-4b85-8637-a55843654f8f` | `4e329832-aa01-42c9-b055-23773f0448ce` | RECOVERY_REQUIRED |
| restart | `dbdfac9b-19fd-44a1-987f-f7d9a9779f71` | `e7f054b0-e265-41b5-8d38-d8a63b09895a` | RECOVERY_REQUIRED |
| lease | 포트 8743, DB `data/fms-phase221-lease.sqlite3` | `519bf6a4-d3d6-45b8-b19c-cfcc8cf87297` | RECOVERY_REQUIRED. evidence null |

Lease DB는 새로 만들었다. `fms-phase22-lease2.sqlite3`와 `fms-gazebo-e2e.sqlite3`는 지우지 않았다. Lease TTL 8초, heartbeat 한계 120초, 주행 중 heartbeat 없음. 결과 전송은 recovery 이후 owner epoch를 사용했다. 상태는 `COMPLETED`가 되지 않았고 UUID는 저장됐다.

SIGKILL의 저장 이유는 `agent_session_lost`다. 세션 heartbeat 8초가 프로세스 종료보다 먼저 세션을 지운다. `heartbeat_timeout` 문자열은 세션 객체가 살아 있는 경로다.

FMS 재시작은 SIGTERM 뒤 같은 DB로 uvicorn 하나만 다시 띄웠다. 그 프로세스 환경에는 `ROS_DISTRO`가 없다. 재시작 후 gateway boot id는 `2a0e928f-c982-4d89-8d48-eb687faae041`이다.

종료된 세션 케이스는 세션을 끊은 뒤 같은 session id로 assignment를 두 번 호출해 둘 다 409였다. 그 task는 Nav2 UUID 없이 `RECOVERY_REQUIRED`였고, `FAILED`로 닫으면서 goal을 다시 보내지 않았다.

## 차단

| 호출 | HTTP | 코드 |
| --- | --- | --- |
| `POST /api/robots` mode `physical` | 403 | `PHYSICAL_CONTROL_BLOCKED` |
| `POST /api/robots/pinky-sim/cmd_vel` | 404 | 라우트 없음 |
| `POST /api/robots/pinky-sim/mock-observe` | 403 | Gazebo 로봇의 mock 관측 거부 |
| health `physical_control` | | `blocked` |
| health `ros_bridge` | | `disabled` |
| Dashboard `http://127.0.0.1:8742/` | 200 | 화면 동작은 변경하지 않음 |

실물 모터, 엘리베이터, 문, `/cmd_vel` 발행은 하지 않았다.
