# Phase 2.2 안정화 보고

시각은 KST(UTC+9)다. 로그 원문의 타임스탬프는 UTC다.

## 판정

구현과 Gazebo 검증은 따로 적는다.

| 구분 | 판정 |
| --- | --- |
| 세션 생명주기, Nav2 UUID 저장, 재전송 금지 | 구현됨. unsourced pytest 55 passed. 허용 오차 0.25 m는 그대로다 |
| 계획된 Gazebo 반복 주행 5회 | 실행됨. **0/5 PASS. 성공률 0%**. Nav2 status 4여도 관측 거리가 0.25 m를 넘으면 FMS는 `COMPLETED`가 되지 않았다 |
| 그 밖의 실측 완료 | 5회 표본 밖에 거리가 0.25 m 이하인 `COMPLETED`가 있다. 표본 성공률에 넣지 않는다 |
| Phase 3 다중 로봇 주행 | 하지 않음 |

실패 기록을 지우거나 오차 기준을 늘리지 않았다. Pinky 제조사 파라미터의 `SimpleGoalChecker`도 바꾸지 않았다. 그 checker는 `stateful: true`이고 xy 허용이 0.25 m다. 반경 안에 한 번 들어가면 xy가 잠겨, 이후 로봇이 반경 밖으로 나가도 Nav2는 성공을 낼 수 있다. FMS는 그 성공만으로 완료하지 않는다.

## 1. Git 증거

기준 저장소로 적힌 GitHub `https://github.com/KimSunWooo/FMS.git` 의 `main`은 `be5d319a3020fd5722517a02977aabad8752ddcf`다. GitHub API는 `dd80162c34dc0e8818b9c51d3339b6240d23e887`에 대해 `No commit found`를 반환했다. GitHub에는 Phase 2.1 PR이 없다.

Origin에는 PR이 있다.

| 항목 | 값 |
| --- | --- |
| PR #1 | 열림. https://cursor.com/codebase/kssgss94/multi-robot-fms/pull/1 |
| head | `cursor/phase2-1-gazebo-e2e-8a20` |
| SHA | `dd80162c34dc0e8818b9c51d3339b6240d23e887` |
| base | `main` |
| PR #2 | 열림. PR #1 위에 쌓여 있다. https://cursor.com/codebase/kssgss94/multi-robot-fms/pull/2 |
| PR #2 head | `cursor/phase2-2-stability-8a20` |

Phase 2.1 브랜치와 `dd80162`는 Origin에만 푸시되었다. GitHub에 없는 이유는 그 푸시가 GitHub remote로 가지 않았기 때문이다. PR 생성을 추정하지 않는다.

이 작업을 시작했을 때 로컬 `cursor/phase2-1-gazebo-e2e-8a20`은 `origin/cursor/phase2-1-gazebo-e2e-8a20`의 `dd80162`와 같았고, 작업 트리는 깨끗했다. 보존할 미커밋 변경은 없었다.

Phase 2.1 DB `data/fms-gazebo-e2e.sqlite3`의 task `20a239aa-4de1-4f0c-8869-c3c6862c400b`는 여전히 `COMPLETED`다. 그 행을 지우지 않았다.

## 2. 구현

FMS API는 Nav2 클라이언트를 열지 않는다. 로봇 에이전트만 `/navigate_to_pose`를 보낸다.

- 정상 종료는 세션 stop이다. pose는 `agent_lost`가 되고 신뢰할 수 없는 상태가 된다. 그 뒤 새 task는 새 세션과 map pose가 있기 전에는 `PENDING`에 남는다.
- 외부 세션의 heartbeat가 끊기면 세션이 종료된다. 진행 중 goal은 `RECOVERY_REQUIRED`가 되고 자동으로 다시 보내지 않는다. 프로세스 종료와 네트워크 단절은 같은 세션 상실로 본다.
- 종료된 세션에는 task를 배정하지 않는다. 같은 로봇에 세션이 살아 있으면 새 세션 open은 409다.
- 결과의 owner epoch, execution, goal, session이 맞지 않으면 거절한다. 이전 epoch의 성공은 완료가 되지 않는다.
- 이미 claim된 goal에 대한 두 번째 claim은 `send_navigate_to_pose=false`이고 navigation goal 행을 추가하지 않는다.
- 에이전트는 Nav2 status 목록이 없거나 status 1, 2, 3이 있으면 새 goal을 보내지 않는다.
- Nav2가 goal을 수락한 직후 UUID를 FMS에 저장한다. 결과 시점까지 기다리지 않는다. 그래서 주행 중 프로세스가 죽어도 UUID가 DB에 남는다.
- 성공, abort, 취소, 거부, 타임아웃에서 UUID가 있으면 저장한다. 다른 UUID로 덮어쓰지 않는다. UUID가 없으면 `nav2_action_goal_id_absent_reason`에 이유를 남긴다. schema 3은 이 컬럼만 추가하고 기존 행을 삭제하지 않는다.
- recovery로 이미 끝난 task에, fence가 맞는 늦은 결과가 오면 UUID는 저장하고 상태는 `RECOVERY_REQUIRED`로 둔다. disposition은 `rejected_task_not_running`이다.
- `timed_out`과 불확실한 상태는 성공이 아니고 재전송하지 않는다.
- Gazebo 출처의 성공에 UUID가 없으면 `success_without_nav2_goal_id`다. mock 완료는 Nav2 UUID를 요구하지 않는다.
- 에이전트의 FMS 쓰기가 HTTP 오류면 `task_completed`를 true로 두지 않는다. 강제 DB 장애를 Gazebo에서 주입한 실험은 `NOT_RUN`이다. 그 분기는 pytest의 `note_fms_failure`가 고정한다.
- 첫 pose 불일치는 `RUNNING`과 `observation_mismatch`로 남는다. 그 상태에서 세션이 끝나면 이유는 `pose_mismatch`다. 두 번째 불일치도 `pose_mismatch`다.

## 3. 반복 주행 5회

환경은 Gazebo다. 도메인 73, localhost only, Cyclone DDS. 월드 `pinky_factory`. 실물 bringup은 실행하지 않았다. 목표점은 공장 벽(x=±2, y=±1.5) 안이고, `config/zones.json`의 ZONE_E(x 0.4–2.6, y 0.4–2.2) 밖이다.

먼저 PYTHONPATH를 `/workspace`만으로 둔 기동은 `Nav2 action libraries are not importable`로 끝났고 goal을 보내지 않았다. 그 5회는 점수에 넣지 않는다. 로그: [phase2-2-logs/import-miss/summary.json](phase2-2-logs/import-miss/summary.json).

아래 5회는 ROS Jazzy와 Pinky overlay를 source한 뒤의 독립 주행이다. 각 회는 새 에이전트 프로세스다. 원본: [phase2-2-logs/repeats/summary.json](phase2-2-logs/repeats/summary.json).

판정 규칙은 그대로다. Nav2 성공, map pose, 거리 0.25 m 이하, lease와 epoch와 세션이 맞을 때만 `COMPLETED`다.

| # | 목표 (map) | 시작 pose | 최종 pose | 거리 m | Nav2 | FMS 스냅샷 | Goal UUID | 전송→결과 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | (-0.8, 0.2) `run1-west` | (0.4670, 0.1162) | (-1.0000, -0.0118) | 0.2913 | status 4 succeeded | `RECOVERY_REQUIRED` | `81e11f28-8f8f-4423-9cfc-9c20ee44ae0c` | 15:06:20.870 → 15:07:14.457 KST |
| 2 | (0.2, -0.8) `run2-south` | (-0.9981, -0.0167) | (-0.0146, -0.9376) | 0.2549 | status 4 succeeded | `RECOVERY_REQUIRED` | `75ff10ca-5d03-4681-a4aa-6ede3c9bb303` | 15:07:21.014 → 15:07:56.868 KST |
| 3 | (-0.2, 0.8) `run3-north` | (-0.0190, -0.9501) | (-0.0989, 0.5260) | 0.2921 | status 4 succeeded | `RECOVERY_REQUIRED` | `ec52c958-dde6-46c0-b355-53a9f5d9c6d8` | 15:08:03.574 → 15:08:33.983 KST |
| 4 | (0.8, -0.3) `run4-east` | (-0.0832, 0.5312) | (0.6099, -0.1213) | 0.2609 | status 4 succeeded | `RECOVERY_REQUIRED` | `4913b70d-4d8b-4bdf-8455-226d91c6e339` | 15:08:40.781 → 15:09:01.039 KST |
| 5 | (-0.6, -0.4) `run5-southwest` | (0.5966, -0.1185) | (-0.2648, -0.2750) | 0.3578 | status 4 succeeded | `RECOVERY_REQUIRED` | `86b90a3e-ff91-4233-bb5a-f12ee6d84e5e` | 15:09:07.427 → 15:09:37.225 KST |

다섯 모두 **FAIL**. 성공 0, 실패 5, 성공률 0%.

각 회의 연계는 맞다. 예로 1회는 task `c5f36e8d-75c3-4d27-b12a-fb8f73d27e65`, execution `2d013042-1613-436d-9f24-4a40df081191`, FMS goal `84c4b61d-7c87-419e-a32b-2655faa6def7`, Nav2 UUID `81e11f28-8f8f-4423-9cfc-9c20ee44ae0c`다. 에이전트 JSON의 UUID와 DB `navigation_goals.nav2_action_goal_id`가 같다. disposition은 `observation_mismatch`다. 그 스냅샷의 recovery 이유는 `agent_session_lost`다. 이 5회를 돌릴 때는 세션 종료가 pose hold와 구분되기 전이었고, 첫 불일치 직후 에이전트가 stop하면서 그 이유가 붙었다. 현재 코드는 같은 순서에 `pose_mismatch`를 쓴다. 저장된 5회 로그를 고쳐 쓰지 않았다.

홀드는 같은 goal을 다시 보내지 않고 `FAILED`로 닫았다. DB의 최종 status는 그 닫기 이후의 값이다. 점수에 쓰는 상태는 주행 직후 스냅샷이다.

## 4. 5회 표본에 넣지 않은 완료

거리가 0.25 m 이하여서 FMS가 `COMPLETED`가 된 주행이 있다. 장애 하니스가 goal 종료보다 늦게 끼어들려다 놓친 것이거나, stale 거절 뒤에 원래 goal이 끝난 것이다. 5회 성공률의 분모에 넣지 않는다.

| Task | 목표 | 최종 pose | 거리 m | Nav2 UUID | 비고 |
| --- | --- | --- | --- | --- | --- |
| `4ba9ed8e-0106-4cbe-a9b8-36892741afad` | (0.8, 0.25) | (0.6705, 0.0424) | 0.2446 | `d2eac115-f219-49c7-b538-ea0e6e31c5a3` | 주입 전에 종료. stale 시험이 아님 |
| `b3bdcba5-23a8-4ef1-97e6-6c51c028410f` | (-0.9, -0.2) | (-1.0182, -0.4018) | 0.2339 | `36937f62-ef46-4e37-8e70-bf888e0eae8c` | 주입 전에 종료 |
| `8b16d7a7-6251-49f2-b036-e07de313822b` | (-0.2, -0.7) | (-0.1086, -0.4763) | 0.2417 | `2dd78289-6808-4af5-a864-66e8ab016cf9` | 재시작 시험이 아님 |
| `9a60a5e0-2dd1-4ecc-9c3b-b6298e0f7a58` | (0.7, -0.2) | (0.4942, -0.0666) | 0.2453 | `d261bdd4-5024-4342-b05e-76f1cb03f4ab` | 주행 중 stale 거절 뒤의 원래 goal 완료 |

`9a60a5e0`만 장애 시험 PASS와 겹친다. 그 판정은 stale 거절이지, 5회 표본의 성공이 아니다.

## 5. 회귀

unsourced pytest는 55개다. ROS를 source하지 않은 환경에서 55 passed. source된 셸에서 돌리면 preflight 가정이 깨지므로, 그 실패는 제품 실패로 세지 않는다.

| 확인 | 결과 |
| --- | --- |
| Dashboard `http://127.0.0.1:8742/` | 200 |
| `physical_control` | `blocked` |
| physical 로봇 등록 | 403 `PHYSICAL_CONTROL_BLOCKED` |
| Gazebo 로봇 `mock-observe` | 403. mock 스냅샷은 Gazebo를 열지 않음 |
| `POST /api/robots/pinky-1/cmd_vel` | 404. 속도 라우트가 없다 |
| API 프로세스 | 시험 도중 재시작 하니스가 ROS 환경을 물려준 적이 있다. 그 프로세스는 내렸고, 지금의 API는 ROS 환경이 없다. health의 gazebo preflight는 `not_ready`다. 에이전트만 DDS에 붙는다 |
| Phase 2.1 task 행 | `fms-gazebo-e2e.sqlite3`에 그대로 있다 |
| Pinky 제조사 트리 | 읽기만 했다. Open-RMF 트리는 이 작업 공간에 없고 수정하지 않았다 |

`gazebo_e2e` health 필드는 아직 고정 문자열 `NOT_RUN`이다. 이 문서의 주행 기록과 그 필드는 다르다.
