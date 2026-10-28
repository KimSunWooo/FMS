# FMS Task 취소 및 Pinky-1 5cm 주행 진단 보고

작성 시각: 2026-10-28

이번 작업은 기존 Task Manager, Scheduler, Reservation Manager, Route Planner, Task Receiver, Frontend를 유지한 채 Task 취소와 5cm 전진 실패 원인을 코드에 반영했다. 실물 5cm 주행 POST는 실행하지 않았다. 로봇이 움직이지 않았으므로 5cm 성공으로 기록하지 않는다.

## 1. 기존 Task 취소 기능 분석

기존 코드에 취소의 뼈대는 있었다.

- `POST /api/fleet-tasks/{task_id}/cancel`가 이미 있었다.
- `fleet_task_machine.py`는 `CANCELLED`를 종단 상태로 두고, `CREATED`부터 `WAITING`까지는 로컬 취소가 가능했다.
- Reservation Manager의 해제는 해당 `task_id`의 confirmed 예약만 `released`로 바꾼다. 다른 로봇의 행은 건드리지 않는다.
- Frontend 대기열에는 취소 버튼이 있었으나, 확인 Dialog, 예약 해제 건수, Receiver 취소 결과, 보류 상태가 없었다.
- Dispatcher는 수신 ACK만 처리했다. Receiver에 저장된 Task를 폐기하는 취소 ACK는 없었다.
- `ACKNOWLEDGED` 이후 취소는 DB만 바꾸면 로봇 inbox의 Task가 실행 가능 상태로 남았다.
- 통신 실패를 취소 성공으로 처리하는 경로가 없었다. 이번 변경 전에도 자동 성공은 하지 않았고, 보류 상태 자체가 없었다.
- `FAILED`는 실패 이력을 남기는 상태였다. 취소로 덮어쓰면 안 된다.
- 단위 테스트는 계획·예약·전달·재시작 복구를 다뤘고, 상태별 취소와 Receiver 취소 ACK는 없었다.

중복 API는 만들지 않았다. 기존 cancel 경로, `_release`, 상태 기계, Receiver stdin JSON을 확장했다.

## 2. 수정한 Task 취소 API 및 Frontend

상태별 동작은 다음과 같다.

| 상태 | 처리 |
|---|---|
| CREATED, PLANNING, PLANNED, RESERVED, WAITING, REPLANNING, UNREACHABLE | 해당 Task 예약만 해제하고 `CANCELLED`. outcome `local_only` |
| DISPATCHING | 예약을 먼저 해제하고 `CANCEL_PENDING`. 진행 중인 전달이 끝나면 Receiver 취소를 이어서 요청 |
| DELIVERED, ACKNOWLEDGED | `CANCEL_PENDING` 후 Receiver `action=cancel`. 취소 ACK가 확인된 뒤에만 `CANCELLED` |
| CANCELLED | 같은 요청으로 멱등 반환. 새 이벤트를 추가하지 않음 |
| FAILED | HTTP 409. 실패 이력 유지 |
| CANCEL_PENDING | 재요청 시 Receiver 취소를 다시 시도. 타임아웃·SSH 실패는 성공으로 바꾸지 않음 |

`CANCEL_PENDING`은 종단 상태가 아니다. 재시작 복구는 이 상태를 재계획하거나 전달 완료로 바꾸지 않는다. `CANCELLED`와 `CANCEL_PENDING`은 다시 dispatch하면 HTTP 409다.

취소 기록은 `fleet_tasks.cancellation_json`에 저장한다. 스키마 버전 4에서 컬럼이 없으면 `ALTER TABLE`만 수행하고 기존 행을 삭제하지 않는다. 기록 항목은 cancellation request id, 시각, 사유, outcome, 해제된 예약 수, Receiver 결과 또는 오류다.

Receiver 수신 ACK와 취소 ACK는 구분한다. 취소 ACK의 `kind`는 `cancel_ack`다. 원래 수신 ACK와 payload hash는 유지하고 `executable=false`만 기록한다. 가상 로봇 취소는 `channel=simulated`이며 `CANCEL_ACKNOWLEDGED`로 표시하지 않는다.

전달은 `DISPATCHING`을 커밋한 뒤 Receiver를 호출한다. 그 사이에 취소가 들어오면 전달 성공이어도 `ACKNOWLEDGED`로 올리지 않고 취소 ACK를 요청한다. 전달 자체가 실패하고 Receiver에 저장되지 않았으면 `not_delivered`로 `CANCELLED`가 된다.

Frontend 작업 대기열은 Task 선택, 상세, 취소 버튼, 확인 Dialog, 처리 결과, 예약 해제 건수, 실패 사유를 보여 준다. 취소 버튼은 `FAILED`, `UNREACHABLE`, `CANCELLED`에서 비활성화되고, `CANCEL_PENDING`에서는 "취소 재시도"다. `CANCELLED`와 `CANCEL_PENDING`의 지도 polyline은 그리지 않는다. 다른 Task를 선택하면 그 Task의 저장 경로는 그대로 남는다. 5cm Goal 취소 버튼은 Task 취소와 별도다.

## 3. 예약 해제 및 상태 전이 테스트

`backend/tests/test_fleet_task_cancel.py`와 전체 pytest를 실행했다. ROS mock과 실물 호출은 분리되어 있다. pytest는 SSH로 로봇에 취소나 주행을 보내지 않는다.

확인된 시나리오:

1. CREATED 취소
2. PLANNED 취소
3. RESERVED 취소와 해당 Task 예약만 삭제
4. WAITING 취소
5. 같은 Task 중복 취소는 멱등
6. 다른 로봇 Task의 confirmed 예약은 유지
7. DISPATCHING 중 취소는 최종 `ACKNOWLEDGED`가 되지 않고 `CANCELLED`와 `CANCEL_ACKNOWLEDGED`
8. 로컬 Receiver 취소 ACK. 원래 수신 hash 유지, `executable=false`
9. Receiver 타임아웃은 `CANCEL_PENDING`. inbox 파일은 그대로
10. 새 `FleetTaskService`로 다시 열어도 `CANCEL_PENDING` 후 재시도로 `CANCELLED`가 유지되고, 취소된 Task dispatch는 409
11. FAILED 취소는 409이고 실패 문구가 남음
12. 가상 취소는 simulated이며 physical cancel ACK가 아님

전체 `pytest -q --tb=line`은 이번 백엔드 수정 이후 종료 코드 0이다. 이전 104개에 취소·속도 분류 테스트를 더한 약 112개다. 그 뒤 Frontend 대기열 문구만 고쳤고 백엔드는 다시 바꾸지 않았다.

## 4. 실제 Pinky Receiver 취소 처리 결과

이미 수신 ACK가 있던 Pinky-1 Task를 취소 API로 처리했다. 새 Task를 만들거나 dispatch하지 않았다.

| 항목 | 값 |
|---|---|
| Task ID | `5b3bc030-0b45-4688-bd97-7618c1102fde` |
| Robot ID | `pinky-1` |
| Cancellation Request ID | `1f9fd3e5-8cfd-4b89-a7a5-845ec9d8e603` |
| 요청 시각 | `2026-10-28T11:02:55.884521+00:00` |
| 사유 | `acceptance cancel verification` |
| FMS 상태 | `CANCELLED` |
| ACK 상태 | `CANCEL_ACKNOWLEDGED` |
| outcome | `cancel_ack` |
| 해제한 예약 | 29건. 이 Task만 |
| 남은 confirmed 예약 | 194건. pinky-1, pinky-2, pinky-3 모두 포함 |

원격 inbox `fms_task_receiver/inbox/5b3bc030-0b45-4688-bd97-7618c1102fde.json`을 읽어서 확인했다.

- `executable`은 `false`
- `cancellation.kind`는 `cancel_ack`
- `cancellation.robot_id`는 `pinky-1`
- `cancellation.execute`는 `false`
- 원래 수신 ACK와 payload hash는 그대로다
- 같은 Task 재전송은 HTTP 409 `Cannot dispatch CANCELLED`

관제 화면에서 이 Task를 선택하면 "취소 완료 · 예약 해제 29건 · cancel_ack"가 보이고, 취소 버튼과 전달 버튼은 비활성화된다. 선택 중 지도 polyline은 0개다. 이어서 pinky-3 `RESERVED` Task를 선택하면 polyline 3개가 다시 표시된다. 다른 Task의 저장 경로는 지우지 않았다.

## 5. 5cm 주행 실패의 확인된 원인

과거 실주행 기록은 다음이다. 이번 작업에서 그 주행을 다시 실행하지 않았다.

- DriveOnHeading Goal은 수락됨
- `/cmd_vel_nav`와 `/cmd_vel_smoothed`의 `linear.x`는 0.04
- `/cmd_vel` 메시지는 관측되지 않음
- Nav2는 time allowance 초과
- Action status 6, 이름 `ABORTED`
- 실제 이동 0.0 m
- Collision Monitor 로그에 StopZone polygon 정지와 invalid source가 있었음

이번 정지 상태 진단에서 확인한 원인은 StopZone 점유다. 이동 명령은 보내지 않았다.

- 첫 관측: 반경 0.18 m 안 반환 49점, 최근접 약 0.025 m
- API 진단 GET: 46점, 최근접 0.102 m, `stop_zone_would_stop=true`, `motion_command_sent=false`
- `min_points`는 4이므로 두 표본 모두 정지 조건이다
- scan frame은 `rplidar_link`
- `base_footprint` → `rplidar_link`는 x=-0.017 m, z=0.125 m, yaw 180°
- 레이저 프레임 기준 가까운 반환은 왼쪽 구간에 몰려 있다. yaw가 180°이므로 이것을 로봇의 왼쪽이라고 단정하지 않는다. 로봇 구조물인지 맞닿은 물체인지도 이번 표본만으로 구분하지 않는다

그래서 과거 `/cmd_vel` 공백과 status 6은 Collision Monitor가 StopZone 때문에 출력을 멈춘 경우와 맞다. 다만 이번 턴에는 Goal을 보내지 않았으므로, 그 실패 순간의 세 토픽을 다시 동시에 기록하지는 못했다. 속도 0.04 m/s와 time allowance 10초는 바꾸지 않았다.

`start()`는 진단에서 `stop_zone_block_reason`이 있으면 state `blocked`, HTTP 409로 끝내고 DriveOnHeading을 보내지 않는다. 이것은 우회가 아니라 거절이다. 라이브 POST는 호출하지 않았으므로 그 409 응답 자체는 서버에서 발생시키지 않았다. 거절에 쓰는 진단 값은 위의 GET으로 확인했다.

invalid source 로그는 이번 정지 표본에서 재현되지 않았다. scan은 들어왔고 TF도 있었다. 과거 로그의 유일한 원인이라고 쓰지 않는다.

## 6. ROS 속도 명령 체인 분석 결과

도메인 24에서 구독만 수행했다. publish는 없다.

| Topic | 타입 | QoS | Publisher | Subscriber |
|---|---|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` RELIABLE | behavior_server, controller_server | velocity_smoother |
| `/cmd_vel_smoothed` | Twist RELIABLE | velocity_smoother | collision_monitor |
| `/cmd_vel` | Twist RELIABLE | collision_monitor | pinky_bringup |

세 구간 모두 Twist다. Twist와 TwistStamped가 어긋나서 `/cmd_vel`이 사라진 것은 아니다. `enable_stamped_cmd_vel`은 false다. launch는 behavior_server, controller_server, velocity_smoother의 `cmd_vel`을 `cmd_vel_nav`로 바꾸고, collision_monitor는 파라미터의 `cmd_vel_smoothed` 입력과 `cmd_vel` 출력을 쓴다. pinky_bringup은 `cmd_vel`의 Twist를 구독한다.

`/scan` publisher는 `sllidar_node` RELIABLE이고, collision_monitor와 AMCL, local costmap은 BEST_EFFORT로 구독한다. Reliable writer와 best-effort reader 조합은 허용된다. AMCL TF가 이미 동작하므로 scan이 완전히 끊긴 상태는 아니다. collision_monitor lifecycle은 active다.

정지 중에는 세 cmd_vel 토픽에 샘플이 없어 속도 케이스는 `null`이다. 전진 명령이 없었으므로 Case A/B/C로 분류하지 않는다.

주행 중 관측은 `nav2_forward_5cm_worker.py`의 `MotionWatch`가 Goal과 같은 시간에 `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel`, `/odom`을 기록하도록 넣어 두었다. 이번 턴에는 Goal을 보내지 않아 이 기록은 비어 있다.

- Case A: nav와 smoothed가 양수이고 cmd_vel이 0이면 Collision Monitor 차단 후보
- Case B: nav와 smoothed가 양수이고 cmd_vel이 없으면 발행 중단 또는 타입·통신 문제 후보. 과거 실패의 형태와 같다
- Case C: cmd_vel이 양수인데 odometry 이동이 없으면 bringup·모터·odom 후보
- Case D: odometry는 움직였는데 map TF만 0이면 측정 오류. 성공 판정은 둘을 같이 본다

## 7. Collision Monitor 진단 결과

설정은 유지했다.

- 입력 `/cmd_vel_smoothed`, 출력 `/cmd_vel`
- StopZone circle, 반경 0.18 m, min_points 4, action stop
- source `/scan`, source_timeout 2.0 s, stop_pub_timeout 2.0 s
- lifecycle active

확인 결과 StopZone 안에 반환이 충분하다. 반경을 줄이거나 모니터를 끄거나 `/cmd_vel`을 직접 발행하지 않았다. 장애물이 있으면 소프트웨어가 정지를 우회하지 않고, 관제 화면이 이유를 보여 준다.

화면의 "속도 체인 진단"은 GET이며 Goal을 만들지 않는다. 표시된 문구:

`Collision Monitor 진단: StopZone 안에 반환이 있어 정지 가능 · scan rplidar_link 최소 0.102m · 반경 내 46점`

5cm 시작 버튼은 누르지 않았다. 테스트 Goal 취소 버튼은 진행 중인 Goal이 없어 비활성화였다. Nav2 READY와 TF 수신은 표시됐다. 현재 5cm 상태는 API 재시작 뒤 메모리 초기값 `idle`이다. 이전 status 6 기록은 프로세스 메모리에만 있었고, 재시작이 그 기록을 지운 것이다. 성공으로 바꾼 것이 아니다.

## 8. 수정 파일 목록

- `robot_receiver/pinky_task_receiver.py` — `action=cancel`, 취소 ACK, `executable=false`
- `backend/app/services/fleet_dispatcher.py` — 수신과 구분되는 SSH·로컬·가상 취소
- `backend/app/services/fleet_task_machine.py` — `CANCEL_PENDING`, FAILED는 취소로 이동하지 않음
- `backend/app/services/fleet_task_service.py` — 상태별 취소, 전달·취소 경쟁, 재전송 거부
- `backend/app/models/tables.py`, `backend/app/core/database.py` — `cancellation_json`, 스키마 4
- `backend/app/api/fleet_tasks.py` — 취소 사유. 본문 없는 기존 호출도 허용
- `backend/app/services/velocity_chain.py` — 속도 케이스와 StopZone 거절 문구
- `backend/app/services/nav2_chain_probe.py` — 구독 전용 진단. publish 없음
- `backend/app/services/nav2_forward_5cm_worker.py` — Goal과 동시에 속도·odom·map TF 기록. 거리와 속도 값은 유지
- `backend/app/services/forward_5cm_service.py` — StopZone이면 Goal 전에 blocked
- `backend/app/api/physical.py` — 진단 GET. 시작 POST와 분리
- `backend/tests/test_fleet_task_cancel.py`
- `backend/tests/test_static_isolation.py` — 진단 프로세스를 API import 금지 목록에 포함
- `frontend/src/components/fleet-task-panel.tsx` — 취소 Dialog, 결과, 지도에서 취소 Task 경로 숨김
- `frontend/src/components/fleet-console.tsx` — Nav2, TF, Goal, 거리, 실패 근거, 진단, 테스트 취소

목표 거리 0.05 m, 속도 0.04 m/s, time allowance 10초, action `/drive_on_heading`은 변경하지 않았다. 성공으로 보려면 map TF 전진과 odometry 전진이 둘 다 0.035–0.10 m이고 둘의 차이가 0.03 m 이하여야 한다. 화면 허용 오차는 목표 5 cm보다 1.5 cm 부족부터 5 cm 초과까지다. map TF만으로 성공 처리하지 않는다. Nav2 status 4 `SUCCEEDED`가 필요하다. status 6은 `ABORTED`로 이름을 붙인다.

## 9. 수정 전후 테스트 결과

| 검사 | 결과 |
|---|---|
| 백엔드 pytest 전체 | 통과. 종료 코드 0 |
| Task 취소·예약 격리·Receiver ACK·타임아웃 보류·재시작·재전송 거부 | 통과. 임시 DB와 임시 inbox |
| 가상 취소가 physical ACK로 둔갑하지 않음 | 통과 |
| 속도 케이스 A/B/C와 StopZone 문구 | 통과. ROS 프로세스 없음 |
| Frontend `tsc --noEmit` | 통과. 대기열 문구 수정 후 |
| Frontend 격리 빌드 `FMS_NEXT_DIST_DIR=.next-verify` | 통과. `/`, `/api/physical-5cm`. 페이지 JS 약 33.3 kB |
| 격리 빌드가 tsconfig에 넣은 `.next-verify/types` include | 빌드 후 제거. `.next-verify`는 gitignore |
| 실물 5cm POST | 실행하지 않음 |
| 기존 Task·예약 삭제, git reset, push | 하지 않음 |

회귀로 유지된 항목: Pinky-1 SSH 연결, Pinky-2·3 가상 등록, `1f_map_f`, 승인된 home, 시설 좌표 없음, A*와 Yen, 예약, 스케줄러, 가상 수신 상태, 기존 예약 194건. 헬스의 `phase` 필드는 계속 2다. 이번 작업의 대상이 아니다.

## 10. 실제 주행 테스트 결과

**BLOCKED.** 실물 전진은 실행하지 않았고 성공이 아니다.

막힌 이유:

1. 정지 중 scan에서 StopZone 반경 0.18 m 안에 46–49개 반환이 있다. 최근접은 약 0.025–0.102 m다.
2. Collision Monitor는 active이고 이 조건에서 정지한다.
3. 현장 운영자가 주변이 비었음을 확인하고 시작 버튼을 누르기 전에는 주행 POST를 보내지 않는다.
4. 자동 테스트와 이번 검증 스크립트는 5cm POST를 호출하지 않았다.

이동 없이 확인한 항목은 통과다. 진단 API, Nav2 READY, TF, scan 수신, Collision Monitor 설정, 타입 일치, Goal을 보내지 않는 거절 조건, 진행 중 Goal이 없어 비활성인 테스트 취소 버튼.

운영자가 주변을 비우고 방향을 확인한 뒤 직접 시작해야 다음을 동시에 기록할 수 있다. Goal 상태, 세 cmd_vel, odometry 변위, map TF 변위, 종료 후 정지, Collision Monitor active. 그때까지 5cm는 BLOCKED다.

## 11. 남아 있는 문제와 해결 조건

- 과거 status 6 주행과 이번 StopZone 점유는 정황이 맞지만, 같은 Goal 동안의 세 토픽 동시 기록은 아직 없다. 해결 조건은 StopZone이 비고, 운영자가 시작을 누른 뒤 `MotionWatch` 기록이 Case를 나누는 것이다.
- 가까운 반환이 로봇 몸체인지 외부 물체인지는 확정하지 않았다. 반경을 줄여 해결하지 않는다. 센서 위치와 실제 주변을 현장에서 봐야 한다.
- 과거 invalid source는 이번 표본에서 재현되지 않았다. 해결 여부를 말하지 않는다.
- `CANCEL_PENDING`의 서버 재시작 유지는 단위 테스트로 확인했다. 라이브 Pinky 취소는 ACK까지 끝났으므로 보류 상태로 남지 않았다.
- 헬스 `phase` 값 2, 카드 pose와 live TF의 간헐적 불일치, 시설 좌표 없음, 예약 슬롯 지연 상한은 이번 범위 밖이다.
- 승인된 home과 기존 Task 데이터는 되돌리거나 삭제하지 않았다.
