# FMS 1~5단계 최종 인수 검증

검증 시각: 2026-10-28 19:13–19:24 KST  
대상: `/home/sw/DevSpace/FMS`  
관제 화면: `http://127.0.0.1:8742/`  
API: `http://127.0.0.1:8741/`

이번 검증은 새 주행 기능을 추가하지 않았다. 기존 Task와 Reservation은 삭제하지 않았다. Nav2 Goal, DriveOnHeading, `cmd_vel`, 모터 명령은 보내지 않았다. Pinky-1에 새 Task를 전송하지 않았다. 등록 좌표는 되돌리지 않았다.

판정은 PASS, FAIL, BLOCKED, NOT RUN만 사용한다. 실행하지 않은 실물 이동은 PASS로 적지 않는다.

## 1. 전체 시스템 구성

| 구성 | 확인 값 | 판정 |
| --- | --- | --- |
| API | FastAPI, health `status=ok`, database `ok` | PASS |
| 관제 화면 | Next.js dev, 제목 Care-Move FMS | PASS |
| 환경 | `environment_id=care-move-fms-local` | PASS |
| 실행 모드 | `execution_mode=mock`, `motion_backend=mock-only`, `physical_control=blocked` | PASS |
| Health의 `phase` | 응답 필드가 `2` | 아래 미해결 이슈 |
| Fleet Task | `/api/fleet-tasks`. Mock `/api/tasks`와 분리. 화면의 Mock 목록은 비어 있음 | PASS |
| 수신기 | 실물은 SSH의 `pinky_task_receiver.py`. 가상은 `fms-virtual-receiver` | PASS |
| 계획 속도 | 0.15 m/s. 계획용 값이며 실측 속도가 아님 | PASS |

`physical_control=blocked`와 `motion_backend=mock-only`는 수신 ACK와 경로 계획이 주행 실행과 분리되어 있다는 현재 설정이다.

## 2. 현재 로봇 등록 정보

`config/physical_fleet.json` 현재 값이다. 이번 검증에서 수정하지 않았다.

| 로봇 | 종류 | 등록 위치 | SSH | Nav2 |
| --- | --- | --- | --- | --- |
| pinky-1 | physical | 등록 home 없음. 위치는 TF | 대상 `192.168.0.83`, 연결됨 | READY |
| pinky-2 | virtual | (0.425, 1.925), yaw 0 | 미사용 | 해당 없음 |
| pinky-3 | virtual | (0.425, 0.575), yaw 0 | 미사용 | 해당 없음 |

화면 표기: Pinky #1 실물, pose 약 1.69, 0.12 m. Pinky #2 등록 위치 0.42, 1.93 m, grid 43, 193. Pinky #3 등록 위치 0.42, 0.57 m, grid 43, 57. 상단 집계는 등록 3대, 실물 1, SSH 1, 가상 2.

### 등록 좌표가 바뀐 경로

이전 서버 access log에 다음 요청이 있다.

- `POST /api/fleet-tasks/robots/pinky-2/home` → 200
- `POST /api/fleet-tasks/robots/pinky-3/home` → 200

처리 경로는 관제 화면의 `제안 위치 승인`이다. 확인 대화 뒤 `{x, y, yaw: 0, approve: true}`를 보내고, `FleetTaskService.approve_home`이 이동 가능 여부를 확인한 다음 `update_virtual_home`이 `config/physical_fleet.json`에 쓴다. 저장된 좌표는 당시 clearance 실패 지점의 첫 제안과 같다.

- (0.40, 1.90)의 첫 제안 → pinky-2 (0.425, 1.925)
- (0.40, 0.50)의 첫 제안 → pinky-3 (0.425, 0.575)

이번 검증은 그 POST를 다시 보내지 않았고, 파일을 이전 값으로 되돌리지 않았다.

### Clearance 재확인

로봇 반경 설정 0.06 m 기준이다. 이 반경은 검증된 안전 반경이 아니다.

| 좌표 | 결과 | 최근접 점유 | 판정 |
| --- | --- | --- | --- |
| pinky-2 (0.425, 1.925) | free, traversable, reason `ok` | 0.08 m | PASS |
| pinky-3 (0.425, 0.575) | free, traversable, reason `ok` | 0.10 m | PASS |
| 이전 (0.40, 1.90) | free, reason `clearance` | 0.055227 m | PASS |
| 이전 (0.40, 0.50) | free, reason `clearance` | 0.025495 m | PASS |

이전 좌표의 제안은 `suggestion_only=true`, `replaces_registered_pose=false`다. 제안만으로 등록 파일은 바뀌지 않는다.

## 3. 실제 지도 및 좌표계

`config/maps/physical_site/1f_map_f.yaml`

- image `1f_map_f.pgm`
- mode trinary, resolution 0.01 m, origin (0, 0, 0)
- negate 0, occupied 0.65, free 0.196
- 186 × 236 px, 실제 크기 1.86 × 2.36 m
- frame `map`
- grid row 0은 이미지 아래쪽. 계획 stride 0.05 m

화면 지도 설명은 `1f_map_f.yaml · 1.86 × 2.36 m · 0.01 m/px · frame: map`이다. 지도 클릭은 `pixelToWorld`로 map 좌표를 만들고, 경로 선은 backend waypoint를 `worldToPixel`로 그린다.

시설 `config/facilities.json`의 화장실, 식당, 대기 구역은 x, y가 null이다. 화면 옵션은 `좌표 미등록`이고 disabled다. 좌표를 새로 넣지 않았다.

## 4. Task 생성부터 ACK까지의 흐름

기존 Task 10건은 유지했다. 아래는 이번 검증에서 추가한 계획이다. 가상 로봇의 등록 위치는 바꾸지 않았고, 경과한 예약 시간을 실제 이동으로 처리하지 않았다.

| Task | 로봇 | 상태 | 내용 | 판정 |
| --- | --- | --- | --- | --- |
| `7f947d36-...` | pinky-1 | RESERVED, NOT_SENT | 자동 배정. 출발 (0.9, 1.2)는 요청 좌표. 후보 3개, 4 waypoint, 출발 지연 1.333 s | PASS. 전송은 NOT RUN |
| `f6f3ab27-...` | pinky-2 | DELIVERED | 수동. 출발 (0.625, 0.075), 목표 (0.625, 0.475). 후보 3개, 9 waypoint | 가상 수신 PASS. 실물 이동 NOT RUN |
| `ddb32a56-...` | pinky-3 | RESERVED, NOT_SENT | 수동. 출발 (0.625, 1.025), 목표 (0.625, 1.575). path-1 0.55 m, 12 waypoint | PASS |
| `29103196-...` | pinky-3 | CANCELLED | 위 pinky-2 경로와 같은 기하. 출발을 1.333 s 늦춘 뒤 예약했고, 취소로 그 예약만 제거 | PASS |
| `3ae46a74-...` | 미배정 | WAITING | 등록 위치에서 (1.2, 0.8)로 가는 수동 계획. 기존 예약과 겹쳐 지연 한도 안에서 예약 불가 | PASS |
| `15f55e0b-...` | 미배정 | CANCELLED | 같은 이유로 대기였던 pinky-3 계획. 예약은 생기지 않았고 취소됨 | PASS |
| `38b53bc5-...` | 미배정 | FAILED | 목표 (0, 0) `goal occupied` | PASS |

자동 배정 근거는 화면에 그대로 표시된다.

`score 0.451 = (travel 1.28s + delay 1.33s + queue 2) / priority 8`

계획 충돌 회피 문구는 `대기 후 출발 · 실측 안전 검증 아님`이다.

전달 결과:

- pinky-2 `f6f3ab27`: `ack_status=SIMULATED_RECEIPT`, channel `simulated`, receiver `fms-virtual-receiver`, `execute=false`, `actual_motion_verified=false`. 화면은 `ACK 가상 수신`.
- FAILED와 WAITING에 대한 dispatch는 HTTP 409. 화면의 `Task 전달`은 FAILED에서 disabled.
- 이미 ACKNOWLEDGED인 실물 Task의 `Task 전달`과 `취소`는 disabled.
- pinky-1 자동 계획 `7f947d36`는 RESERVED라 전달 버튼이 활성이다. 누르지 않았다. 운영자 승인 없이 전송하면 안 된다.

관제 화면에서 확인한 조작:

1. 로봇 3대와 실물/가상 구분.
2. Pinky-1 TF 화살표, Pinky-2·3 등록 위치 마커.
3. 지도 클릭이 X=1.020 m, Y=1.416 m, grid col 102.0, row 141.6을 입력.
4. 시설 3종이 `좌표 미등록` disabled.
5. 수동/자동 전환. 자동이면 대상 로봇 선택이 disabled.
6. 대기열에 배정, 상태, 전달, ACK 문구 표시.
7. 선택 Task의 Yen 후보와 선택 경로. 예: path-1 0.55 m, path-2/3 0.591421 m.
8. `경로 계산 및 예약`은 전송하지 않는다고 화면에 적혀 있다. 로봇을 고르기 전에는 disabled.

## 5. 충돌 회피 테스트 결과

| 항목 | 결과 | 판정 |
| --- | --- | --- |
| 서로 다른 로봇 배정 | pinky-1 자동, pinky-2 수동, pinky-3 수동 | PASS |
| 시간 충돌 시 출발 조정 | 같은 기하의 후속 Task가 depart 1.333 s, `conflicts_avoided_by_delay=true` | PASS |
| 지연 한도를 넘는 기존 예약 | 등록 위치 출발 계획은 WAITING, 예약 0건. dispatch 409 | PASS |
| 예약 중복 | 확정 예약 사이 충돌 보고 0건 | PASS |
| 취소 시 예약 정리 | `29103196` 예약 17건이 0건이 됨. 다른 Task 예약 23건은 유지 | PASS |
| 계획 실패 시 전송 방지 | `goal occupied`는 FAILED, dispatch 409, 화면 전달 버튼 disabled | PASS |
| 자동 배정 근거 | score 문자열이 Task에 저장되고 화면에 표시 | PASS |
| 계획 점유의 의미 | `occupancy_class=planned`, `actual_motion_verified=false`, `collision_safety_verified=false` | PASS |
| 가상 로봇 실제 이동 | 등록 좌표는 그대로. 이번 출발점은 `start_source=request` | NOT RUN |
| 실물 충돌 안전 | 계획 검사만 수행. 실물로 검증하지 않음 | NOT RUN |

출발 지연 한도는 8 slot이다. slot은 0.05/0.15 s라 최대 약 2.67 s다. 기존 예약이 그 시간보다 길게 같은 칸을 차지하면 경로는 예약되지 않고 WAITING이 된다. 이것은 예약 실패를 통과로 처리하지 않는 동작이다.

## 6. 경로 시각화 검증 결과

판정: PASS.

pinky-3 예약 Task를 선택했을 때 SVG polyline 3개와 waypoint circle이 그려졌다. 첫 점 `62.5,133.5`는 backend 출발 (0.625, 1.025)를 지도 프레임으로 바꾼 값과 같다.

- x pixel = 0.625 / 0.01 = 62.5
- y pixel = 236 - 1.025 / 0.01 = 133.5

후보 경로는 점선, 선택 경로는 실선이다. 색은 로봇 id에 연결된다. Frontend가 별도 경로를 계산하지 않는다.

지도에는 Pinky-1 화살표, Pinky-2 주황 마커, Pinky-3 보라 마커, 클릭 목적지 십자가 함께 보였다. 경로가 보인다는 것은 계획 좌표가 보인다는 뜻이며, 로봇이 그 경로를 주행했다는 뜻이 아니다.

## 7. 실제 Pinky ACK 검증 근거

판정: PASS. 읽기 전용 재확인이다. 새 실물 Task는 보내지 않았다.

Task `5b3bc030-0b45-4688-bd97-7618c1102fde`

| 항목 | FMS | Pinky `~/fms_task_receiver/inbox/<task_id>.json` |
| --- | --- | --- |
| Task ID | 동일 | 동일 |
| Robot ID | pinky-1 | payload robot_id pinky-1 |
| Payload hash | FMS ACK와 동일 | 저장 hash와 동일 |
| Received At | 동일 | 동일 |
| ACK status | ACKNOWLEDGED | receiver `pinky-fms-receiver` |
| execute | false | false |
| channel | physical | 수신 파일 존재 |

화면 대기열은 `전달 DELIVERED · ACK ACKNOWLEDGED`, 선택 경로는 path-1 0.886396 m, 15 waypoint다. 전달 버튼은 비활성이다. `actual_motion_verified`는 false다. ACK는 수신 확인이지 주행 완료가 아니다.

## 8. Frontend 및 Backend 테스트 결과

| 항목 | 결과 | 판정 |
| --- | --- | --- |
| Backend pytest | 104건, exit code 0 | PASS |
| Frontend 격리 빌드 | `FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack` 성공. `/`, `/api/physical-5cm` | PASS |
| 관제 화면 조작 | 3대, 위치, 지도 클릭, 시설 disabled, 수동/자동, 후보 경로, 대기열, ACK 문구 | PASS |

승인된 등록 좌표가 (0.425, 1.925), (0.425, 0.575)가 된 뒤, 테스트를 다시 실행하면 5건이 이전 좌표 (0.4, 1.9), (0.4, 0.5)를 기대하고 실패했다. 테스트를 임시 registry로 분리해 이전 좌표의 clearance 거부와 `approve=false`일 때 파일이 변하지 않는 조건은 유지하고, 실제 `config/physical_fleet.json`은 읽기 비교만 하도록 고쳤다. 수정 후 104건이 통과했다.

## 9. 기존 기능 회귀 여부

| 항목 | 결과 | 판정 |
| --- | --- | --- |
| SSH | 연결됨. 마지막 확인 2026-10-28 19:24 KST | PASS |
| Nav2 | `nav2_ready=true`, state `ready` | PASS |
| AMCL | active | PASS |
| TF | ready. pose (1.691, 0.124), source `tf:map->base_footprint` | PASS |
| 지도 | `1f_map_f` 표시 | PASS |
| 5cm UI | 버튼이 따로 있고, 이번 검증에서 누르지 않음. 표시 상태는 failed, Nav2 result status 6, 관측 전진 0.0 cm | 조회 PASS. 전진 NOT RUN |
| 5cm GET | HTTP 200. state `failed`, finished_at `2026-10-28T10:11:13.536305+00:00` | PASS |
| 5cm POST | 이번 검증에서 호출하지 않음 | NOT RUN |
| 재시작 복구 | pytest의 재시작 케이스는 104건에 포함되어 통과. 운영 API 프로세스는 이번 검증에서 재시작하지 않음 | 단위 PASS. 운영 재시작 NOT RUN |
| Frontend 빌드 | 격리 dist로 통과. dev 서버의 `.next`는 덮지 않음 | PASS |
| 기존 Task 유지 | 검증 전 10건의 id가 검증 후 목록에 남아 있음 | PASS |

5cm 상태가 재시작 뒤 idle로 보이는 이유: `PinkyFiveCmTest.current`는 프로세스 메모리이고 초기값이 idle이다. 디스크에 실패 이력을 저장하지 않는다. 따라서 재시작 후 idle은 저장된 실패 기록을 지운 것이 아니다. 현재 프로세스의 failed는 이번 API 로그의 `POST /api/physical/tests/pinky-1/forward-5cm` 202 이후 메모리에 남은 Nav2 status 6 결과다. GET이 500인 상태는 아니다.

로봇 카드 문구는 한 갱신 주기에서 `pose 미수신`으로 바뀌었고, 같은 화면의 TF 상세와 지도 화살표는 1.691, 0.124를 유지했다. 카드 문구는 fleet 로봇 pose를 보고, 지도 마커는 5초 이내의 live TF를 본다. 기능 실패로 보지는 않았으며 미해결 이슈에 적는다.

## 10. 수정 파일 목록

이번 인수 검증에서 바꾼 파일:

- `backend/tests/test_fleet_tasks.py`  
  잘못된 등록 좌표 검사는 임시 파일에서 하고, 승인 거부가 실제 registry를 쓰지 않는지 byte 비교로 확인.
- `backend/tests/test_route_planning.py`  
  현재 등록 좌표는 통과로 확인하고, (0.4, 1.9)와 (0.4, 0.5)는 직접 검사해 clearance 실패를 유지. preview의 clearance 거부 요청에 명시 출발점을 넣음.
- `docs/FMS_Final_Acceptance_Report.md`  
  이 보고서.

`frontend/tsconfig.json`은 격리 빌드가 `.next-verify/types` include를 넣었고, 검증 후 `.next/types`만 남기도록 되돌렸다. `config/physical_fleet.json`은 변경하지 않았다.

Git reset, 기존 데이터 삭제, 원격 push, 배포는 하지 않았다.

## 11. 미해결 이슈

- 실물 주행은 검증하지 않았다. ACK와 가상 수신을 이동 완료로 보면 안 된다.
- pinky-1 계획 `7f947d36`는 RESERVED이고 전달 버튼이 활성이다. 별도 승인 없이 전송하지 않아야 한다.
- 5cm 메모리 상태가 failed다. 이번 검증의 POST는 아니고, 재시작해도 과거 실패 파일이 지워지는 구조가 아니다. 실패 이력 자체가 저장되지 않는다.
- 출발 지연 한도 약 2.67 s를 넘는 기존 예약이 있으면 새 계획은 WAITING이 된다. 경로를 더 길게 우회하는 단계는 없다.
- `collision_safety_verified`는 false다. 계획 속도 0.15 m/s는 실측 속도가 아니다.
- 화장실, 식당, 대기 구역 좌표는 없다.
- health `phase` 필드가 2다. Fleet Task 구현 단계와 응답 라벨이 다르다.
- 로봇 카드의 pose 문구와 지도 TF 마커가 서로 다른 출처를 본다.
- 운영 프로세스에 대한 재시작 복구와, 실제 SSH 지연 주입은 이번 검증에서 실행하지 않았다. 단위 테스트만 통과했다.
- `simulation/ros_ws`의 끊긴 launch symlink는 그대로다. ROS 파일을 만들지 않았다.

## 12. 향후 실제 Nav2 연동에 필요한 작업

1. 주행 착수 조건을 수신 ACK와 분리한다. `execute=false` 수신이 성공한 뒤에만, 별도 승인으로 Nav2 Goal을 보낸다.
2. 계획 속도와 로봇 반경을 Pinky 실측값으로 바꾼다. 바꾸기 전에는 `collision_safety_verified`를 true로 두지 않는다.
3. 예약 시각이 지나도 로봇 pose가 예약 칸에 도착했다고 처리하지 않는다. 다음 Task 자동 진행도 pose 확인 뒤에 둔다.
4. 초기 pose와 TF 유효 시간을 카드 문구와 지도 마커가 같은 기준으로 보게 한다.
5. 시설 좌표는 실측 후에만 등록한다.
6. 5cm 결과가 필요하면 메모리 밖의 이력으로 남긴다. 재시작 직후 idle을 이전 성공으로 해석하지 않게 한다.
7. health의 phase 라벨을 현재 구현과 맞춘다.

## 인수 결론

관제 화면에서 세 로봇의 위치를 보고, 목적지를 지정하고, 수동·자동으로 배정하고, backend waypoint로 경로와 시간 예약을 확인한 뒤, 실패 계획은 전송되지 않고, 가상 수신과 기존 Pinky-1 수신 ACK가 구분되어 표시되는 것까지 확인했다.

실물 이동, 새 Pinky-1 Task 전송, 5cm 전진 POST는 NOT RUN이다.
