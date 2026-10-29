# FMS 다중 로봇 ROS Domain 구조

2026-10-28. 공식 지도는 `1f_map_f`이고, Pinky 세 대는 Registry의 Domain 24·25·26으로 나뉜다. 이번 작업에서 실물 주행 POST는 호출하지 않았다. Map Server 프로세스도 종료하지 않았다.

## 1. 변경 전 구성

FastAPI 하나가 `FMS_ROS_DOMAIN_ID`로 ROS 명령을 실행했다. Pinky-1 Nav2 모니터와 5cm 시험이 그 값을 썼다. Pinky-2·3는 `config/physical_fleet.json`의 가상 로봇이었고 Domain이 없었다. Task, 경로, 예약, SSH 수신 ACK, 취소 ACK는 로봇 ID로 동작했다.

실행 중인 Map Server의 프로세스 인자는 삭제된 `1f_map.yaml`을 가리킨다. 직전 복구에서 `LoadMap`과 `yaml_filename` 파라미터는 `1f_map_f.yaml`로 맞춰 두었다.

## 2. 변경 후 구조

```text
FastAPI :8741  /  Next.js :8742
        |
        +-- Domain supervisor (API 환경 변수는 바꾸지 않음)
              |-- pinky-1 worker   ROS_DOMAIN_ID=24
              |-- pinky-2 worker   ROS_DOMAIN_ID=25
              +-- pinky-3 worker   ROS_DOMAIN_ID=26
```

중앙 Task Manager와 Reservation Manager는 그대로 하나다. Domain이 달라도 예약 충돌 검사를 생략하지 않는다. 좌표는 같은 `1f_map_f`의 map 미터다.

Worker는 로봇마다 별도 프로세스다. 상태 파일은 `data/domain_workers/{robot_id}.json`이고, 다른 로봇 파일을 위치로 쓰지 않는다. 12초보다 오래된 샘플은 pose를 지우고 `disconnected`로 본다. 가상 로봇 샘플에 pose가 있어도 버린다.

## 3. Domain 매핑

| Robot ID | Domain | kind | physical_connection_enabled | planned_physical | home |
|---|---:|---|---|---|---|
| pinky-1 | 24 | physical | true | false | 없음 |
| pinky-2 | 25 | virtual | false | true | (0.425, 1.925, yaw 0) |
| pinky-3 | 26 | virtual | false | true | (0.425, 0.575, yaw 0) |

Registry에 없는 Domain으로 알려진 로봇 ID를 바꾸면 로딩이 거부된다. 가상 로봇은 `physical_connection_enabled=true`를 둘 수 없다. 실물 전환은 `kind`를 바꾸는 운영자 수정이 필요하다. IP와 SSH 계정은 추가하지 않았다.

`FMS_ROS_DOMAIN_ID`가 73이어도 Pinky-1 Nav2 명령과 5cm 명령은 Registry의 24를 쓴다. 24가 아니면 5cm 시작은 409다.

## 4. 변경 파일

- `config/physical_fleet.json`
- `backend/app/core/config.py`
- `backend/app/main.py`
- `backend/app/api/physical.py`
- `backend/app/services/fleet_registry.py`
- `backend/app/services/domain_status.py`
- `backend/app/services/domain_worker.py`
- `backend/app/services/domain_supervisor.py`
- `backend/app/services/physical_nav2_auto.py`
- `backend/app/services/forward_5cm_service.py`
- `backend/app/services/fleet_task_service.py`
- `backend/app/services/nav2_chain_probe.py`
- `backend/tests/test_domain_workers.py`
- `backend/tests/conftest.py`
- `frontend/src/components/fleet-console.tsx`
- `scripts/run_domain_worker.sh`
- `README.md`
- `docs/FMS_Operation_Guide.md`
- `docs/FMS_Communication_API_Specification.md`

## 5. Worker 실행

API 기동 시 `FMS_DOMAIN_WORKERS_ENABLED` 기본값 true면 supervisor가 세 프로세스를 만든다.

```bash
./scripts/run_api.sh
```

이 명령은 API를 `:8741`에 띄운다. `export ROS_DOMAIN_ID=24`만으로는 Pinky-2·3가 연결되지 않는다.

supervisor가 꺼져 있을 때만 한 대를 수동 실행한다. supervisor와 동시에 같은 로봇을 띄우면 상태 파일을 두 프로세스가 쓴다.

```bash
./scripts/run_domain_worker.sh pinky-1
./scripts/run_domain_worker.sh pinky-2
./scripts/run_domain_worker.sh pinky-3
```

조회는 `GET /api/physical/domain-workers`다. 프로세스가 죽으면 supervisor가 약 2초 주기로 다시 띄운다. 확인에서 Pinky-2 Worker만 종료했고, 새 PID가 Domain 25의 `physical_connection_not_configured`로 돌아왔다. Pinky-1 Worker PID는 유지됐다. pose는 null이었다.

가상 Worker는 ROS 노드를 만들지 않는다. 실물 연결이 켜진 Worker만 `ros2 node list`, `bt_navigator` lifecycle, `map` → `base_footprint` TF를 읽는다. `cmd_vel`과 Nav2 Goal은 보내지 않는다.

## 6. Registry

홈 좌표는 바꾸지 않았다. Task 응답의 `assigned_ros_domain_id`는 로봇 ID로 Registry를 찾은 값이다. payload hash에는 넣지 않는다. Domain 번호로 Task 소유자를 고르지 않는다.

Pinky-2·3의 Task 전달은 계속 가상 receiver다. 실물 ACK를 만들지 않는다.

## 7. Pinky-1 연결 검증

| 항목 | 결과 |
|---|---|
| Worker | 실행 중. `observed_domain_id=24`. 노드 20개. `bt_navigator` active |
| SSH `192.168.0.83:22` | 실패. `No route to host`. ping 응답 없음 |
| `map` → `base_footprint` | `lookup_failed`. `map` frame 없음 |
| Nav2 모니터 | `waiting_for_ssh`. SSH가 되기 전에는 지도 일치를 기록하지 않음 |
| 5cm 버튼 | 비활성. `live TF lookup failed`, `map identity not checked` |
| 주행 POST | 호출하지 않음. 상태 `idle` |

SSH와 ROS는 다른 경로다. Domain 24에서 Nav2 노드는 이 PC에서 보이지만, 로봇 호스트로 TCP 22가 열리지 않는다. TF가 없으므로 Nav2 READY로 올리지 않았다.

## 8. Pinky-2·3

Domain 25와 26에서 `ros2 node list`는 노드 없이 종료 코드 0이었다. 기록은 NOT CONNECTED다. Worker는 ROS에 붙지 않고 `physical_connection_not_configured`다. 화면의 0.42 m, 1.93 m와 0.42 m, 0.57 m는 등록 위치이며 TF가 아니다.

실물로 붙일 때 필요한 것:

- 운영자가 `kind`를 `physical`로 변경
- `physical_connection_enabled=true`
- `FMS_PHYSICAL_SSH_TARGETS`에 해당 로봇의 `user@host`
- 같은 `1f_map_f`, AMCL, `map` → `base_footprint` 확인
- 그 전에는 실물 주행 가능으로 바꾸지 않음

IP와 계정은 아직 없으므로 넣지 않았다.

## 9. 지도 일치

코드의 플래너, Map Frame, 설정 기본값은 `config/maps/physical_site/1f_map_f.yaml`과 `1f_map_f.pgm`이다. 실행 경로에서 필수 파일로 `1f_map.yaml`을 읽는 코드는 없다. 테스트 한 곳은 불일치 사례로 없는 `1f_map.yaml` 경로를 일부러 넣는다.

이번 확인:

| 항목 | 값 |
|---|---|
| `/map_server` `yaml_filename` | `.../1f_map_f.yaml` |
| `/map` | 186×236, resolution 0.01, origin 0, frame `map` |
| occupancy sha256 | `6009e9fc529e2807a64ce19be1b0bfafc5ac655f1819305dd9675b3c97028b57` |
| FMS grid sha256 | 같은 값 |

프로세스 인자 문자열은 시작 당시의 `1f_map.yaml`이 남아 있다. 파라미터와 `/map` 내용이 일치하므로 Map Server를 재시작하거나 `LoadMap`을 다시 호출하지 않았다. PID 590031은 유지했다.

관제 화면의 `map identity not checked`는 지도가 다르다는 뜻이 아니다. Nav2 모니터가 SSH 실패로 그 검사를 돌리지 않은 상태다.

## 10. Task Manager

생성, 수동·자동 배정, A*와 Yen, 예약, 전달, 수신 ACK, 취소 ACK, archive는 코드 경로를 유지했다. 재시작 전후 SQLite는 fleet task 22건, reservation 381건이다. 화면 진행 목록에 `e9afb44b-9c95-4fc3-bfc5-973b7f3797e9`가 예약 65건으로 남아 있다. 전달하거나 취소하지 않았다.

## 11. Frontend

로봇 카드에 ROS Domain, 가상/실물, SSH, ROS, TF, 지도 일치, Task 할당, 실물 주행 가능 여부를 표시한다. Pinky-2·3는 `Physical connection not configured`이고 등록 위치를 TF로 그리지 않는다. 5cm 버튼은 Pinky-1에만 있고 문구는 ROS Domain 24다. 이번 화면에서 그 버튼은 비활성이며 누르지 않았다.

## 12. 5cm와 StopZone

차단 로직은 유지했다. Collision Monitor를 끄거나 StopZone 반경을 줄이지 않았다. `cmd_vel`을 발행하지 않았다.

Domain 24 진단 GET 결과:

- `ros_domain_id=24`
- `motion_command_sent=false`
- scan 미수신, `transform_status=no_scan`
- `stop_zone_would_stop=false`는 구역이 비었다는 뜻이 아니다. scan이 없어 판정하지 못했다.
- 상세 문장은 이 경우를 실패로 적고, 맑은 구역으로 보지 않는다.

로봇에 경로가 없어 오른쪽 0.18 m 반환의 새 좌표는 이번 샘플에 없다. 직전 연결 상태에서 확인했던 오른쪽 군집은 해소된 것이 아니다. 자체 반사인지 외부 장애물인지는 여전히 확인되지 않았고, 필터를 넣지 않았다.

실물 5cm 이동은 NOT RUN이다.

## 13. 테스트

| 항목 | 결과 |
|---|---|
| Backend pytest | 131 passed, exit 0 |
| Frontend `tsc --noEmit` | exit 0 |
| 격리 Next.js 빌드 `.next-verify` | exit 0. 이후 `tsconfig.json`의 `.next-verify` include는 제거 |
| Domain 분리, 상태 파일 혼입, Worker 재시작, 5cm Domain 24 | `backend/tests/test_domain_workers.py` |
| 기존 Task·예약 회귀 | 같은 pytest. DB 건수 유지 |

ROS가 source된 셸에서는 기존과 같이 `ROS_DISTRO`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`을 지우고 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 pytest를 실행했다.

## 14. 미해결

- `192.168.0.83` SSH와 ping이 `No route to host`다. Domain 개편으로 주소를 바꾸지 않았다.
- `map` frame이 없어 Pinky-1 TF와 Nav2 READY를 갱신하지 못했다.
- 이번 구간의 StopZone 점유 좌표는 다시 측정하지 못했다. 주행은 BLOCKED / NOT RUN이다.
- Map Server 프로세스 인자에는 삭제된 `1f_map.yaml` 문자열이 남아 있다. 라이브 파라미터와 `/map` 해시는 `1f_map_f`와 같다.
- Git 이력의 이전 비밀값 제거와 커밋은 하지 않았다.

## 15. 이후 실물 Pinky-2·3에 넣을 설정

1. 로봇이 실제로 같은 `1f_map_f`를 쓰는지 확인한다.
2. Registry에서 해당 로봇의 `kind`를 `physical`로 바꾸고 home을 제거한다. 가상 상태에서는 실물 연결 플래그를 켤 수 없다.
3. `FMS_PHYSICAL_SSH_TARGETS`에 그 로봇 ID의 `user@host`를 넣는다. 비밀번호는 `.env`에만 둔다.
4. Domain은 pinky-2=25, pinky-3=26을 유지한다.
5. AMCL과 `map` → `base_footprint`가 살아 있고 `/map` sha256이 FMS와 같은 뒤에만 실물 주행을 허용한다.
6. 5cm 시험은 Pinky-1 Domain 24 전용이다. 다른 로봇으로 넓히지 않았다.
