# FMS 운영 설명서

이 문서는 `/home/sw/DevSpace/FMS`의 현재 코드와 스크립트를 기준으로 한다. 비밀번호, lease secret, 5cm 토큰 값은 적지 않는다.

읽기 전용 명령과 상태를 바꾸는 명령을 구분한다. Pinky-1에 Task를 보내거나 5cm 주행을 시작하는 명령은 운영자가 승인한 뒤에만 실행한다. 이 문서의 예제 흐름에 그 호출을 넣지 않는다.

## 1. Git clone

```bash
git clone https://github.com/KimSunWooo/FMS.git
cd FMS
git submodule update --init --recursive
```

`simulation/ros_ws/src/pinky_pro`는 제조사 submodule이다. `simulation/ros_ws/build`, `install`, `log`는 colcon 산출물이며 저장소에 다시 넣지 않는다. 시뮬레이터 스크립트 `simulation/scripts/run_nav2_fms.sh`는 이 워크스페이스 install이 아니라 `/opt/ros/jazzy`와 `/home/ubuntu/pinky_pro/install`을 읽는다. 그 경로는 이 개발 PC의 Pinky 소스 위치와 다를 수 있다.

## 2. Python과 Node.js

Backend는 저장소 루트의 `.venv`를 쓴다.

```bash
python3 -m venv .venv
.venv/bin/pip install -e 'backend[dev]'
```

Frontend는 Next.js 15.5.28, React 19다. `package.json`에 engines 필드는 없다. 타입 정의는 Node 20이다.

```bash
cd frontend && npm install
```

## 3. `.env`

`.env.example`을 복사한 뒤 로컬 값만 채운다.

```bash
cp .env.example .env
```

`.env.example`과 현재 Pinky 개발 환경의 차이는 다음과 같다.

| 항목 | `.env.example` | 현재 Pinky 개발 환경 |
|---|---|---|
| `FMS_ROS_DOMAIN_ID` | 73 | 24 |
| `FMS_PHYSICAL_SSH_TARGETS` | 없음 | `pinky-1=pinky@192.168.0.83` 형식 |
| `FMS_PHYSICAL_SSH_PASSWORD` | 없음 | 설정됨. 값은 문서에 없음 |
| `FMS_PHYSICAL_MAP_YAML` | 없음. 코드 기본값은 `config/maps/physical_site/1f_map_f.yaml` | 같은 지도 |
| `FMS_PHYSICAL_NAV2_AUTO_START` | 없음. 코드 기본값 false | 개발 환경에서 사용 |
| `FMS_PHYSICAL_5CM_TEST_ENABLED` | 없음. 코드 기본값 false | 시험 화면을 쓰려면 true |
| `FMS_PHYSICAL_5CM_TEST_TOKEN` | 없음 | 설정됨. 값은 문서에 없음 |

`scripts/run_api.sh`는 `.env`가 없으면 도메인 73과 새 lease secret으로 파일을 만든다. 이미 있는 `.env`를 덮어쓰지 않는다. 현재 `.env`를 지우지 않는다.

`FMS_PHYSICAL_CONTROL_ENABLED=true`, `FMS_ROS_BRIDGE_ENABLED=true`, `FMS_EXECUTION_MODE=physical`이면 서버가 기동을 거부한다. 이 거부와 별도로, 현재 코드는 ROS 상태를 읽고, 수신 전용 Task를 SSH로 보내며, 운영자가 승인한 5cm Goal만 Nav2에 보낸다. Task 수신 payload의 `execute`는 false다.

## 4. ROS 2

세 로봇은 서로 다른 DDS 도메인을 쓴다. `export ROS_DOMAIN_ID=24` 뒤에 `./scripts/run_api.sh`를 실행하는 것만으로 Pinky-2와 Pinky-3가 연결되지는 않는다.

| Robot ID | Domain | 설정 |
|---|---:|---|
| pinky-1 | 24 | `kind=physical`, `physical_connection_enabled=true` |
| pinky-2 | 25 | `kind=virtual`, `physical_connection_enabled=false`, `planned_physical=true` |
| pinky-3 | 26 | `kind=virtual`, `physical_connection_enabled=false`, `planned_physical=true` |

도메인은 `config/physical_fleet.json`의 `ros_domain_id`다. API는 `DomainSupervisor`가 Worker 프로세스를 만들고, 자식 환경에만 `ROS_DOMAIN_ID`를 넣는다. 중앙 프로세스는 그 값을 자신의 환경 변수로 덮어쓰지 않는다. Domain 24의 `/tf`를 Pinky-2 위치로 쓰지 않는다.

Pinky-1을 직접 볼 때:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Pinky-2·3는 아직 실물이 아니다. Domain 25 또는 26에서 노드가 없어도 Pinky-1 관제는 계속된다. 실물로 바꿀 때는 Registry의 `kind`를 `physical`로 바꾸고, 운영자가 SSH 대상을 넣은 뒤에만 `physical_connection_enabled`를 true로 둔다. IP와 계정은 이 문서에 만들지 않는다. 가상 등록 좌표는 TF 위치가 아니다.

읽기 전용 확인:

```bash
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_smoothed -v
ros2 topic info /cmd_vel -v
ros2 lifecycle get /collision_monitor
ros2 action list -t
```

FMS가 Nav2를 띄우는 조건은 `FMS_PHYSICAL_NAV2_AUTO_START`와 `physical_nav2_auto.py`다. `/odom`과 `/scan` publisher가 있어야 하고, 모터 bringup을 SSH 성공만으로 원격 시작하지 않는다. map_server에는 `FMS_PHYSICAL_MAP_YAML` 또는 기본값 `1f_map_f.yaml`을 넘긴다. AMCL에는 `set_initial_pose:=false`를 넘긴다. 이미 Nav2 노드가 있으면 launch를 다시 실행하지 않는다.

2026-10-28에 이미 떠 있던 map_server는 삭제된 `1f_map.yaml` 경로를 인자로 가지고 있었다. 코드가 다음에 새로 띄우는 지도는 `1f_map_f.yaml`이다. 실행 중인 프로세스를 이 문서의 확인 명령으로 죽이지 않는다. 운영자가 Nav2를 재시작할 때 인자 경로를 확인한다.

저장 `nav2_params.yaml`의 AMCL 초기 위치 x=0.11, y=2.527, yaw=-3.052는 `set_initial_pose: true`로 적혀 있다. 실행 중인 프로세스가 `-p set_initial_pose:=false`로 시작됐다면 그 YAML 좌표는 적용되지 않는다.

## 5. Pinky SSH

대상은 `FMS_PHYSICAL_SSH_TARGETS`다. 현재 개발 대상 형식은 `pinky-1=pinky@192.168.0.83`이다. 인증은 Paramiko 비밀번호이고 `look_for_keys=false`, `allow_agent=false`, 시스템 host key만 신뢰한다. 비밀번호는 `.env`의 `FMS_PHYSICAL_SSH_PASSWORD`다.

읽기 전용 접속 확인은 운영자 터미널에서 한다. 비밀번호를 명령줄에 붙이지 않는다.

```bash
ssh -o BatchMode=yes pinky@192.168.0.83 true
```

BatchMode는 키 로그인을 시험한다. FMS Dispatcher는 키 대신 비밀번호를 쓰므로, 이 명령이 실패해도 Dispatcher 설정이 틀렸다는 뜻은 아니다. host key가 로컬 `known_hosts`에 있어야 Paramiko `RejectPolicy`가 통과한다.

Receiver는 전송 때 SFTP로 `~/fms_task_receiver/pinky_task_receiver.py`에 올라간다. 수신 파일은 `~/fms_task_receiver/inbox/<task_id>.json`이다. 이 경로는 Task를 보낼 때만 생긴다.

## 6. Backend 실행

```bash
./scripts/run_api.sh
```

스크립트는 `.venv`가 없으면 만들고, `uvicorn app.main:app --app-dir backend --host 0.0.0.0 --port 8741`로 실행한다. 바인딩이 `0.0.0.0`이므로 방화벽이 8741을 열면 다른 호스트가 Fleet Task API를 호출할 수 있다. 이 API에는 로그인 토큰이 없다. 개발 PC에서는 localhost만 쓰고 포트를 외부에 노출하지 않는다.

데이터베이스 경로는 `FMS_DATABASE_URL`이다. 스크립트가 만드는 기본값은 `sqlite:///<저장소>/data/fms.sqlite3`다.

읽기 전용 상태:

```bash
curl -sS http://127.0.0.1:8741/api/system/health
curl -sS http://127.0.0.1:8741/api/system/readiness
```

종료는 그 uvicorn 프로세스에 SIGTERM이다. 재시작은 같은 `./scripts/run_api.sh`다. 애플리케이션 로그는 프로세스 stdout과 `logs/`다. Nav2 자동 기동 로그는 `logs/physical-nav2-*.log`를 본다.

## 7. Frontend 실행

다른 터미널에서:

```bash
./scripts/run_dashboard.sh
```

`FMS_API_ORIGIN` 기본값은 `http://127.0.0.1:8741`이다. `npm run dev`는 `next dev --turbopack -H 127.0.0.1 -p 8742`다. 브라우저는 `http://127.0.0.1:8742`이고 `/fms-api/*`는 Backend `/api/*`로 rewrite된다.

개발 서버가 `.next`를 쓰는 동안 프로덕션 확인은 다른 디렉터리를 쓴다.

```bash
cd frontend
FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack
```

## 8. 로봇, 지도, 위치

읽기 전용:

```bash
curl -sS http://127.0.0.1:8741/api/physical/robots
curl -sS http://127.0.0.1:8741/api/physical/nav2/status
curl -sS http://127.0.0.1:8741/api/physical/nav2/pose
curl -sS http://127.0.0.1:8741/api/physical/map
curl -sS http://127.0.0.1:8741/api/fleet-tasks/availability
```

Registry는 `config/physical_fleet.json`이다. pinky-1은 physical, pinky-2와 pinky-3은 virtual이다. 위치는 `map`에서 `base_footprint` TF다. `/amcl_pose`를 FMS가 구독하지는 않는다.

경로만 계산하고 Task를 만들지 않는 요청:

```bash
curl -sS -X POST http://127.0.0.1:8741/api/planning/routes/preview \
  -H 'content-type: application/json' \
  -d '{"robot_id":"pinky-2","goal":{"x":1.0,"y":1.0}}'
```

좌표는 지도 미터다. 점유 셀이면 422와 빈 경로가 반환될 수 있다. 이 호출은 Nav2 Goal을 보내지 않는다. 좌표 변환은 `POST /api/physical/map/coordinates`에 `{"map":{"x":1.0,"y":1.0,"yaw":0}}` 또는 `{"grid":{"col":10,"row":20}}`를 보낸다.

## 9. Task

관제 화면에서 로봇, 목적지, 할당 방식을 고른 뒤 생성, 계획, 선택, 전달 순서로 진행한다. 전달 버튼은 Pinky-1에 SSH 수신을 수행하므로 운영자 승인 단계다. 아래 생성 예시는 가상 로봇용 형태만 보여 준다. 실제 `robot_id: "pinky-1"` 생성 후 dispatch는 여기서 실행하지 않는다.

생성 body:

```json
{
  "assignment_mode": "manual",
  "robot_id": "pinky-2",
  "goal": {"x": 1.0, "y": 1.0},
  "label": "대기",
  "task_type": "navigate",
  "priority": 5
}
```

`assignment_mode`가 automatic이면 `robot_id`를 비울 수 있다. 이어서 화면 또는 API에서 plan, select, dispatch를 호출한다. dispatch 전 예약은 `GET /api/planning/reservations`로 확인한다. 가상 로봇 ACK의 receiver는 `fms-virtual-receiver`이고 channel은 `simulated`다. Pinky-1 ACK의 receiver는 `pinky-fms-receiver`이고 channel은 `physical`이다. 둘 다 `execute: false`이며 로봇이 이동했다는 뜻이 아니다.

취소 body는 `{"reason":"operator"}`다. 이미 수신된 Task는 `CANCEL_PENDING`으로 남고, Receiver cancel ACK 뒤에 `CANCELLED`가 된다. 보관과 복원은 `POST /api/fleet-tasks/{task_id}/archive`와 `restore`다. 삭제 API는 없다.

목록:

```bash
curl -sS 'http://127.0.0.1:8741/api/fleet-tasks/browse?scope=active&page=1&page_size=10'
curl -sS 'http://127.0.0.1:8741/api/fleet-tasks/browse?scope=history&page=1&page_size=10'
curl -sS 'http://127.0.0.1:8741/api/fleet-tasks/browse?scope=archived&page=1&page_size=10'
```

## 10. 5cm 시험

준비 조건은 현장 확인, 전방 공간, 비상 정지, Collision Monitor active, LiDAR와 TF, 다른 Nav2 Goal이 없는 상태다. 현재 진단은 StopZone 안에 유효 반환이 남아 Goal을 보내지 않는다. 자세한 수치는 `docs/FMS_5cm_Motion_Diagnosis_Report.md`에 있다.

읽기 전용 진단:

```bash
curl -sS http://127.0.0.1:8741/api/physical/tests/pinky-1/forward-5cm/diagnosis
curl -sS http://127.0.0.1:8741/api/physical/tests/pinky-1/forward-5cm
```

시작은 관제 화면의 5cm 버튼뿐이다. 브라우저는 `POST /api/physical-5cm`에 `{"action":"start"}`와 헤더 `x-fms-ui-intent: physical-5cm`를 보낸다. Next 서버가 `.env`의 토큰을 `X-FMS-Operator-Token`으로 Backend에 붙인다. 토큰은 브라우저에 내려가지 않는다. Backend는 클라이언트가 `127.0.0.1` 또는 `::1`이고 토큰이 일치할 때만 202를 반환한다. 진단이 StopZone 정지를 보고하면 409이고 Goal은 나가지 않는다.

## 11. 종료

Frontend는 `run_dashboard.sh` 프로세스에 SIGINT다. Backend는 uvicorn에 SIGTERM이다. SQLite 파일과 Pinky inbox는 종료만으로 지워지지 않는다.
