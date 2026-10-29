# Care-Move FMS

Open-RMF를 쓰지 않는 독립 Fleet Management System입니다. FastAPI가 Task, 경로, 예약을 관리하고, Next.js 관제 화면이 그 API를 표시합니다. Pinky-1 Task 전달은 SSH로 수신 전용 Receiver를 실행합니다. 5cm 전진은 그와 별도의 Nav2 `DriveOnHeading` 시험입니다.

## 구성

```text
FMS/
├── backend/          FastAPI, Task, 경로, 예약
├── frontend/         Next.js 관제 화면
├── config/           1f_map_f, Nav2 파라미터, 로봇 Registry
├── launch/           Pinky-1 Nav2 launch
├── robot_receiver/   Pinky 수신 전용 스크립트
├── robot_agent/      모의 실행 에이전트
├── scripts/          API와 대시보드 실행
├── simulation/       Gazebo용 워크스페이스. 제조사 소스는 submodule
├── docs/             운영, 통신, 진단 문서
└── data/             로컬 SQLite. Git에 태스크 데이터를 넣지 않습니다
```

## 실행

로컬 비밀값은 `.env`에만 둡니다. `.env.example`의 `FMS_ROS_DOMAIN_ID` 기본값은 73입니다. 이 값 하나로 Pinky 세 대를 연결하지 않습니다. 로봇별 도메인은 `config/physical_fleet.json`에 있습니다.

| Robot | ROS Domain | 현재 |
|---|---:|---|
| pinky-1 | 24 | 실물 |
| pinky-2 | 25 | 가상. 실물 연결은 아직 없음 |
| pinky-3 | 26 | 가상. 실물 연결은 아직 없음 |

```bash
python3 -m venv .venv
.venv/bin/pip install -e 'backend[dev]'
./scripts/run_api.sh
```

`./scripts/run_api.sh`는 FastAPI만 띄웁니다. `FMS_DOMAIN_WORKERS_ENABLED` 기본값은 true라서 API가 로봇마다 Worker 프로세스를 만듭니다. Worker의 `ROS_DOMAIN_ID`는 Registry 값이고, API 프로세스의 환경 변수는 바꾸지 않습니다. 같은 로봇 Worker를 수동으로 한 번 더 띄우지 않습니다.

```bash
./scripts/run_domain_worker.sh pinky-1
```

위 수동 명령은 API supervisor가 꺼져 있을 때만 씁니다. `FMS_DOMAIN_WORKERS_ENABLED=false`이면 supervisor가 뜨지 않습니다.

다른 터미널:

```bash
./scripts/run_dashboard.sh
```

- Backend: `http://127.0.0.1:8741`
- Frontend: `http://127.0.0.1:8742`
- 프록시: `/fms-api/*` → Backend `/api/*`
- Pinky-1 Worker: ROS Domain 24
- Pinky-2 Worker: ROS Domain 25. 로봇이 없으면 미연결
- Pinky-3 Worker: ROS Domain 26. 로봇이 없으면 미연결

`scripts/run_api.sh`는 API를 `0.0.0.0:8741`에 바인딩합니다. 같은 네트워크의 다른 호스트가 접근할 수 있으므로 개발 PC 밖으로 포트를 열지 않습니다. Fleet Task API에는 별도의 로그인 토큰이 없습니다.

## 문서

- [운영 설명서](docs/FMS_Operation_Guide.md)
- [통신 및 API 명세](docs/FMS_Communication_API_Specification.md)
- [5cm 진단](docs/FMS_5cm_Motion_Diagnosis_Report.md)
- [저장소 정리](docs/FMS_Repository_Cleanup_Report.md)

Task 전달과 5cm 주행은 운영자가 명시적으로 시작할 때만 수행합니다. 수신 ACK는 로봇이 이동했다는 뜻이 아닙니다.

## 테스트

```bash
cd backend
env -u ROS_DISTRO -u ROS_DOMAIN_ID -u RMW_IMPLEMENTATION \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv/bin/pytest -q -p pytest
cd frontend && ./node_modules/.bin/tsc --noEmit
cd frontend && FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack
```

ROS가 source된 셸에서 `ROS_DISTRO`를 지우지 않으면 preflight 테스트는 준비되지 않은 개발 PC를 다른 이유로 실패 처리한다. `PYTEST_DISABLE_PLUGIN_AUTOLOAD`는 ROS `launch_testing` 플러그인이 이 가상환경의 pytest를 깨지 않게 한다.
