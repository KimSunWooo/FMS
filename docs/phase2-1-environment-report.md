# Phase 2.1 환경 보고

조사 시각은 2026-10-26 13:47 KST(UTC+9) 이후다. 이 문서는 그 시점의 호스트와, 조사 뒤에 이 VM에서만 설치한 ROS 스택을 구분한다.

## 1. 작업공간

| 항목 | 값 |
| --- | --- |
| 절대 경로 | `/workspace` |
| 원격 | `origin` `https://origin.cursor.com/git/kssgss94/multi-robot-fms.git` |
| 조사 시작 브랜치 | `main` |
| 조사 시작 커밋 | `be5d319a3020fd5722517a02977aabad8752ddcf` |
| 조사 시작 작업 트리 | 깨끗함. 수정 파일 없음 |
| 작업 브랜치 | `cursor/phase2-1-gazebo-e2e-8a20` |

디렉터리:

- `backend/` FastAPI, SQLite, Task Manager, Scheduler, Ownership
- `frontend/` Next.js 관제 화면, 포트 8742
- `robot_agent/` Gazebo Nav2 에이전트
- `simulation/` 런처 없음. `PHASE = "not_run"`
- `docs/phase2-report.md`, `docs/phase2-gazebo-preflight.json` Phase 2 기록

기존 Open-RMF 트리 `/home/sw/DevSpace/care-move-rmf-starter` 는 이 머신에 없다. 그 저장소와 DB는 열지 않았다.

## 2. 조사 직후의 시뮬레이터

조사 직후, ROS를 설치하기 전:

| 점검 | 결과 |
| --- | --- |
| Ubuntu | 24.04.5 LTS, 커널 `6.12.94+`, x86_64 |
| CPU | 4 vCPU, Intel Xeon |
| RAM | 15 GiB. 프로세스 RSS 합 약 1.3 GiB. 하이퍼바이저 balloon 10 GiB. `free` available 약 4.5 GiB |
| GPU | 없음. `nvidia-smi` 없음 |
| 디스크 | `/` 254G 중 사용 약 7.5G, 여유 약 246G |
| `/opt/ros` | 없음 |
| `ros2`, `gz`, `gazebo`, `ign`, `colcon` | PATH에 없음 |
| ROS apt 소스 | 없음. `ubuntu.sources`, `google-chrome.sources`만 있음 |
| Nav2, Pinky 패키지 | 없음 |
| `/usr/bin/pinky` | coreutils finger. 로봇 패키지로 세지 않음 |
| Docker | 클라이언트 없음. `docker0` 인터페이스만 있고 트래픽 0 |
| `ROS_DISTRO`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` | 프로세스 환경에 없음 |
| UDP 소켓 | `/proc/net/udp`, `/proc/net/udp6` 모두 0건 |
| ROS / Gazebo 프로세스 | 없음 |
| NIC | `lo`, `enp0s3`, `docker0` |

공개 Pinky Pro 시뮬레이터는 저장소 밖에 있다. `pinklab-art/pinky_pro` 커밋 `014a09f289e6988894fbffde2624fdb1e257815b`. Ubuntu 24.04, ROS 2 Jazzy, 패키지 `pinky_gz_sim`, `pinky_navigation`, 월드 `pinky_factory.world`, 맵 `my_map.yaml`. 이 호스트에는 그 체크아웃이 없었다.

## 3. 도메인 격리

도메인 73이 비어 있다고 가정하지 않았다. 설치 전 UDP 소켓이 0건이고 ROS 프로세스가 없었다. 이 VM 안에서는 73을 쓰는 DDS 참가자가 없었다.

그 다음에도 현장망과 겹치는지 이 머신에서는 알 수 없다. 그래서 Gazebo와 에이전트는 다음으로만 붙였다.

- `ROS_DOMAIN_ID=73` (FMS 설정의 기대 도메인과 같음. 조사로 비어 있음을 확인한 뒤)
- `ROS_LOCALHOST_ONLY=1` (Jazzy에서 deprecated 경고가 났고, 그대로 적용됨)
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

API 프로세스는 ROS를 source 하지 않는다. 헬스의 preflight는 그 프로세스 기준으로 `not_ready`다. Nav2 goal은 에이전트 프로세스만 보낸다. `config/cyclonedds.fms.xml` 은 여전히 로드하지 않는다. 실물 bringup `pinky_bringup bringup_robot.launch.xml` 은 실행하지 않았다.

## 4. 조사 후 이 VM에 설치한 것

E2E에 필요해서 이 클라우드 VM에만 설치했다. FMS git 트리에 ROS 패키지를 넣지 않았다. 셸 시작 파일은 수정하지 않았다.

- ROS 2 Jazzy (`ros-jazzy-ros-base`, Nav2 bringup 1.3.13)
- Gazebo Sim 8.15.0 (`gz sim --versions`). Jazzy의 Harmonic 계열이다. 바이너리는 `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`
- Pinky 워크스페이스 `/home/ubuntu/pinky_pro`, 패키지 `pinky_description`, `pinky_gz_sim`, `pinky_navigation`
- 컴파일러는 `/usr/bin/g++`. 기본 `c++`는 clang이라 libstdc++ 링크가 실패해서 g++로 빌드했다

재현에 필요한 최소 절차는 `ros2` apt 저장소를 넣고 Jazzy, Nav2, `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`을 설치한 뒤 위 Pinky 커밋을 colcon으로 빌드하는 것이다. 빌드 산출물은 이 저장소에 없다.

## 5. FMS 프로세스

| 항목 | 결과 |
| --- | --- |
| Backend | `./scripts/run_api.sh`, `http://127.0.0.1:8741` |
| Frontend | `./scripts/run_dashboard.sh`, `http://127.0.0.1:8742` HTTP 200. 본문에 Care-Move, Pinky, FMS |
| Health | `status=ok`, `database=ok`, `physical_control=blocked`, `ros_bridge=disabled` |
| Readiness | HTTP 200, `ready=true` |
| SQLite | `schema_migrations.version=2`, `navigation_goals.nav2_action_goal_id` 열, partial unique index `uq_tasks_one_active_per_robot`, `uq_zone_one_active`, journal WAL |
| 회귀 | unsourced pytest **50 passed**. 기존 49개에 UUID/yaw 단위 테스트 1개가 더해졌다. ROS를 source 한 셸에서 pytest를 돌리면 ROS 플러그인이 수집을 깨므로, 회귀는 ROS 변수를 뺀 환경에서 돌렸다 |
| Preflight, API 프로세스 | `not_ready`. 이유: distro, ros2 PATH, gz PATH, Pinky overlay, domain, RMW. Nav2 인터페이스 디렉터리는 `/opt/ros/jazzy/share`에 생겨서 `nav2_interface_missing`은 빠진다 |
| Preflight, sourced 에이전트 | `ready=true`. Pinky 패키지 `pinky_description`, `pinky_gz_sim`, `pinky_navigation` |
| 실물 | `mode=physical` 등록 HTTP 403 `PHYSICAL_CONTROL_BLOCKED`. `execution_mode=physical`, `physical_control_enabled`, `ros_bridge_enabled` 는 기동 거부. `/api/robots/pinky-1/cmd_vel` 은 HTTP 404 |

Gazebo 모드 기동은 `FMS_EXECUTION_MODE=gazebo`다. 기본 `.env`는 mock이다.
