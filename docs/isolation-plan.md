# 격리 계획

신규 FMS와 기존 Care-Move가 같은 로봇, 같은 DB, 같은 ROS 그래프를 만지지 않게 하는 운영 규칙입니다. 이 개발 세션은 기존 트리가 없는 머신에서 수행했으므로, 아래 항목은 앞으로 로봇 워크스테이션에 둘 때의 기준이기도 합니다.

## 저장소

- 신규 코드는 이 git 저장소에만 있습니다.
- 기존 remote, branch, worktree에 commit하거나 push하지 않습니다.
- 심볼릭 링크로 기존 소스를 가리키지 않습니다.
- 제안 경로 `/home/sw/DevSpace/care-move-fms` 는 이 머신에 없습니다. 로봇 PC에 둘 때는 그 빈 경로로 이 저장소를 clone 하면 됩니다. 기존 `care-move-rmf-starter` 를 복사해 수정하는 방식은 쓰지 않습니다.

## 런타임

| 자원 | 신규 FMS | 기존과 공유 |
| --- | --- | --- |
| ROS_DOMAIN_ID | 73. Phase 1은 노드를 만들지 않음 | 금지. 기존 도메인 번호는 여기서 확인하지 못했으므로, ROS를 켜기 전에 현장에서 대조 |
| DDS | `config/cyclonedds.fms.xml` 은 로드하지 않음 | 금지 |
| API | 8741 | 금지 |
| 관제 화면 | 8742 | 금지 |
| Gateway | 프로세스 내부 Mock. boot id는 메모리 | 기존 gateway 포트에 접속하지 않음 |
| Database | `data/fms.sqlite3` 또는 compose profile의 `care_move_fms` | 금지. URL에 레거시 표식이 있으면 기동 거부 |
| Docker project / network / volume | `care-move-fms`, `care-move-fms-net`, `care-move-fms-pg` | 사용하지 않았고, 기본 `docker compose up` 은 profile 때문에 아무 서비스도 안 뜸 |
| Lock / PID | `data/fms.lock` | 두 번째 프로세스는 같은 DB로 기동 거부 |
| 로그 | `logs/fms.log` | 기존 로그 디렉터리를 쓰지 않음 |
| Lease 주체 | `FMS_ENVIRONMENT_ID=care-move-fms-local` 와 신규 secret | 기존 HMAC 키 복사 금지 |
| Task store | 이 DB의 `tasks` | 공유 금지 |

`scripts/run_api.sh` 는 실행 파일로만 동작해야 합니다. source 하면 안 되고, `~/.bashrc` 나 ROS setup 을 고치지 않습니다.

## 실물 로봇

기본 실행 모드는 `mock` 입니다. `physical` 등록, `FMS_PHYSICAL_CONTROL_ENABLED=true`, `FMS_ROS_BRIDGE_ENABLED=true`, `FMS_EXECUTION_MODE=physical` 은 프로세스가 뜨지 않거나 403을 반환합니다. `gazebo` 로봇 등록은 `FMS_EXECUTION_MODE=gazebo` 일 때만 열리고, 그 경우에도 API 프로세스는 Nav2 클라이언트를 만들지 않습니다. 에이전트는 preflight가 준비되지 않으면 goal을 보내지 않습니다.

FMS는 `/cmd_vel` 을 발행하지 않습니다. `robot_agent/nav2_bridge.py` 의 `send_navigate_to_pose` 는 계속 예외를 냅니다. Gazebo 클라이언트는 모드, 도메인, preflight가 모두 맞을 때만 열리고, 액션 결과를 관측하지 못하면 SUCCEEDED를 만들지 않습니다.

같은 Pinky의 주소, 모터 장치, SSH, GPIO를 이 세션에서 조사하지 못했습니다. 그 정보가 생기기 전에는 신규 FMS의 실물 제어권 인계를 하지 않습니다. 기존 프로젝트가 점유한 제어권을 해제하지 않습니다.

## 화면

관제 화면은 `/fms-api` 를 이 API로만 프록시합니다. GET 묶음(함대, 작업, 구역, 이벤트, 헬스)은 서버에서 읽기 트랜잭션 뒤 롤백합니다. 폴링이 조정 횟수나 작업 상태를 바꾸면 안 됩니다. 이 동작은 `test_reads_do_not_move_task_state` 로 확인했습니다.

## 증거

세션 중의 읽기 전용 조회는 [isolation-evidence.txt](isolation-evidence.txt)입니다. 기존 프로세스를 종료하는 명령은 사용하지 않았습니다.
