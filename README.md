# Care-Move FMS

Pinky Pro 3대를 관제하기 위한 독립 Fleet Management System입니다. 작업의 권위 있는 상태는 이 저장소의 데이터베이스에만 있고, Open-RMF 런타임에 연결하지 않습니다. 기본 실행은 Mock입니다. Gazebo Nav2 단일 로봇 E2E는 이 머신에서 실행하지 못했고, 그 상태를 PASS로 적지 않습니다. 기록은 [docs/phase2-report.md](docs/phase2-report.md)입니다.

이 저장소는 기존 Care-Move / Open-RMF 프로젝트를 복제한 것이 아닙니다. 이 개발 머신에는 그 소스와 보고서가 없었고, 기존 비밀값도 복사하지 않았습니다.

## 지금 되는 것

`POST /api/tasks` 로 접수한 이동 작업이 스케줄러와 Mock 로봇 에이전트를 거쳐 `COMPLETED` 가 됩니다. 완료는 HTTP 응답이 아니라 Mock goal 결과, goal id, execution id, owner epoch, 위치 오차로 판정합니다.

실물 모터 명령과 `/cmd_vel` 은 거부됩니다. Nav2 goal은 Gazebo 모드의 로봇 에이전트만 보낼 수 있고, 시뮬레이터 preflight가 실패하면 그 에이전트도 goal을 보내지 않습니다.

## 실행

```bash
python3 -m venv .venv
.venv/bin/pip install -e "backend[dev]"
./scripts/run_api.sh
```

다른 터미널에서 관제 화면:

```bash
./scripts/run_dashboard.sh
```

- API: http://127.0.0.1:8741
- 관제 화면: http://127.0.0.1:8742
- 헬스: http://127.0.0.1:8741/api/system/health

`./scripts/run_api.sh` 는 `.env` 가 없으면 이 프로젝트용 lease secret 을 새로 만듭니다. 셸 설정 파일은 바꾸지 않습니다. 스크립트는 source 하지 마세요.

한 대만 붙는 로봇 에이전트(이 환경에서는 preflight 실패로 종료 코드 2):

```bash
./scripts/run_agent.sh --robot-id pinky-sim
```

테스트:

```bash
cd backend && ../.venv/bin/pytest
```

## 격리 기본값

| 항목 | 값 |
| --- | --- |
| ROS_DOMAIN_ID (기록만, 노드 없음) | 73 |
| API | 8741 |
| Dashboard | 8742 |
| 선택 Postgres | 55432, compose profile `postgres` |
| Task DB | `data/fms.sqlite3` |
| Lock | `data/fms.lock` |
| Environment ID | `care-move-fms-local` |

Phase 2의 Gazebo 주행과 Phase 5 실물 시험은 실행되지 않았습니다. 상태는 [docs/phase2-report.md](docs/phase2-report.md)와 [docs/roadmap.md](docs/roadmap.md)에 있습니다.

## 문서

- [docs/architecture.md](docs/architecture.md)
- [docs/migration-analysis.md](docs/migration-analysis.md)
- [docs/isolation-plan.md](docs/isolation-plan.md)
- [docs/risk-register.md](docs/risk-register.md)
- [docs/verification-report.md](docs/verification-report.md)
