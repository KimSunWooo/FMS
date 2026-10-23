# Care-Move FMS

Pinky Pro 3대를 관제하기 위한 독립 Fleet Management System의 Phase 1입니다. 작업의 권위 있는 상태는 이 저장소의 데이터베이스에만 있고, Open-RMF 런타임에 연결하지 않습니다.

이 저장소는 기존 Care-Move / Open-RMF 프로젝트를 복제한 것이 아닙니다. 이 개발 머신에는 그 소스와 보고서가 없었고, 기존 비밀값도 복사하지 않았습니다.

## 지금 되는 것

`POST /api/tasks` 로 접수한 이동 작업이 스케줄러와 Mock 로봇 에이전트를 거쳐 `COMPLETED` 가 됩니다. 완료는 HTTP 응답이 아니라 Mock goal 결과, goal id, execution id, owner epoch, 위치 오차로 판정합니다.

실물 모터 명령, `/cmd_vel`, Nav2 액션, Gazebo 실행은 이 빌드에서 거부됩니다.

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

Phase 2 이후의 Gazebo 연동과 Phase 5 실물 시험은 [docs/roadmap.md](docs/roadmap.md)에만 있습니다. 이 빌드는 그 단계를 실행하지 않습니다.

## 문서

- [docs/architecture.md](docs/architecture.md)
- [docs/migration-analysis.md](docs/migration-analysis.md)
- [docs/isolation-plan.md](docs/isolation-plan.md)
- [docs/risk-register.md](docs/risk-register.md)
- [docs/verification-report.md](docs/verification-report.md)
