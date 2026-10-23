# 이전 분석

이 문서는 두 종류를 섞지 않습니다.

- **사용자 제공 정보**: 이번 요청에 적힌 기존 Care-Move 성과와 미해결 문제. 이 머신에서 다시 보지 못했습니다.
- **이 머신에서 확인한 사실**: 읽기 전용 조회 결과.

코드를 복사하지 않았습니다. 기존 트리에 쓸 권한도 쓰지 않았습니다.

## 사용자 제공 정보 (여기서 재검증하지 않음)

프로젝트명 Care-Move. 스택은 Ubuntu 24.04, ROS 2 Jazzy, Pinky Pro 3대, Nav2, Gazebo Harmonic, FastAPI, Open-RMF, Fleet Adapter, RMF-Web, Gateway / Control Lease 로 적혀 있습니다.

소스 후보:

- `/home/sw/DevSpace/care-move-rmf-starter`
- `DB_care_RMF_System`

보고서 후보:

- `Care_Move_Development_Report_Day1_Day2_2026-10-22.md`
- `Care_Move_Day3_Development_Report_2026-10-23.pdf`

요청에 적힌 기존 성과(단일 Pinky의 RMF→Nav2 E2E, NavigateToPose SUCCEEDED, Pinky #1 odometry 관측, HMAC lease, reconciliation, Mock 2대 작업 4건, 어댑터 재시작 시 goal 재실행 차단)는 **기존 프로젝트의 보고**이며 신규 FMS의 검증 결과가 아닙니다.

요청에 적힌 미해결 문제는 신규 설계 요구로만 반영했습니다.

| 기존 문제 (요청 문구) | 신규에서의 대응 |
| --- | --- |
| Orphaned task | 활성 작업은 DB 유니크 인덱스가 로봇에 묶고, 프로세스 재시작 시 in-flight 는 `RECOVERY_REQUIRED` |
| Task ownership mismatch | `owner_epoch` 불일치 결과는 상태를 바꾸지 않음 |
| 작업 취소 timeout | 시간 초과는 `CANCELED`가 아니라 복구 격리 |
| 어댑터 재시작 후 소유권 상실 | 새 boot id / 세션 id. 이전 goal 재전송 없음 |
| Gateway 이전 세션 간섭 | 신규 environment id, 신규 secret, 기존 gateway에 접속하지 않음 |
| 기존 어댑터의 lease 미지원 | 신규 Mock gateway가 발행과 검증을 수행 |
| 실물 3대 동시 관제 미검증 | Phase 1은 Mock. 실물 명령 거부 |
| 다중 로봇 교통 미검증 | 용량 1 구역 예약만 구현. Gazebo 검증은 하지 않음 |
| `/cmd_vel` 접근 통제 | FMS는 해당 토픽을 발행하지 않음 |

## 이 머신에서 확인한 사실

조회 시각의 원문은 [isolation-evidence.txt](isolation-evidence.txt)에 있습니다.

- 워크스페이스는 빈 git 저장소에서 시작했습니다. 원격은 이 신규 작업용입니다.
- `/home/sw/DevSpace/care-move-rmf-starter`, `/home/sw/DevSpace/care-move-fms`, `/home/sw` 는 없습니다.
- `Care_Move` 보고서 파일은 `/workspace`, `/home`, `/opt`, `/tmp` 검색에서 나오지 않았습니다. 보고서 본문을 추정하지 않았습니다.
- `ros2`, Docker, ROS/RMF 프로세스, `ROS_*` 환경 변수는 없습니다.
- 그래서 Fleet Adapter, Gateway, RMF Task DB, launch 파일의 실제 심볼과 라이선스를 읽을 수 없었습니다.

## 재사용 판정

| 후보 | 판정 | 이유 |
| --- | --- | --- |
| Open-RMF 런타임 | 사용하지 않음 | 의존성도 import도 없음 |
| 기존 Gateway HMAC 구현 | 복사하지 않음 | 소스가 이 머신에 없음 |
| 기존 secret, SSH 키 | 복사하지 않음 | 파일 자체가 없음. 신규 secret은 로컬에서 생성 |
| 기존 화면 | 수정하지 않음 | 소스가 없음. 관제 화면은 새로 작성 |
| ROS 워크스페이스, systemd, bashrc | 변경하지 않음 | 대상이 없음 |

Lease에 넣은 필드는 요청서의 검증 목록(environment, robot, session, lease, generation, boot id, 만료, 폐기, HMAC)을 새로 구현한 것입니다. 기존 함수의 이식이 아닙니다.

## 유지하는 기술과 빼는 기술

유지(이후 단계의 인터페이스로 문서화, Phase 1은 미실행): ROS 2, Nav2 액션, odometry, TF, Gazebo.

제거: Open-RMF API, dispatcher, fleet adapter, RMF traffic schedule, RMF-Web, 기존 gateway 프로세스 접속.

Phase 1에 실제로 있는 것: Python 3.12, FastAPI, SQLAlchemy, SQLite, Next.js 관제 화면, Mock agent.
