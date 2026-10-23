# Care-Move FMS 아키텍처

Phase 1은 하나의 FastAPI 프로세스 안에 등록, 작업, 스케줄, 소유권, 구역 예약, 조정을 넣은 모듈러 모놀리스입니다. 로봇 에이전트는 프로세스 안의 Mock이며, 나중에 로봇마다 프로세스를 나눌 수 있게 명령과 결과의 경계만 나눠 두었습니다.

Open-RMF의 API, Dispatcher, TaskManager, Fleet Adapter, Traffic Schedule에는 링크하지 않습니다. ROS 2, Nav2, DDS, TF, Gazebo는 이후 단계의 실행 경로로 남기고, 이 빌드에서는 켜지지 않습니다.

## 요청 경로

```text
관제 화면 / API 클라이언트
        |  REST, 조회는 무기록
        v
FastAPI
        |
        +-- Robot registry
        +-- Task manager  -- 상태 기계, 멱등 키
        +-- Scheduler     -- 결정적 규칙과 유클리드 비용
        +-- Ownership     -- epoch, 로봇당 활성 작업 1개
        +-- Traffic       -- 용량 1 구역 예약
        +-- Mock gateway  -- HMAC lease, 프로세스 boot id
        +-- Mock agent    -- Nav2를 흉내 낸 수락/결과/취소
        +-- Reconciliation
        v
SQLite (WAL, BEGIN IMMEDIATE)
```

관측(텔레메트리, goal 결과)은 작업 상태와 별도입니다. DB에 `COMPLETED`가 있어도 그 행의 completion evidence(goal 상태, id, epoch, 거리)가 없으면 도착으로 보지 않습니다. 화면의 GET은 읽기 세션을 열고 롤백하므로 조정을 돌리거나 상태를 바꾸지 않습니다.

## 상태 기계

```text
CREATED → PENDING → ASSIGNED → DISPATCHING → RUNNING → COMPLETED
                         |            |           |
                         |            +-----------+--→ CANCEL_REQUESTED → CANCELED
                         |            |           |
                         +→ PENDING (goal 생성 전 실패)
                         |
                    RECOVERY_REQUIRED → CANCELED | FAILED
```

금지한 지름길:

- HTTP 202/200 만으로 `COMPLETED` 금지. 생성 API는 202와 `motion_completed: false` 를 반환합니다.
- goal accepted 는 `RUNNING` 이지 완료가 아닙니다.
- `COMPLETED` 는 succeeded 결과와 goal id, execution id, session, owner epoch, 유효 lease, 허용 오차 안의 위치가 맞을 때만.
- 취소 요청(`CANCEL_REQUESTED`)과 취소 확인(`CANCELED`)을 나눕니다.
- 시간 초과는 성공도 취소 완료도 아닙니다. `RECOVERY_REQUIRED` 로 격리합니다.
- 프로세스가 다시 떠도 in-flight goal 을 재전송하지 않습니다.

Goal을 만들기 전에 실패한 배정은 모션이 없다는 점이 트랜잭션 안에서 확인되므로 `PENDING`으로 되돌리거나, 횟수를 넘기면 `FAILED`로 닫습니다. Goal 행이 생긴 뒤의 불확실함은 `RECOVERY_REQUIRED` 이며, 그 로봇의 새 주행을 막습니다. 운영자가 닫을 때도 `COMPLETED`로는 갈 수 없고, 위치를 확인했다고 인정하기 전에는 구역 block 을 풀지 않습니다.

## 소유권

활성 상태 `ASSIGNED`, `DISPATCHING`, `RUNNING`, `CANCEL_REQUESTED`, `RECOVERY_REQUIRED` 에 대해 로봇당 한 행만 허용하는 부분 유니크 인덱스가 있습니다. 배정은 `BEGIN IMMEDIATE` 트랜잭션 안에서 epoch를 올리고, 오래된 epoch의 결과 기록은 작업 상태를 바꾸지 않은 채 `fence_reject` 로 남깁니다. 메모리 락만으로 이 제약을 만들지 않았습니다.

재시작 시 gateway boot id는 메모리에만 있으므로 이전 lease 서명은 실패합니다. 에이전트 세션 id도 새로 발급됩니다. 진행 중 작업은 복구 격리되고 navigation goal 행은 늘어나지 않습니다.

## 스케줄러

후보에서 빼는 조건: 오프라인, Mock이 아님, 제어기 준비 안 됨, 배터리 하한, fault 또는 estop, 위치 불신, 에이전트 다운, gateway 다운, 활성 작업, 다른 로봇이 잡은 목적지 구역.

비용은 설정 가능한 가중치로 다음을 더합니다.

```text
travel (euclidean) + battery penalty + congestion penalty + queue penalty
```

로그와 `scheduler_decision` 이벤트의 `distance_model` 은 `euclidean_phase1_not_obstacle_aware` 입니다. 직선거리는 복도나 장애물을 반영하지 않습니다.

## 교통

1차 구현은 용량 1인 구역 예약입니다. 복도와 교차로를 포함한 다섯 구역을 DB에 두고, 목적지가 구역 안이면 배차 시 예약을 잡습니다. 예약 만료만으로 해제하지 않습니다. 위치를 신뢰할 수 없거나 로봇이 오프라인이면 예약을 `blocked`로 두고, 운영자가 비었음을 확인하기 전에는 다른 로봇이 들어가지 못합니다. 대기 예약 상태는 만들지 않습니다. 그래서 맞교차 대기열 교착은 생기지 않는 대신, 안전하지 않은 동시 진입도 허용하지 않습니다.

## 통신 선택

Phase 1 서버와 Mock 에이전트는 같은 프로세스의 함수 호출입니다. ROS 그래프에 참여하지 않습니다.

Phase 2에서 로봇마다 에이전트 프로세스를 두고, FMS와는 WebSocket으로 명령과 결과를 주고, Nav2와는 그 프로세스 안의 액션 클라이언트만 두기로 했습니다. gRPC는 계약은 분명하지만 이 규모의 관측에 불필요합니다. FMS 프로세스가 DDS에 직접 붙으면 도메인과 `/cmd_vel` 경계를 지키기 어렵습니다. FMS는 `/cmd_vel`을 발행하지 않습니다. `SafetyGuard.publish_cmd_vel` 은 항상 거부합니다.

## 데이터베이스

Phase 1 저장소는 SQLite입니다. 작성자는 한 API 프로세스이고, `BEGIN IMMEDIATE`와 부분 유니크 인덱스로 활성 작업과 구역 점유를 직렬화합니다. Docker와 PostgreSQL은 이 머신에 없어 프로덕션 다중 프로세스 검증은 하지 않았습니다. 스키마는 SQLAlchemy라 `DATABASE_URL`만 바꾸면 PostgreSQL을 붙일 수 있고, compose 파일은 profile `postgres`, 호스트 포트 55432, 데이터베이스 이름 `care_move_fms` 입니다. 그 파일은 실행하지 않았습니다.

레거시 이름으로 보이는 DB URL(`care_rmf`, `db_care`, `open-rmf`, `rmf_web`, `care-move-rmf-starter`)은 기동을 거부합니다.

## 이 빌드에서 의도적으로 없는 것

- Gazebo, Nav2, 실물 Pinky 연결
- Open-RMF 트래픽 협상 재구현
- 경로 계획. 비용은 유클리드입니다.
- 대시보드 조회에 엮인 조정 작업
