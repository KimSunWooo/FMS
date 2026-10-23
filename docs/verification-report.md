# 검증 기록

구분: PASS, FAIL, PARTIAL, NOT_RUN, BLOCKED.

Mock 결과를 실물 결과로 적지 않습니다. 실행 환경은 이 저장소의 pytest이며, ROS와 Gazebo와 실물 Pinky는 없습니다.

## 자동 테스트

`cd backend && pytest` 결과 34 passed. 개별 이름은 아래와 같습니다. 전부 PASS.

작업과 API

- `test_happy_path_and_rejections`
- `test_mock_task_reaches_completed_only_after_goal_evidence`
- `test_duplicate_idempotency_key_returns_the_same_task`
- `test_reads_do_not_move_task_state`
- `test_health_hides_secret_and_reports_isolation`
- `test_cancel_before_dispatch_is_terminal_without_a_goal`

소유권

- `test_one_robot_cannot_take_two_active_tasks`
- `test_parallel_schedule_keeps_a_single_owner`
- `test_database_constraint_rejects_a_second_active_row`
- `test_stale_epoch_cannot_complete`
- `test_assign_transaction_rolls_back`
- `test_pose_mismatch_does_not_complete`

복구

- `test_restart_quarantines_inflight_and_does_not_replay`
- `test_agent_crash_then_late_success_stays_in_recovery`
- `test_cancel_ack_is_not_completion_and_timeout_is_not_cancel`
- `test_resolve_recovery_does_not_replay_a_goal`
- `test_heartbeat_loss_is_recovery_not_success`

Lease와 안전

- `test_gateway_rejects_stale_and_foreign_leases`
- `test_expired_lease_is_rejected`
- `test_dispatch_without_a_lease_sends_no_goal`
- `test_estop_does_not_publish_velocity`
- `test_physical_and_ros_cannot_be_enabled`
- `test_physical_registration_is_rejected`

배차

- `test_low_battery_robot_is_not_selected`
- `test_tie_breaks_on_robot_id`
- `test_controller_offline_robot_is_skipped`

교통 (Mock 정책)

- `test_single_zone_allows_one_robot`
- `test_corridor_and_intersection_reject_a_second_entry`
- `test_offline_owner_blocks_the_zone_until_an_operator_clears_it`
- `test_no_wait_queue_can_deadlock`

다중 로봇 (Mock)

- `test_three_robots_finish_independent_tasks`
- `test_one_robot_failure_leaves_the_other_two_running`

정적 격리

- `test_sources_do_not_import_rmf_or_ros`
- `test_pyproject_has_no_rmf_dependency`

## 요청 시나리오와 상태

| 시나리오 | 상태 | 근거 |
| --- | --- | --- |
| 같은 로봇에 활성 작업 둘 | PASS | Mock DB 제약과 스케줄 |
| 중복 HTTP 생성 | PASS | 멱등 키 |
| 이전 owner의 상태 갱신 | PASS | stale epoch |
| DB 트랜잭션 중단 | PASS | 배정 롤백 |
| FMS 재시작 | PASS | in-flight 격리, goal 수 유지 |
| 에이전트 종료 후 복구 | PASS | late success 거부 |
| 지연된 goal 결과 | PASS | 복구 중 결과 거부 |
| 취소 중 통신 단절 | PASS | 타임아웃은 취소 완료가 아님 |
| lease 없음, 만료, 이전 generation, 이전 boot, 다른 로봇 | PASS | Mock gateway |
| 에이전트 종료, heartbeat 중단, 오프라인 | PASS | 복구 격리 |
| 비상 정지 | PASS | Mock 취소 확인. velocity 발행 0 |
| Mock 3대 독립 작업 | PASS | |
| 1대 장애, 나머지 유지 | PASS | |
| 같은 목적지 구역 | PASS | 한 대만 완료, 나머지는 PENDING |
| 복도 동시 진입 | PASS | 예약 정책. 물리 맞교차는 NOT_RUN |
| 교차로 예약 충돌 | PASS | 예약 정책 |
| 예약 owner 종료 | PASS | blocked, 만료로 해제하지 않음 |
| 대기열 교착 | PARTIAL | 대기 상태를 만들지 않아 교착이 생기지 않음. 대기열 탐지기는 NOT_RUN |
| Gazebo Nav2 E2E | NOT_RUN | ROS 없음 |
| 실물 Pinky goal | BLOCKED | 이 빌드가 명령을 거부. 현장 승인 없음 |
| 실물 3대 교통 | NOT_RUN | |

## 관제 화면

데스크톱과 가로 390px에서 확인했습니다. Pinky #1–#3, 구역 지도, 작업 폼이 보였고 가로 스크롤로 버튼이 잘리지 않았습니다. 개방 구역 작업을 접수한 뒤 추적 줄이 `RUNNING`에서 `COMPLETED`로 바뀌고 `goal succeeded`가 표시됐습니다. 접수 직후 문구는 도착이 아니라고 알려 줍니다. 콘솔 오류는 없었습니다. 이 확인은 Mock 에이전트 기준입니다.
