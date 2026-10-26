# Phase 2.2 라이브 장애 결과

시각은 KST(UTC+9)다. 시뮬레이터는 도메인 73의 Gazebo Pinky 1대다. 상태가 불확실한 goal은 다시 보내지 않았다. 재현하지 못한 항목은 PASS로 올리지 않는다.

원본 로그:

- 정상 종료: [phase2-2-logs/faults-first/normal_stop.json](phase2-2-logs/faults-first/normal_stop.json)
- 주입이 주행 창을 놓친 시도: [phase2-2-logs/faults-invalid-window/](phase2-2-logs/faults-invalid-window/summary.json)
- 주행 중 주입: [phase2-2-logs/faults-inflight/summary.json](phase2-2-logs/faults-inflight/summary.json)

## 요약

| 사례 | 판정 | 근거 |
| --- | --- | --- |
| 에이전트 정상 종료 | PASS | 세션 stop, pose `agent_lost`, 후속 task는 배정되지 않음 |
| 에이전트 SIGKILL | PASS | UUID가 남은 채 `RECOVERY_REQUIRED`. 완료 증거 없음. 재전송 없음 |
| FMS 재시작 | PASS | 이유 `process_restart`. UUID 유지. 완료로 올리지 않음 |
| 작업 취소 | PASS | HTTP 202, Nav2 canceled, FMS `CANCELED`, UUID 저장 |
| Nav2 abort | PASS | 벽 밖 목표. `FAILED` / `nav2_goal_aborted`. UUID가 에이전트와 DB에 같음 |
| Stale epoch | PASS | epoch 0 성공은 `rejected_stale_owner`. 가짜 UUID는 저장되지 않음 |
| 같은 로봇의 두 번째 세션 | PASS | 주행 중 open은 409 |
| 두 번째 claim | PASS | `send_navigate_to_pose` false. goal 행 1개 |
| Lease 만료 후 성공 | PASS | `lease_invalid` 유지. UUID 저장. disposition `rejected_task_not_running` |
| 재연결 후 중복 goal | PASS | 409와 claim 거절. 새 NavigateToPose를 보내지 않음 |
| 네트워크만 끊고 프로세스는 살린 경우 | NOT_RUN | 프로세스 종료와 구분되는 단절을 따로 만들지 않음. 구분할 수 없으면 `RECOVERY_REQUIRED`이며 재전송하지 않는다 |
| 강제 DB 쓰기 실패 | NOT_RUN | pytest의 실패 처리만 있다 |

## 정상 종료

에이전트 종료 코드 0. 보낼 goal이 없어 `nav2_goal_sent`는 false다. stop 이후 `pose_source`는 `agent_lost`, `pose_trustworthy`는 false, `agent_online`은 false다. 이어서 만든 task `d0ba9343-25f6-4fd8-aefa-46a9e75c0463`는 `PENDING`에 남았고, goal 없이 `CANCELED`가 되었다.

## 주행 중 세션, claim, stale epoch

Task `9a60a5e0-2dd1-4ecc-9c3b-b6298e0f7a58`. Nav2가 수락한 뒤 FMS status는 `RUNNING`이고 UUID는 `d261bdd4-5024-4342-b05e-76f1cb03f4ab`였다.

- 두 번째 `POST /api/agent/session`은 409.
- 같은 execution에 대한 두 번째 claim은 `send_navigate_to_pose=false`, reason `fence_mismatch`, `navigation_goal_count` 1. status가 이미 `RUNNING`이라 `DISPATCHING` claim fence를 통과하지 못한다. goal 행은 늘지 않았다.
- owner epoch 0과 UUID `deadbeef-dead-beef-dead-beefdeadbeef`인 성공 결과는 HTTP 200이지만 status는 `RUNNING`, disposition은 `rejected_stale_owner`다. DB에 deadbeef UUID는 없다.
- 원래 에이전트의 결과가 나중에 도착했다. 최종 UUID는 수락 때의 `d261bdd4-5024-4342-b05e-76f1cb03f4ab`이고 goal 수는 1이다. 최종 status는 `COMPLETED`다. 거리 0.2453 m라 원래 goal의 완료 조건을 만족했다. stale 결과는 그 완료의 UUID가 아니다.

## 작업 취소

Task `8d341b38-bc5a-4f1b-82c5-cdeab58d00dc`. `RUNNING`인 동안 취소 API는 202다. 에이전트가 Nav2 goal을 취소했다. outcome `canceled`, Nav2 status 5, FMS는 `CANCELED`다. UUID `80a46374-1e73-4fff-9437-e77bc8e11657`가 에이전트 JSON과 DB에 같다. disposition `agent_ack`. goal 수 1. 시작 pose와 취소 시점 pose는 거의 같다.

## Nav2 abort

Task `347cb043-61c5-4526-8472-8eb816f158e3`, 목표 (3, 0). 공장 벽 밖이다. FMS는 `FAILED`, `failure_reason`은 `nav2_goal_aborted`다. UUID `5bde81f3-5238-4cc5-b98b-7c5fbb8bb0aa`가 에이전트와 DB에 같다. disposition `accepted_abort`. goal 수 1. 부재 이유는 null이다. UUID가 발급된 abort다.

## SIGKILL

Task `3775073f-cc0a-4882-aa89-7bc6070c4326`. `RUNNING`과 UUID `5e6104a5-c491-4571-b0e8-7ba164112363`를 확인한 뒤 에이전트에 SIGKILL을 보냈다. finally의 stop은 실행되지 않는다. 약 12초 뒤 status는 `RECOVERY_REQUIRED`, 이유는 `agent_session_lost`다. 외부 세션 heartbeat 한계는 8초다. 그 시간이 지나 세션이 사라진 뒤에 reconciliation이 프로세스 종료와 네트워크 단절을 구분하지 않고 세션 상실로 올린다. `heartbeat_timeout`이라는 이유 문자열은, 세션 객체가 아직 살아 있고 task heartbeat만 오래된 경로의 pytest에 있다. 이번 SIGKILL의 저장 이유는 `agent_session_lost`다.

UUID는 수락 직후 저장분이 그대로다. `completion_evidence`는 null이다. goal 수 1. 같은 goal을 다시 보내지 않고, Nav2에 남아 있던 goal은 상태 기록 뒤 cancel cleanup으로 지운 다음 홀드를 `FAILED`로 닫았다.

## FMS 재시작

Task `1e876469-3eb0-4f1f-b1f2-594f0521d0c9`. `RUNNING`, UUID `757e6602-15f0-4a79-82c3-4c44593f55f7`인 동안 API pid 27506에 SIGTERM을 보내고, 프로세스가 끝난 뒤 API 하나만 다시 띄웠다. 새 pid는 28906, boot는 그 프로세스의 새 id다. 기동 reconciliation 이유는 `process_restart`다. UUID는 같다. goal 수 1. `completion_evidence`는 null이다. 새 프로세스는 죽은 에이전트의 세션을 기억하지 않으므로 늦은 결과를 완료로 받지 않는다. 홀드는 재전송 없이 `FAILED`로 닫았다.

이 재시작 프로세스는 하니스의 ROS 환경을 물려받아 health preflight가 `ready`로 보였다. 검증이 끝난 뒤 그 프로세스를 내리고, ROS 환경이 없는 API로 다시 띄웠다. 그 boot는 `29d33f24-9d6c-4ae3-9ea4-aa6fb2c81fc4`이고 preflight는 `not_ready`다.

## Lease 만료 후 성공

별도 DB `data/fms-phase22-lease2.sqlite3`. lease TTL 8초, heartbeat timeout 120초. 프로세스 lock은 끄고 포트 8743에만 띄웠다. 에이전트는 주행 중 heartbeat를 보내지 않았다.

Task `c7f90e87-f16f-416a-8aaf-331554de90e9`. Nav2 outcome `succeeded`, UUID `05051264-19cd-44e5-a099-cb8ac2dd30ab`. 결과를 보내기 전에 status는 이미 `RECOVERY_REQUIRED`, 이유 `lease_invalid`였다. 그 다음 현재 epoch로 성공을 보내자 UUID가 DB에 저장되고 disposition은 `rejected_task_not_running`이 되었다. status는 `RECOVERY_REQUIRED`로 남았고 `completion_evidence`는 null이다. 완료로 간주하지 않았고 다시 보내지 않았다.

이전 파일 `data/fms-phase22-lease.sqlite3`는 지우지 않았다. 그 시도는 세션이 먼저 죽어 UUID가 DB에 남지 않았다. lease2가 그 수정의 실측이다.

## 창을 놓쳐 PASS가 아닌 시도

초기 하니스는 FMS status가 `RUNNING`이 된 뒤에만 주입했다. 그때 에이전트는 Nav2가 끝난 뒤에야 수락을 올렸으므로 `RUNNING` 구간이 매우 짧았다. 그 시도의 FAIL은 제품이 409를 어겼다는 뜻이 아니다.

- `4ba9ed8e`와 `b3bdcba5`는 주입 전에 `COMPLETED`가 되었다.
- 취소로 기록하려던 `144e991e-9a1b-4419-a79d-8da9693fae31`는 취소 요청 전에 Nav2가 성공하고 거리가 0.25 m를 넘어 `pose_mismatch`가 되었다. 현재 코드의 pose hold 실측으로는 남기고, 취소 PASS로는 세지 않는다.
- kill로 기록하려던 시도는 `RUNNING`을 보기 전에 프로세스를 끊어 UUID가 null이었다. 위의 SIGKILL PASS와 다르다.
- 재시작으로 기록하려던 `8b16d7a7`는 API를 내리지 않은 채 `COMPLETED`가 되었다.

이 항목들을 삭제하지 않았다.
