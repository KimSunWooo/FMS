# FMS Task 목록 관리 UI 개선 보고

작성 시각: 2026-10-28

관제 화면은 진행 중인 Task만 먼저 보여주고, 취소·실패로 끝난 Task는 별도 이력에서 보거나 숨길 수 있다. 숨기기는 취소가 아니며 Task 행을 삭제하지 않는다. 실물 주행과 신규 Pinky 전송은 하지 않았다.

## 1. 기존 Task 목록 구조

변경 전에는 `GET /api/fleet-tasks`가 생성 시각 역순으로 모든 fleet task를 반환하고, 관제 화면이 그 배열을 한 대기열에 모두 그렸다. 진행 중, 취소, 실패가 같은 목록에 쌓였다.

취소 API, 상태 전이, 예약 해제, Receiver 취소 ACK는 이미 있었다. 이번 작업은 그 경로를 다시 만들지 않았다.

- 취소는 계속 `POST /api/fleet-tasks/{task_id}/cancel`이다.
- `FAILED` 취소는 HTTP 409이고 실패 문구를 유지한다.
- `ACKNOWLEDGED`와 `DELIVERED`는 Receiver 취소 ACK 뒤에만 `CANCELLED`가 된다. 통신 실패는 `CANCEL_PENDING`이다.
- `CANCELLED` 재취소는 멱등이다.
- 예약 해제는 그 Task의 confirmed 행만 `released`로 바꾼다.

수신 ACK는 주행 완료가 아니다. `ACKNOWLEDGED`는 진행 중 목록에 남긴다. `COMPLETED` 상태는 만들지 않았다.

종료로 보는 상태는 현재 코드의 `CANCELLED`, `FAILED`, `UNREACHABLE`이다.

## 2. 기존 취소 기능 재사용 내역

취소 버튼은 기존 cancel API만 호출한다. 보관 API는 Receiver에 명령을 보내지 않고 상태와 예약도 바꾸지 않는다.

확인 Dialog를 연 뒤에만 취소 요청을 보낸다. Dialog에는 Task ID, 로봇, 현재 상태, 목적지, 해제될 활성 예약 수, 실물 취소 ACK 필요 여부를 보여 준다. 요청이 끝나기 전에는 같은 Dialog에서 다시 보낼 수 없다. `CANCEL_PENDING`이면 취소 완료로 쓰지 않는다.

`CANCELLED`에는 추가 취소 대신 취소 완료와 목록 숨기기를 둔다. `FAILED`는 취소 버튼을 두지 않고 실패 원인과 숨기기만 둔다.

## 3. 수정 및 생성한 파일

- `backend/app/services/fleet_task_machine.py` — 진행 중 목록과 종료 이력 상태 집합
- `backend/app/models/tables.py` — `is_archived`, `archived_at`, `archived_by`
- `backend/app/core/database.py` — 스키마 5. 없는 컬럼만 `ALTER`. 기존 행 삭제는 없음
- `backend/app/services/fleet_task_service.py` — `browse`, `set_archived`, 활성 예약 수
- `backend/app/api/fleet_tasks.py` — `GET /browse`, `POST /{id}/archive`, `POST /{id}/restore`
- `backend/tests/test_fleet_task_browse.py`
- `frontend/src/components/fleet-task-panel.tsx`
- `frontend/src/components/fleet-console.tsx` — 이력 경로는 회색 점선

`GET /api/fleet-tasks`는 페이지 없이 전체 목록을 그대로 반환한다. 숨긴 Task도 이 응답에는 남는다.

## 4. Task 목록 UI 변경 사항

목록은 세 영역이다.

- 진행 중: `CREATED`, `PLANNING`, `PLANNED`, `RESERVED`, `WAITING`, `REPLANNING`, `DISPATCHING`, `DELIVERED`, `ACKNOWLEDGED`, `CANCEL_PENDING`
- 완료·취소 이력: `CANCELLED`, `FAILED`, `UNREACHABLE`
- 숨긴 Task: 보관한 종료 Task

각 행에는 상세, 경로, 그리고 상태에 맞는 취소 또는 숨기기/다시 표시가 있다. 진행 중 Task에는 숨기기가 없다.

취소가 성공하면 목록과 선택 Task, 지도 경로를 다시 읽는다. `CANCEL_PENDING` 경로는 지도에서 빼 둔다. 다른 Task의 저장 경로는 바꾸지 않는다. 이력에서 경로를 열면 후보 경로 없이 저장된 경로만 회색 점선으로 그린다.

## 5. 필터·페이지네이션 구현

화면은 `GET /api/fleet-tasks/browse`만 사용한다. 한 번에 전체 행을 받아 길게 그리지 않는다.

- 상태: 진행 중은 전체, `RESERVED`, `WAITING`, `DISPATCHING`, `ACKNOWLEDGED`, `CANCEL_PENDING`. 이력과 보관은 전체, `CANCELLED`, `FAILED`, `UNREACHABLE`
- 로봇: 전체, Pinky-1, Pinky-2, Pinky-3. 배정 로봇이 없으면 요청 로봇으로 맞춘다
- 검색: Task ID 부분 문자열
- 페이지: 기본 10건, 10/20/50만 허용. 그 외 크기는 HTTP 400. 최신 생성 순. 전체 건수와 이전/다음

현재 운영 DB는 진행 중 9건, 이력 8건이라 10건 페이지에서는 다음 버튼이 비활성이다. 12건을 넘는 페이지 이동은 테스트 DB에서 확인했다.

## 6. 목록 숨기기·복원 방식

종료 상태만 보관할 수 있다. 진행 중 보관은 HTTP 409이다.

보관은 `is_archived`, `archived_at`, `archived_by`만 기록한다. 상태, Task ID, 수신 ACK, 취소 ACK, 예약 행, 이벤트는 남긴다. `DELETE`는 쓰지 않는다. 복원은 보관 표시만 지우고 이력 목록으로 되돌린다. 이미 보관됐거나 이미 복원된 요청은 같은 결과를 다시 반환한다.

서버를 새로 연 뒤에도 보관 플래그가 SQLite에 남는 것을 테스트로 확인했다. 테스트는 프로세스 잠금을 해제한 뒤 같은 DB로 앱을 다시 띄운다.

## 7. 기존 Task 및 Reservation 보존 결과

스키마 5 적용 후 fleet task는 17건 그대로다. 확인된 예약은 171건이고, 이전에 해제된 예약 행 120건도 남아 있다.

화면에서 이미 `CANCELLED`였던 `29103196-f1e0-4fbc-947f-407af97f1bef`를 숨긴 뒤 다시 표시했다. 상태는 `CANCELLED`이고 `is_archived`는 다시 false다. 숨긴 목록은 0건, 이력은 8건, 진행 중은 9건이다.

취소 Dialog는 가상 `DELIVERED` Task에서 내용만 확인하고 닫았다. 취소 요청은 보내지 않았다. 신규 실물 Task 전송과 실물 취소 명령은 하지 않았다.

## 8. 테스트 결과

변경 전 `pytest -q`는 112건 통과였다. 이전에 말한 104건은 취소 테스트가 더해지기 전 수치다.

변경 후 전체 pytest는 117건, 종료 코드 0이다. 추가 5건이 목록 분리, 검색, 로봇·상태 필터, 페이지, 예약이 있는 Task만 해제, 보관이 Receiver와 예약을 건드리지 않음, 실패 이력 유지, `CANCEL_PENDING` 보관 거부, 재시작 후 보관 유지를 본다.

Frontend `tsc --noEmit`와 `FMS_NEXT_DIST_DIR=.next-verify` 빌드도 통과했다. 페이지 JS는 약 34.8 kB다. 빌드가 tsconfig에 넣은 `.next-verify` include는 제거했다.

관제 화면에서 진행 중 9건, 이력 8건, 취소 Dialog, 이력 경로의 점선, 숨기기와 다시 표시를 확인했다. 5cm 시작 버튼은 누르지 않았다.

## 9. 기존 기능 회귀 여부

유지한 항목: Pinky-1 SSH, Nav2 READY, `1f_map_f`, Pinky-2·3 가상 등록, A*와 Yen, 스케줄러, 예약, 전달, 수신 ACK와 취소 ACK, 5cm 화면. Collision Monitor와 StopZone 반경은 바꾸지 않았다. 5cm 실주행은 계속 blocked이며 성공이 아니다.

기존 `GET /api/fleet-tasks`와 취소 API의 409 정책은 그대로다.

## 10. 미해결 사항

- 운영 DB의 진행 중 Task가 10건 미만이라 화면의 다음 페이지는 이번 데이터로는 넘어가지 않는다. 서버 페이지 테스트는 통과했다.
- `UNREACHABLE` Task는 지금 운영 목록에 없다. 이력 필터와 보관 대상에는 포함된다.
- `ACKNOWLEDGED`를 완료로 옮기는 기능은 없다. 실제 주행 완료가 검증되기 전에는 `COMPLETED`를 만들지 않는다.
- 5cm StopZone 점유와 실주행은 이번 범위 밖이다. 주행 성공으로 기록하지 않는다.
