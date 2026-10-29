# 5cm 시작 조건과 Map Server 지도 불일치

작성 시각: 2026-10-28

5cm POST와 Nav2 Goal은 호출하지 않았다. map_server 프로세스도 종료하지 않았다. 실주행 결과는 없다.

## 문제 1 — READY인데 관측 없음

원인: `snapshot()`이 느린 Nav2 폴의 `pose_checked_at`을 실시간 TF 스트림으로 덮어썼다. 스트림 샘플이 없거나 4초를 넘기면 그 필드를 `None`으로 바꿨다. `forward_5cm_service.start()`는 `checked_at`과 `pose_checked_at`을 한 번의 `fromisoformat`으로 읽고, `None`이나 파싱 실패를 모두 `Recent Nav2/TF observations unavailable`로 반환했다.

화면의 Nav2 READY는 `nav2_ready`만 본다. 현재 TF 문장은 마지막 성공 샘플을 최대 5초 보여 준다. 그래서 준비 완료와 TF가 보이는 동안에도 시작 요청은 어떤 값이 비었는지 말하지 않고 거절됐다.

수정:

- `checked_at`은 폴 완료 시각으로 유지한다.
- 폴의 pose는 `poll_pose`로만 남긴다. 만료되거나 실패한 실시간 TF 자리에 다시 넣지 않는다.
- 실시간 TF는 `fresh`, `expired`, `missing`, `invalid`, `lookup_failed`, `observer_down`, `withheld`로 구분한다.
- 잘못된 시각을 현재 시각으로 바꾸지 않는다. 미래 시각은 `invalid`다.
- 5cm 시작은 비어 있는 필드 이름을 409 본문에 넣는다.

수정 후 라이브 상태: `nav2_ready=true`, `live_tf_status=fresh`, `live_tf_age_s` 약 0.03초. 타임스탬프 거절은 사라지고, 아래 지도 불일치만 시작을 막는다.

## 문제 2 — 실행 지도와 FMS 지도

원인: `_ensure_lifecycle()`은 `/map_server`가 `active`이면 YAML을 읽지 않고 반환했다. 실행 중인 인자 `yaml_filename`은 삭제된 `config/maps/physical_site/1f_map.yaml`이다. FMS 설정은 `1f_map_f.yaml`이다.

`/map`과 `1f_map_f`의 크기는 같다. 186×236, 해상도 0.01 m, origin 0. 점유 셀은 같지 않다.

| 대상 | occupancy sha256 |
|---|---|
| 로드된 `/map` | `52a69842d7e68f1e43d1551d6406cd93483e1884373b6112157e89543841d58c` |
| `1f_map_f` | `6009e9fc529e2807a64ce19be1b0bfafc5ac655f1819305dd9675b3c97028b57` |

같은 방향 기준으로 다른 셀은 25개다. active라는 사실만으로 일치 처리하지 않는다.

수정 후 `map_matches_fms=false`이고 진단 문장은 다음이다.

`map_server yaml file is missing: .../1f_map.yaml; FMS map is .../1f_map_f.yaml; loaded occupancy sha256 52a69842d7e6 differs from FMS map sha256 6009e9fc529e`

Nav2 READY는 그대로다. 5cm 버튼은 이 문장 때문에 비활성화된다. map_server, AMCL, TF는 재시작하지 않았다. 지도를 바꾸려면 운영자가 별도로 lifecycle과 진행 중인 Goal을 확인한 뒤 `1f_map_f.yaml`로 map_server를 다시 띄우고 AMCL과 `map → base_footprint`를 재확인해야 한다.

## 수정 파일

- `backend/app/services/observation_gate.py`
- `backend/app/services/map_identity.py`
- `backend/app/services/map_identity_probe.py`
- `backend/app/services/physical_nav2_auto.py`
- `backend/app/services/forward_5cm_service.py`
- `backend/app/api/physical.py`
- `backend/app/services/fleet_registry.py`
- `frontend/src/components/fleet-console.tsx`
- `backend/tests/test_observation_and_map_identity.py`
- `backend/tests/test_static_isolation.py`

Collision Monitor와 StopZone 판정은 변경하지 않았다.

## 테스트

ROS 환경 변수를 내린 backend pytest는 통과했다. 추가 6개를 포함해 진행 표시 123개다. `tsc --noEmit`과 `FMS_NEXT_DIST_DIR=.next-verify` 프로덕션 빌드도 통과했다.

관제 화면에서 Nav2 READY, 실시간 TF `fresh`, 지도 불일치, `5cm 시작 불가`가 같이 보였고 5cm 버튼은 비활성화 상태였다. 그 버튼은 누르지 않았다.
