# Map Server 지도 정합성 복구

작성 시각: 2026-10-28

5cm POST와 Nav2 Goal은 호출하지 않았다. AMCL 초기 위치도 바꾸지 않았다. map_server 프로세스는 종료하지 않았다.

## 복구 전 환경

| 항목 | 값 |
|---|---|
| map_server PID | 590031, 부모 590028 |
| 시작 | 2026-10-28 12:14:21, 사용자 `sw` |
| 프로세스 소유 | Terminator의 bash. FMS 자식이 아님. `launch_owned_by_fms=false` |
| 시작 인자 | `yaml_filename:=.../1f_map.yaml` |
| Lifecycle | map_server, amcl, collision_monitor 모두 `active [3]` |
| 실행 중 Goal | 없음. navigate, drive_on_heading, follow_path의 상태 목록에는 1, 2, 5가 없고 종료 상태만 있었다 |

## 25개 셀은 변환 규칙 차이가 아니다

두 YAML 모두 `mode: trinary`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.196`, 해상도 0.01, origin 0, 186×236이다.

FMS와 Nav2는 같은 trinary 규칙을 쓴다. 픽셀 0은 점유, 255는 자유다. 행 방향도 같다. 뒤집어서 비교하면 889칸이 달랐고, 같은 방향에서는 25칸만 달랐다.

그 25칸은 grid row 92, col 110부터 134까지 한 줄이다. 지도 좌표는 x 1.105–1.345 m, y 0.925 m다.

| 지도 | 그 25픽셀 |
|---|---|
| 커밋 `cb93643`의 `1f_map.pgm` | 255, Nav2에서는 free 0 |
| 현재 `1f_map_f.pgm` | 0, FMS에서는 occupied |

로드되어 있던 `/map`은 옛 PGM과 같았고 FMS는 새 PGM을 읽고 있었다. 해시가 다른 이유는 점유 변환이 달라서가 아니다.

## 재로딩

`/map_server/load_map` (`nav2_msgs/srv/LoadMap`)을 사용했다. 진행 중인 Goal이 없는 것을 확인한 뒤 호출했다.

`file:///.../1f_map_f.yaml`은 result 3 `RESULT_INVALID_MAP_METADATA`였고 `/map`을 바꾸지 않았다. Jazzy `MapServer::loadMapCallback`은 URL을 그대로 `loadMapFromYaml`에 넘긴다.

파일 경로 `/home/sw/DevSpace/FMS/config/maps/physical_site/1f_map_f.yaml`은 result 0이었다. 이후 `/map` occupancy sha256은 FMS와 같다.

`6009e9fc529e2807a64ce19be1b0bfafc5ac655f1819305dd9675b3c97028b57`

row 92의 25칸은 Nav2에서 100, 즉 점유다. `/global_costmap/costmap`과 `/global_costmap/static_layer`도 같은 칸이 100이다.

`LoadMap`은 `yaml_filename` 파라미터를 바꾸지 않는다. 로드가 성공한 뒤에만 그 파라미터를 `1f_map_f.yaml`로 설정했다. 프로세스 인자 문자열은 시작 당시의 `1f_map.yaml`이 남아 있다. PID 590031은 그대로다. 라이브 파라미터와 `/map` 내용이 현재 기준이다.

`map → base_footprint` TF는 로드 뒤에도 있었다. 관측 예는 x=1.704 m, y=0.125 m다. 초기 위치 서비스는 호출하지 않았다.

## 같은 시점의 FMS 상태

API를 재시작한 뒤 한 스냅샷이다.

| 필드 | 값 |
|---|---|
| nav2_ready | true |
| map_matches_fms | true |
| map_server_active | true |
| amcl_active | true |
| initial_pose_confirmed | true |
| tf_ready | true |
| collision_monitor_active | true |
| live_tf_status | fresh |
| live_tf_withheld_reason | 빈 문자열 |
| live_tf_age_s | 0.35 |
| checked_at | 2026-10-28T12:38:35.563926+00:00 |
| pose_checked_at | 2026-10-28T12:38:36.311181+00:00 |
| five_cm_blockers | 없음 |
| launch_owned_by_fms | false |

재시작 전 1초 간격 5회도 모두 `fresh`였고 `map_matches_fms`는 true였다. 이번 관측 구간에는 `withheld`가 없었다.

`withheld`는 실시간 TF 시각이 4초 안으로 유효한데 `ssh_connected`, `amcl_active`, `initial_pose_confirmed` 중 하나가 false일 때다. 그 경우 pose는 비우고, `live_tf_withheld_reason`에 false인 필드 이름을 적는다. 폴 pose로 대체하지 않는다.

## 5cm

지도와 TF 조건은 통과했다. 읽기 전용 진단은 여전히 StopZone 정지다.

- `motion_command_sent`: false
- `stop_zone_would_stop`: true
- base 원 안 50점, 유효 range 46점, 오른쪽 46, 왼쪽 4, 전방 0
- 최근접 0.029 m
- transform ok

시작 POST는 호출하지 않았다. 오른쪽 0.18 m 반환이 남아 있으므로 서버는 이 상태에서 Goal을 보내지 않는다. Nav2 READY와 5cm 주행 가능은 다르다. READY와 지도 일치는 버튼 조건이고, StopZone은 시작 요청에서 따로 막는다.

## 테스트

ROS 변수를 내린 backend pytest는 통과했다. withheld 원인 테스트를 포함해 124개다. frontend `tsc --noEmit`과 `FMS_NEXT_DIST_DIR=.next-verify` 빌드도 통과했다.

## 수정 파일

- `backend/app/services/physical_nav2_auto.py` — withheld 원인, collision monitor 상태
- `backend/app/services/observation_gate.py`
- `backend/app/api/physical.py`
- `frontend/src/components/fleet-console.tsx`
- `backend/tests/test_observation_and_map_identity.py`
