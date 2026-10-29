# FMS 공식 지도 표시 복구

2026-10-29. 관제 화면이 비어 있던 이유는 지도 파일이 바뀐 것이 아니다. 실행 중인 API 프로세스가 삭제된 `1f_map.yaml`을 환경 변수로 들고 있었다.

## 실제 오류 원인

`load_geometry()`는 `Settings.physical_map_yaml`을 읽는다. 코드 기본값과 `.env`의 `FMS_PHYSICAL_MAP_YAML`은 둘 다 다음 파일이다.

```text
config/maps/physical_site/1f_map_f.yaml
```

pydantic-settings는 프로세스 환경 변수를 `.env`보다 먼저 쓴다. 오늘 10:38에 시작된 uvicorn(PID 2450178)의 환경에는 다음 값이 있었다.

```text
FMS_PHYSICAL_MAP_YAML=/home/sw/DevSpace/FMS/config/maps/physical_site/1f_map.yaml
```

그 파일은 없다. `GET /api/physical/map`은 503과 `Actual map YAML unavailable`을 돌려줬고, Fleet Console은 지도 메타데이터가 없으면 SVG를 그리지 않았다. Next.js rewrite와 `map-frame.ts`는 이 경로를 만들지 않았다.

부모 셸에는 그 변수가 없었다. 자식 프로세스가 기동 시점에 받은 값이 남아 있었다. `.env`를 고쳐도 이미 떠 있는 프로세스에는 적용되지 않는다.

## 수정

없는 지도 경로를 다른 파일로 바꾸지 않았다.

- `scripts/run_api.sh`는 `FMS_PHYSICAL_MAP_YAML`이 가리키는 파일이 없을 때만 그 프로세스 환경 값을 제거하고, `.env`가 공식 지도를 읽게 한다. 파일이 있는 다른 경로는 건드리지 않는다.
- `Settings`는 지도 파일이 없으면 기동을 거부한다. 메시지에 없는 경로와 공식 지도 `1f_map_f.yaml`을 적는다.
- 지도 API는 로봇 SSH, ROS Worker, TF와 분리되어 있다. 지도 파일이 없으면 503이고, 로봇 목록은 지도 실패만으로 비우지 않는다.

수정 후 API를 다시 띄웠다. 새 프로세스에는 `FMS_PHYSICAL_MAP_YAML`이 없고, 설정은 `.env`의 `1f_map_f.yaml`이다.

## API와 이미지

| 요청 | 결과 |
|---|---|
| `GET /api/physical/map` | 200, `source=1f_map_f.yaml`, 186×236, resolution 0.01, origin (0, 0, 0) |
| `GET /api/physical/map/image` | 200, `image/png`, PNG 시그니처, IHDR 186×236 |
| `GET /fms-api/physical/map` | 200, 같은 메타데이터 |
| `GET /fms-api/physical/map/image` | 200, 같은 PNG |

브라우저에서 `/fms-api/physical/map/image`를 다시 불러 natural size 186×236이었다.

## 화면

수정 전 현장 지도 영역은 `Actual map YAML unavailable: .../1f_map.yaml`이었다.

수정 후 카드 문구는 `1f_map_f.yaml · 1.86 × 2.36 m · 0.01 m/px · frame: map`이고, PGM에서 만든 PNG 벽이 보인다. Pinky #2·#3 가상 마커가 등록 위치에 있다. 지도 클릭은 X 0.650 m, Y 1.064 m, grid col 65.0, row 106.4를 채웠다. 기존 Task `e9afb44b-9c95-4fc3-bfc5-973b7f3797e9`의 경로 버튼은 polyline 3개와 waypoint를 지도에 그렸다. Task 전달 버튼은 누르지 않았다.

Pinky-1 실시간 TF 마커는 없다. `live_tf_status=lookup_failed`라서 마지막 위치를 현재 위치로 남기지 않았다. 5cm 버튼은 비활성이고 POST는 호출하지 않았다.

## Domain

재시작 뒤 Worker는 그대로다.

| Robot | Domain | 상태 |
|---|---:|---|
| pinky-1 | 24 | physical, ROS 노드 확인, TF lookup_failed, pose 없음 |
| pinky-2 | 25 | virtual, physical connection not configured, pose 없음 |
| pinky-3 | 26 | virtual, physical connection not configured, pose 없음 |

## 데이터와 테스트

재시작 전후 fleet task 22건, reservation 381건이다.

| 항목 | 결과 |
|---|---|
| Backend pytest | exit 0. 없는 `1f_map.yaml` 환경 변수는 거부하고 다른 지도로 바꾸지 않는 테스트 포함 |
| Frontend `tsc --noEmit` | exit 0 |
| 격리 Next.js 빌드 | exit 0. 이후 `tsconfig.json`의 `.next-verify` include는 제거 |

## 남은 문제

- Pinky-1 TF는 `lookup_failed`다. Nav2 모니터의 지도 일치 표시는 아직 `map identity not checked`다. 정적 지도 파일과 `/map` 점유 해시 검증과는 별개다. Map Server 프로세스는 재시작하지 않았다.
- 5cm는 시작 불가 상태다. StopZone 반경과 Collision Monitor는 바꾸지 않았다. 주행은 NOT RUN이다.
- 개발 화면의 Next.js 배지는 시각 문자열 hydration 경고다. 지도 요청 실패는 아니다.
