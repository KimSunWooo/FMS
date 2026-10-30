# 초기 위치 Topic + Nav2 기동 순서 복구 보고

날짜: 2026-10-30  
기준: `/home/sw/DevSpace/FMS` 미커밋 로컬 코드 유지 · 커밋/push/배포 없음 · `/cmd_vel`/Nav2 Goal/5cm 미실행

## 1. 원인과 근거

| 증상 | 코드·로그 근거 |
|---|---|
| AMCL active인데 map TF 없음 | `logs/physical-nav2-amcl.log`: “Please set the initial pose…” 반복 |
| Nav2 Aborting bringup | `lifecycle_manager`가 map TF 없이 global_costmap activate 시도 |
| 순환 대기 | 기존 코드가 Nav2를 먼저 올린 뒤 TF를 기다림. TF는 초기 위치 이후에만 생김 |
| active에 configure 재요청 | `Unable to start transition 1 from current state active` — `_ensure_lifecycle`이 상태 재확인 없이 transition |
| 잘못된 시드 좌표 | 코드가 `(1.696, 0.103)` 사용. 등록값은 `nav2_params.yaml` `initial_pose: x=0.11 y=2.527 yaw=-3.052` |
| 자동 초기화 꺼짐 | `physical_initial_pose_auto=false` + awaiting_operator → Nav2는 그래도 launch |

## 2. 기동 상태 흐름 (수정 후)

```
sensors (/odom,/scan)
  → map_server active + map hash match
  → amcl active (이미 active면 transition 생략)
  → localization?
       yes → localization_ready
       no + auto + registered → /initialpose 1회 pub → sent_waiting_for_observation
       no + !auto → waiting_for_initial_pose (Nav2 미기동)
  → (관측 확인 후에만) Nav2 launch 또는 inactive 재활성화
  → cmd_vel chain → ready
```

Lifecycle: map_server/amcl은 FMS가 관리. Nav2 서버는 `lifecycle_manager_navigation`(launch)이 소유. Abort 후에는 위치 확인된 경우에만 inactive 노드 activate 재개(상한 있음).

## 3. 적용 초기 위치·자동화

| 항목 | 값 |
|---|---|
| Topic | `/initialpose` `geometry_msgs/PoseWithCovarianceStamped` |
| 서비스 | `/set_initial_pose` **미사용**(단일 경로) |
| 등록 좌표 | nav2_params: **(0.11, 2.527, yaw=-3.052 rad)** |
| 출처 | `nav2_params:nav2_params.yaml` |
| `.env` | `FMS_PHYSICAL_INITIAL_POSE_AUTO=true` |
| 세션 파일 | `data/ros_ipc/pinky-1/initial_pose_session.json` (재시작 중복 전송 완화) |

`initial_pose_sent` ≠ `initial_pose_confirmed`. 확인은 시드 이후 `/amcl_pose` 또는 map→base_footprint 신규 관측만.

## 4. 변경 파일

- `backend/app/services/physical_nav2_auto.py` — 순서·lifecycle·`/initialpose`
- `backend/app/services/initial_pose_registry.py` — 등록값 해석
- `backend/app/services/initial_pose_session.py` — 시드 세션
- `backend/app/core/config.py` — auto/topic/max/retry/nav2_params 경로
- `backend/app/api/physical.py` · `fleet_registry.py` · frontend 상태 표시
- `backend/tests/test_initial_pose_bringup.py`
- `.env` / `.env.example`

## 5. 자동 테스트

`test_initial_pose_bringup.py` + 관련 스위트: 등록 좌표, active 재전이 방지, 기존 위치 시 미전송, map mismatch 거절, Topic-only pub, 대기 중 중복 방지, max attempts, localization 전 Nav2 미기동, SSH 비필수.

전체 `pytest tests/` — 이 작업 후 재실행으로 확인.

## 6. 실제 ROS

**확인(로그·설정):** AMCL need-init 경고, Aborting bringup, active 재전이 오류, 등록 pose 값.

**LIVE Domain 24 (작업 중 재확인):**
- `/amcl` `/map_server` `/bt_navigator` `/controller_server` `/planner_server`: **active**
- `/amcl_pose` Publisher 1 · 샘플 `(≈1.55, 0.17)` frame=`map` (등록 시드 0.11/2.527과 다름 → **이미 다른 경로로 국소화됨**)
- `map→base_footprint` TF **수신**
- `/initialpose` Subscriber 1 / Publisher 0 → 현재는 기존 위치가 있어 **자동 재전송이 불필요한 상태**(코드 규칙과 일치)
- `/scan` ~10 Hz, `/odom` ~30 Hz
- `collision_monitor` lifecycle 조회 응답 공란 → 추가 점검 권장
- AMCL stamp `sec≈1793328980` vs swPC wall UTC ~2026 → **시계 불일치 후보**. swPC는 NTP sync yes. Pinky chrony 대조·queue/tolerance 확대는 하지 않음.

**미검증:** 이번 코드 경로로 cold-start `/initialpose` 1회 발행→관측→Nav2 재개 E2E(이미 국소화돼 자동 시드 생략). 위치 **정확도**·주행 미검증.

**Receiver/배터리:** 위치·Nav2와 분리. ACK 성공으로 보고하지 않음.

## 7. 실행·확인 명령

swPC:

```bash
cd /home/sw/DevSpace/FMS && source .venv/bin/activate
# .env: FMS_PHYSICAL_INITIAL_POSE_AUTO=true
uvicorn app.main:create_app --factory --host 0.0.0.0 --port 8741 --app-dir backend
curl -sS http://127.0.0.1:8741/api/physical/nav2/status | jq '{state,bringup_phase,initial_pose_state,initial_pose_confirmed,localization_ready,map_matches_fms}'
```

Pinky (Domain 24):

```bash
export ROS_DOMAIN_ID=24
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
# 수동 시드(필요 시)
curl -sS -X POST http://127.0.0.1:8741/api/physical/robots/pinky-1/initial-pose \
  -H 'content-type: application/json' \
  -d '{"x":0.11,"y":2.527,"yaw_deg":-174.87,"confirm":true,"force":false}'
```

## 8. 복구 절차(기존 작업 유지)

1. 미커밋 반경/SSH 분리 변경 유지한 채 API 재시작
2. Pinky bringup으로 `/odom` `/scan` 확인
3. map_server/AMCL active + 지도 해시 일치 확인
4. auto면 `/initialpose` 1회 → `/amcl_pose`/TF 관측
5. 그 다음 Nav2 inactive 재개 또는 launch
6. UI에서 `sent` vs `confirmed`·`bringup_phase` 구분 확인
