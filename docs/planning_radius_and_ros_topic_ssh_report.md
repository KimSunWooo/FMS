# 계획 반경·격자 통일 + 위치/SSH 분리 보고

날짜: 2026-10-30  
범위: `/home/sw/DevSpace/FMS` (Care-Move/Open-RMF와 설정·데이터 비공유)  
실물 이동 명령·Nav2 Goal·`/cmd_vel` 미발행. 커밋/push/배포 없음.

## 1. 본체 치수와 R_effective

근거: `simulation/ros_ws/src/pinky_pro/pinky_navigation/params/nav2_params.yaml`  
local/global costmap footprint (base_footprint):

`[[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]`

| 항목 | 값 |
|---|---|
| 본체 (사각형) | length = width = **0.12 m** |
| R_body | `sqrt(0.12²+0.12²)/2` = **0.08485281374238571 m** |
| 여유 | **0.01 m** (한 번만) |
| R_effective | **0.0948528137423857 m** |
| 계획 격자 | **planning_resolution_m = R_effective** |
| 로봇 간 최소 중심거리 | **2 × R_effective** |

비교(적용하지 않음):

- base_link STL + 구동륜 최대 XY 반경 ≈ 0.08257 m (Nav2 외접반경보다 작음)
- 주석 처리된 `robot_radius: 0.22` — 사용 안 함
- Nav2 `inflation_radius: 0.15`, `footprint_padding: 0.03`, Collision Monitor — **축소/비활성화하지 않음**

이전 기본값 `0.14 / 0.096`은 주석 가정(110×120+여유)이었고, 격자 반올림으로 반경 0.096 → 실제 stride 0.10 m로 조용히 달라질 수 있었음.

## 2. 격자 stride 수정

`WaypointGraph`는 더 이상 `round(resolution/map_res)*map_res`로 요청값을 덮어쓰지 않는다.  
월드 미터 간격 = 요청 `planning_resolution_m` 그대로.  
직교 간격 = R_effective, 대각 = R_effective·√2.

프로파일: `lattice-0.095-r0.095`  
이전 `lattice-0.140-r0.096` 경로는 `needs_replan=true`이며 dispatch 거부.

## 3. 위치 경로

우선순위(물리): `/amcl_pose` IPC → RobotStatus TF → worker TF 진단.  
Registry home / (1.696, 0.103) 고정점은 실물 현재 위치로 쓰지 않음.

- `physical_initial_pose_auto` 기본 **false**
- API 재시작이 유효 TF가 있을 때 `/set_initial_pose`로 덮어쓰지 않음
- 운영자 전용: `POST /api/physical/robots/pinky-1/initial-pose` (`confirm=true`, 선택 `force`)

## 4. SSH 의존 제거 (`fleet_dispatch_mode=ros_topic`)

수정된 간접 차단:

- `physical_nav2_auto.poll` SSH 실패 early return 제거
- snapshot이 SSH/`initial_pose_confirmed`로 fresh TF withhold 하던 동작 제거
- `fresh_tf_pose` / `_fresh_nav2` / `present_robot` SSH 게이트 제거
- `/robots` Nav2 snapshot을 SSH 성공에 묶지 않음

유지: Domain·robot_id·ACK hash·timeout·`execute=false`·명시 `ssh` 모드 인증.

## 5. 적용값 (.env / 프로세스)

| 키 | `.env` | 프로세스 env | Settings 적용 |
|---|---|---|---|
| `FMS_PLANNING_RESOLUTION_M` | 없음 | 없음 | R_effective |
| `FMS_PLANNING_ROBOT_RADIUS_M` | 없음 | 없음 | R_effective |
| `FMS_PHYSICAL_INITIAL_POSE_AUTO` | 없음 | 없음 | false |
| `FMS_FLEET_DISPATCH_MODE` | (example ros_topic) | — | ros_topic |

## 6. 검증

**CODE_PASS**

- 동일 반경: 시작/목적지/구간/예약 conflict
- 격자 간격 = 요청값
- stale 프로파일 dispatch 거부
- SSH 없이 fresh TF 노출
- 관련 pytest (lattice + ros_topic_no_ssh_gate + 회귀)

**LIVE / 미검증**

- Domain 24에서 `/amcl_pose`·Nav2·battery 실발행 여부는 이전과 같이 기체 상태에 의존
- 실물 위치 **정확도**는 이동 시험 없이 미검증
- SSH 실패+ROS 정상 E2E Task ACK는 Gateway/Receiver가 Domain에서 떠 있을 때만 LIVE 확인 가능

## 7. 실행 명령

swPC (API):

```bash
cd /home/sw/DevSpace/FMS
source .venv/bin/activate
# FMS_PHYSICAL_INITIAL_POSE_AUTO 는 기본 false — 고정 좌표 자동 전송 없음
uvicorn app.main:create_app --factory --host 0.0.0.0 --port 8741 --app-dir backend
```

Pinky (Domain 24, 수신 전용 Receiver — 배포는 운영자 승인 후):

```bash
export ROS_DOMAIN_ID=24
# AMCL/Nav2/battery_publisher 는 제조사 bringup
# FMS Receiver: /fms/task/* ACK, execute=false
```

운영자 초기 위치(필요할 때만):

```bash
curl -sS -X POST http://127.0.0.1:8741/api/physical/robots/pinky-1/initial-pose \
  -H 'content-type: application/json' \
  -d '{"x":1.696,"y":0.103,"confirm":true,"force":false}'
```
