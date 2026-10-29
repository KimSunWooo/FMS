# Pinky-1 5cm 주행 진단

작성 시각: 2026-10-28

실물 주행 POST는 호출하지 않았다. 로봇이 이동하지 않았으므로 성공이 아니다. 결과는 **BLOCKED**다.

## 기존 문제

이전 시험에서는 DriveOnHeading Goal이 수락된 뒤 `/cmd_vel_nav`와 `/cmd_vel_smoothed`의 `linear.x`가 약 0.04였고 `/cmd_vel`은 관측되지 않았다. Action status 6은 `ABORTED`이고 측정 이동은 0.0 m였다. Collision Monitor 로그에는 StopZone 정지와 invalid source가 있었다.

그 뒤 FMS는 센서 좌표계에서 `range < 0.18 m`인 반환이 46~49개라는 이유로 Goal 전송을 막았다. Collision Monitor의 StopZone은 `base_footprint`에서 평가하므로, 그 개수가 실제 정지 판정과 같은지는 이번 진단에서 다시 계산했다.

## LiDAR와 TF

무주행 구독 진단이다. `ROS_DOMAIN_ID=24`. 속도 명령은 발행하지 않았다.

| 항목 | 관측 |
|---|---|
| `/scan` frame | `rplidar_link` |
| TF | `base_footprint` ← `rplidar_link`를 scan stamp로 변환 성공 |
| LiDAR 위치 | x=-0.017 m, y=0, z=0.125 m, yaw=π |
| `range_min` / `range_max` | 0.05 m / 40 m |

LiDAR +x는 로봇 후방을 본다. 변환 실패가 아니므로 안전하다고 추정해서 통과시키지 않은 것이 아니라, 변환된 좌표로 다시 센 것이다.

## StopZone

Nav2 Collision Monitor 설정은 유지했다.

- `base_frame_id: base_footprint`
- StopZone circle, 반경 0.18 m, `min_points: 4`, action `stop`
- 입력 `/cmd_vel_smoothed`, 출력 `/cmd_vel`
- lifecycle `active [3]`

같은 scan을 두 기준으로 세었다.

실행 중인 API의 읽기 전용 진단 `GET /api/physical/tests/pinky-1/forward-5cm/diagnosis` 결과다. `motion_command_sent`는 false다.

| 기준 | 0.18 m 안 점수 |
|---|---|
| 센서 range만 비교 | 50 |
| `base_footprint` 원점까지 거리 | 50 |
| range가 0.05~40 m이면서 base 원 안 | 50 |
| `range_min`보다 짧은 반환 | 0 |

API를 재시작한 뒤 같은 진단은 base 원 안 49개, 전방 0개, `transform_status=ok`, `motion_command_sent=false`였다. 그 직전 표본은 base 원 안 50개, 최근접 0.104 m, 오른쪽 50개였다. 더 앞의 표본은 base 원 안 51개, 유효 range 49개, 최근접 0.034 m, 오른쪽 49개, 왼쪽 2개였다. 세 표본 모두 전방은 비어 있고 오른쪽 군집이 `min_points` 4를 넘는다. StopZone은 원 전체이므로 Collision Monitor는 전진 명령도 멈춘다.

센서 좌표 계산이 점 개수를 부풀린 것이 아니다. 변환 후에도 정지 조건이 남는다. 반경을 줄이거나 오른쪽 반환을 무시하지 않았다. 3 cm대의 반환이 로봇 브래킷인지 맞닿은 물체인지는 이 표본만으로 구분하지 않는다. 둘 다 StopZone 안이므로 필터로 지우면 안 된다.

과거 `/cmd_vel` 공백과 status 6은 이 정지와 맞다. 이번 턴에는 Goal을 보내지 않아 그 순간의 세 토픽을 다시 동시에 기록하지는 못했다. invalid source는 이번 신선한 scan에서는 재현되지 않았다.

## Nav2와 초기 위치

저장 `nav2_params.yaml`의 AMCL 초기 위치는 x=0.11, y=2.527, yaw=-3.052이고 `set_initial_pose: true`다.

실행 중인 AMCL 프로세스는 `set_initial_pose:=false`로 떠 있다. 라이브 파라미터 `initial_pose.x/y/yaw`는 0.0이다. FMS `physical_nav2_auto.py`도 map_server와 AMCL을 띄울 때 `set_initial_pose:=false`를 넘기고, 초기 위치는 별도 `/set_initial_pose`로만 보낸다. 목표 좌표는 코드의 `initial_pose_target` x=1.696, y=0.103이다. YAML 초기 위치가 지금 프로세스에 다시 적용되고 있지는 않다. 이번 진단에서 그 값을 바꾸지 않았다.

실행 중인 map_server 인자는 `config/maps/physical_site/1f_map.yaml`이다. 그 파일은 작업 트리에 없고, FMS 설정과 플래너는 `1f_map_f.yaml`을 쓴다. 지도 서버를 재시작하지 않았다. 운영자가 Nav2를 다시 띄우기 전에는 이미 올라간 map_server가 옛 경로를 가리킨 채로 남을 수 있다.

5cm 코드의 목표 거리 0.05 m, 속도 0.04 m/s, time allowance 10초, action `/drive_on_heading`은 바꾸지 않았다. 성공으로 보려면 map TF 전진과 odometry 전진이 모두 0.035 m 이상 0.10 m 이하이고, 횡방향 편차가 0.04 m 이하여야 한다. ±1 cm로 좁히지 않았다. 새 실측이 없어 하드웨어 오차를 더 줄일 근거가 없다.

## 수정한 파일

- `backend/app/services/nav2_chain_probe.py` — scan stamp의 TF로 `base_footprint` 점 개수와 센서 range 개수를 구분. 변환 실패는 진단 실패
- `backend/app/services/velocity_chain.py` — 변환 실패면 Goal을 보내지 않음
- `backend/tests/test_fleet_task_cancel.py` — 변환 실패가 통과로 처리되지 않는지 확인

Collision Monitor, StopZone 반경, 속도, 제한 시간은 유지했다.

## 수정 전후

수정 전 차단 이유는 센서 range 46~49개였다. 수정 후 변환은 성공했고, 라이브 진단은 base 원 안 50개, 전방 0개다. 차단은 유지된다. 좌표를 고친 결과가 구역이 비었다는 뜻은 아니다.

## 실주행

**BLOCKED.** 운영자가 시작하지 않았고, 현재 StopZone은 정지 조건이다. FMS `start()`는 이 진단에서 DriveOnHeading을 보내지 않는다. 전방이 비고 원 안의 반환이 4개 미만이 된 뒤, 현장 확인과 명시적 시작이 있어야 주행 시험이다.
