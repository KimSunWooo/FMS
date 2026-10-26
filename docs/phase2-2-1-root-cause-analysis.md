# Phase 2.2.1 원인 분석

시각은 KST(UTC+9)다. 로그 원문의 타임스탬프는 UTC다. 기존 `docs/phase2-2-logs/` 와 Phase 2.2 보고서는 덮어쓰지 않았다.

## 결론

Nav2 `SUCCEEDED` 와 FMS 미완료는 서로 다른 도착 기준이 겹친 결과다. FMS는 요청 좌표와 `map` → `base_footprint` 거리 0.25 m를 본다. 제조사 Nav2는 그 좌표가 아니라, 플래너가 바꿔 놓은 경로 끝점을 `SimpleGoalChecker`로 본다.

두 가지가 동시에 있었다.

1. 벽 근처 목표는 라이브 costmap에서 중심이 lethal이라 Navfn이 경로를 다른 칸에서 끝낸다. 제조사 `GridBased.tolerance` 0.5 m는 그 대체 점을 요청 좌표에서 최대 0.5 m까지 허용한다. checker의 xy 0.25 m는 그 대체 점 기준이다.
2. 빈 칸에서는 경로 끝이 요청 좌표와 같다. `stateful: true` 인 checker는 xy 안에 한 번 들어가면 그 뒤 xy를 다시 보지 않는다. 제자리 회전이 몸체를 0.25 m 밖으로 밀어도 Nav2는 성공을 유지할 수 있다.

TF 프레임이 어긋나서 거리가 달라진 증거는 없다. 정지 뒤 2초 관측이 합격 자세를 밖으로 밀어낸 증거도 없다.

FMS `pose_tolerance_m` 0.25 m는 바꾸지 않았다.

## GoalChecker

확인한 플러그인은 `nav2_controller::SimpleGoalChecker`, 이름 `general_goal_checker`다. Jazzy `isGoalReached`는 `check_xy_`가 참이고 평면 거리가 `xy_goal_tolerance`보다 크면 실패를 반환한다. 안에 들어오고 `stateful`이면 `check_xy_`를 거짓으로 두고, 이후에는 yaw만 본다. `reset()`이 `check_xy_`를 되돌린다.

수정 전 라이브 파라미터:

| 항목 | 값 |
| --- | --- |
| stateful | true |
| xy_goal_tolerance | 0.25 m |
| yaw_goal_tolerance | 0.25 rad |
| controller_frequency | 20 Hz |
| FollowPath.use_rotate_to_heading | true |
| rotate_to_heading_angular_vel | 1.0 rad/s |
| min_approach_linear_velocity | 0.05 m/s |
| GridBased.tolerance | 0.5 m |

ControllerServer는 local costmap 자세(`odom`, `base_footprint`)와 전역 경로의 마지막 점을 비교한다. 같은 시각의 강체 TF라면 odom 거리와 map 거리는 같다. FMS 거리가 checker보다 크다는 것은 경로 끝이 요청 좌표가 아니거나, 샘플 시각이 다르다는 뜻이다.

Phase 2.2 1회 최종 자세는 map `(-1.000, -0.012)`다. 요청 `(-0.8, 0.2)`까지 0.291 m이고, 당시 경로 끝 `(-0.90, 0.20)`까지는 약 0.23 m다. Nav2 status 4는 대체 점의 0.25 m 안쪽과 맞다. 요청 좌표의 0.25 m 안쪽에 들어간 기록은 없다.

2·3·4회는 경로 끝이 요청 좌표와 같았고 최종 오차는 0.255 m, 0.292 m, 0.261 m였다. checker 경계에서 성공한 뒤 회전으로 몸이 벗어난 경우와 맞다. 5회 0.358 m는 1회와 같이 대체 점 성공이다. 당시 경로 끝은 `(-0.45, -0.40)`으로 요청 `(-0.6, -0.4)`에서 0.15 m였다.

오버레이를 켠 뒤 `ComputePathToPose`를 다시 호출했다. `(-0.8, 0.2)`의 경로 끝은 여전히 `(-0.90, 0.20)`, 오차 0.10 m, pose 11개다. tolerance를 0.15 m로 줄여도 0.10 m 대체는 남는다. `(-0.6, -0.4)`는 pose 0개, action status 6이다. 0.15 m 안에 유효한 중심 칸이 없어 플래너가 경로를 거절한다.

## TF와 위치 추정

오버레이 적용 후 같은 시각의 변환:

| 변환 | x, y, z |
| --- | --- |
| map → base_footprint | -1.026, -0.011, 0 |
| map → base_link | -1.026, -0.011, 0.028 |

XY는 같고 z만 바퀴 반경이다. `bt_navigator`의 `base_link`와 costmap의 `base_footprint`는 이 정적 오프셋 안에서 평면 거리가 같다. FMS 완료는 `map` / `base_footprint`만 쓴다.

odom 원점은 map 원점과 다르다. 예로 같은 순간의 odom → base_footprint는 `(-1.093, 0.239)`였다. FMS 거리 식에 odom을 넣지 않는다. AMCL `update_min_d` 0.15 m는 map 추정을 늦출 수 있으나, 이번 완료 판정은 Nav2 결과 뒤 최신 map TF다.

지도 원점 `[-2.05, -1.62]`, 해상도 0.05 m는 공장 벽 x=±2, y=±1.5와 맞다. 정적 PGM에서 실패 좌표의 픽셀은 자유 칸이었다. lethal은 라이브 inflation이다. 각인 반경 안의 칸은 cost 253/254가 되고 Navfn은 그 칸을 목표 중심으로 쓰지 않는다.

Ground truth로 완료를 판정하지 않았다.

## 정지

12회 모두 결과 뒤 재관측에서 `/odom` twist는 `vx=0`, `wz=0`이다. 점수에 쓴 거리와 정지 후 거리의 차는 1 cm 안쪽이다. 2초 settle이 합격 자세를 불합격으로 바꾼 회차는 없다.

접근 속도 0.05 m/s와 제자리 회전 1.0 rad/s는 경계 성공 뒤 위치를 밀어낼 수 있다. 오버레이는 접근 0.03 m/s, 회전 0.5 rad/s로 낮췄다. 회전 중 이탈을 없앴다고 단정하지 않고, stateful을 끄고 xy 창을 0.12 m로 줄여 이탈이 성공으로 남지 않게 했다.

## 수정 후 같은 좌표

| 좌표 | Phase 2.2 | 이번 재실행 |
| --- | --- | --- |
| (-0.8, 0.2) | Nav2 4, 0.291 m, 미완료 | Nav2 4, 0.177 m, COMPLETED. 경로 끝 오차 0.10 m |
| (0.2, -0.8) | Nav2 4, 0.255 m, 미완료 | Nav2 4, 0.130 m, COMPLETED. 경로 끝 오차 0 |
| (-0.2, 0.8) | Nav2 4, 0.292 m, 미완료 | Nav2 4, 0.109 m, COMPLETED. 경로 끝 오차 0 |
| (0.8, -0.3) | Nav2 4, 0.261 m, 미완료 | Nav2 4, 0.133 m, COMPLETED. 경로 끝 오차 0 |
| (-0.6, -0.4) | Nav2 4, 0.358 m, 미완료 | Nav2 6, 플래너 pose 0, FMS FAILED. 로봇은 출발점 근처에 정지 |

5회차의 Nav2 성공이 사라진 것은 오차를 숨긴 것이 아니다. 0.15 m 안에 설 칸이 없어 경로가 없고, FMS는 `nav2_goal_aborted`로 `FAILED`다. `COMPLETED`가 아니다. 이 좌표를 다시 보내 성공으로 바꾸지 않았다.
