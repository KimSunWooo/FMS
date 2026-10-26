# Phase 2.1 남은 제약

단일 로봇 Gazebo E2E는 PASS다. 아래는 그 판정을 막지 못한 것들과, 아직 막혀 있는 것들이다.

## 이 단계에서 닫히지 않은 것

- 실물 Pinky, `/cmd_vel` 직접 발행, 승강기, 자동문. 등록과 기동 설정이 거부한다. 실물 런치는 실행하지 않았다.
- 라이브 Nav2 goal 취소, 라이브 goal 거부, 주행 도중 FMS 재시작, 주행 도중 에이전트 kill, stale epoch의 라이브 결과 제출, lease 만료 이후의 라이브 success. 해당 FMS 전이는 pytest MOCK_PASS다. 맵 밖 목표는 서버가 받은 뒤 abort였다. 거부와 같은 사건으로 적지 않는다.
- 다중 로봇 Gazebo, 로봇 3대 에이전트 프로세스 격리. Phase 3는 시작하지 않았다.
- `config/cyclonedds.fms.xml` 미사용. 격리는 localhost + 도메인 73이다. 현장 NIC와 도메인 번호가 같은지는 이 VM에서 확인하지 못했다.
- GPU 없음. 센서는 ogre2 소프트웨어 경로로 동작했다. 공장보다 큰 월드나 다수 로봇에서는 같은 호스트가 부족할 수 있다. 조사 시작 시 balloon가 10 GiB였고, Gazebo 중에 4 GiB가 풀렸다.
- Pinky 워크스페이스와 ROS 설치는 이 VM의 `/home/ubuntu/pinky_pro`, `/opt/ros/jazzy`에만 있다. 저장소 클론만으로 재현되지 않는다.
- 완료 허용 오차 0.25 m는 Nav2 `xy_goal_tolerance` 0.25 m와 같다. 첫 목표 (0, 0.8)은 Nav2 성공 직후 관측이 0.257 m라 `COMPLETED`가 되지 않았다. 허용치를 늘리지 않았다.
- Abort 주행의 DB에는 Nav2 UUID가 없다. 로그 JSON에는 `cd751524-b5f6-4277-9f13-3f5dee555b9a`가 있다. 코드는 그 이후에 result UUID를 저장하도록 고쳤고, 라이브 abort를 다시 돌리지는 않았다.
- 에이전트 stop은 pose를 `agent_lost`로 내린다. 다음 배정을 위해 관측 pose를 다시 심는 API는 없다. `mock-observe`는 mock 스냅샷이라 Gazebo 로봇에 쓰지 않았다.
- 관제 화면은 HTTP 200까지 확인했다. 브라우저에서 클릭 경로는 돌리지 않았다. 화면 코드는 바꾸지 않았다.

## 하지 않은 일

- Open-RMF 소스, DB, 프로세스 변경
- TurtleBot 등 다른 모델을 Pinky 주행으로 보고하기
- 종료를 확인하지 못한 goal의 자동 재전송
- Mock fixture UUID를 Nav2 UUID로 적기
