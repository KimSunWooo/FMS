# Phase 3 준비도

다중 로봇 주행은 하지 않았다. Pinky 3대를 스폰하지 않았다. 아래는 현재 프로세스와 런치 파일을 읽은 조사다.

Phase 2.2의 계획된 단일 로봇 5회는 0/5 PASS다. 그 상태로 다중 로봇 주행에 들어가지 않는다.

## 조사

| 항목 | 현재 | 이번 단계 |
| --- | --- | --- |
| Pinky 3대 Gazebo 스폰 | 실행 중인 엔티티는 `pinky` 하나다. `launch_sim.launch.xml`은 `namespace` 기본값이 빈 문자열이고 로봇 이름은 `$(namespace)pinky`다 | NOT_RUN |
| 로봇별 namespace | 라이브 그래프의 Nav2 노드는 `/amcl`, `/bt_navigator`, `/controller_server`, `/planner_server`, `/map_server`, `/velocity_smoother`, `/nav2_container`처럼 전역이다. 네임스페이스 인자는 업스트림 런치에 있다 | 분리되지 않음 |
| TF | 관측에 쓰인 프레임은 `map` → `base_footprint` 하나다. 접두사가 없다 | 분리되지 않음 |
| Nav2 action server | `/navigate_to_pose`가 하나다. 에이전트는 이 절대 이름만 호출한다 | 분리되지 않음 |
| Robot agent 프로세스 | 로봇 하나당 프로세스 하나는 이미 그렇다. 두 번째 세션은 같은 `robot_id`에서 409다 | 1대만 실행 |
| FMS robot registry | `config/fleet.json`에는 mock용 `pinky-1`, `pinky-2`, `pinky-3`가 다른 지도 좌표에 있다. 이번 Gazebo DB는 `FMS_SEED_FLEET=false`라 등록 로봇은 `pinky-sim` 하나다 | 3대 Gazebo 등록은 NOT_RUN |
| Task 자동 배정 | 스케줄러는 신뢰할 수 있는 pose와 살아있는 세션이 있는 로봇에 PENDING을 배정한다. 로봇당 점유 task는 하나다 | 단일 로봇에서만 확인 |
| 세션과 goal 소유권 | session id, owner epoch, lease, execution id, FMS goal id로 한 로봇의 결과를 가른다. 다른 로봇의 세션 격리는 실측하지 않았다 | 1대 실측 |
| 구역 점유 | `zones.json`은 12 m × 8 m 바닥이고 용량은 1이다. 공장 월드(벽 x=±2, y=±1.5)와 같은 지도가 아니다. ZONE_E는 x 0.4–2.6, y 0.4–2.2다 | 두 로봇이 같은 구역에 들어가는 실험은 NOT_RUN |

## 막히는 지점

빈 네임스페이스로 두 번째 Pinky를 같은 그래프에 올리면 노드 이름, `/cmd_vel`, `/scan`, `/odom`, `/tf`가 첫 로봇과 겹친다. 브리지 파일 `pinky_bridge.yaml`의 ROS 토픽은 전역 이름이다. 이미 1대 그래프에서도 `/image_bridge` 이름이 중복된다는 경고가 있다.

FMS robot id `pinky-sim`은 ROS 이름으로 쓸 수 없다. 하이픈이 들어 있다. 이 id를 네임스페이스로 붙이면 cmd_vel이 브리지와 어긋난다.

에이전트와 FMS 클라이언트는 로봇별 action 이름을 받지 않는다. 로봇 두 대가 같은 `/navigate_to_pose`를 쓰면 goal 소유를 나눌 수 없다.

공장 맵과 zone 파일이 다르다. 용량 1인 구역은 로봇이 둘 이상이면 배정을 막을 수 있지만, 그 충돌은 이번 월드에서 재현하지 않았다.

## 결론

Phase 3의 다중 로봇 주행 진입 조건은 충족되지 않았다. 네임스페이스, TF 접두사, 브리지, Nav2 action server, 에이전트 action 이름을 로봇마다 나누는 작업이 남아 있다. 그 작업은 이 단계에서 시작하지 않았다.
