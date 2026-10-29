# FMS 저장소 정리

작성 시각: 2026-10-28

로컬 `main`은 `6e856c9`이고 `origin/main`과 앞뒤 커밋 차이는 0이었다. 정리는 작업 트리의 삭제로 남겼다. 커밋, 이력 재작성, force push는 하지 않았다.

`simulation/ros_ws/src/pinky_pro` submodule은 내용이 수정된 상태로 보였다. 제조사 소스를 되돌리거나 삭제하지 않았다.

## 제거한 파일

추적에서 제거한 과거 백업과 패치:

- `.env.before_5cm_20261028_154016`
- `.env.before_nav2_auto_20261028_134132`
- `.env.before_password_ssh_20261028_132832`
- `apply_update.py`, `test_patch.py`, `README.txt`
- `fms_restore_full_dashboard_patch.zip`
- `backend`와 `frontend`의 `*.before_*`, `*.backup_*`, `*.bak`
- `config/maps/physical_site/1f_map.yaml.bak`
- `nav2_params.yaml.bak`, `nav2_params.yaml.before_collision_monitor`
- `launch/pinky1_navigation.launch.launch.py.before_smoother_fix`

이 파일들은 현재 Python, TypeScript, launch, 테스트 import에 없다. 실행 중인 코드는 같은 이름의 현재 소스를 쓴다.

재생성 가능한 colcon 산출물도 추적에서 제거했다.

- `simulation/ros_ws/build` 355개
- `simulation/ros_ws/install` 617개
- `simulation/ros_ws/log` 62개

`simulation/scripts/run_nav2_fms.sh`는 `/opt/ros/jazzy`와 `/home/ubuntu/pinky_pro/install`을 읽고, 이 워크스페이스의 `install`을 source하지 않는다. 로컬 colcon 결과가 필요하면 `colcon build`로 다시 만든다.

무시되는 빌드 디렉터리 `frontend/.next-verify`도 삭제했다. 개발 서버가 쓰는 `frontend/.next`는 남겼다.

정리 전 추적 파일은 1778개였고, 작업 트리 전체는 약 992MB였다. 정리 후 추적 파일은 709개다. 스테이징된 삭제는 symlink 하나를 되돌린 뒤 1069개 경로다. `.venv`, `frontend/node_modules`, `frontend/.next`를 뺀 작업 트리는 약 214MB다.

## 유지한 파일

- `backend/`, `frontend/src`, `robot_receiver/`, `robot_agent/`, `launch/pinky1_navigation.launch.py`
- `config/maps/physical_site/1f_map_f.pgm`, `1f_map_f.yaml`, `nav2_params.yaml`
- `config/physical_fleet.json`
- 현재 `.env`와 `data/`의 SQLite
- `backend/care_move_fms.egg-info`. 편집 가능 설치가 참조한다
- `simulation/ros_ws/src/pinky_pro`
- 기존 개발 보고서와 테스트

## Gitignore

다음을 무시하도록 추가했다.

- `.env.before_*`, `*.before_*`, `*.bak`, `*.backup_*`
- `frontend/.next-verify/`
- `simulation/ros_ws/build/`, `install/`, `log/`

## 보안

세 `.env.before_*` 파일은 Git에 추적되어 있었고 다음 변수 이름을 포함한다. 값은 출력하지 않았다.

- 공통: `FMS_ENV`, `FMS_ENVIRONMENT_ID`, `FMS_LEASE_SECRET`, `FMS_DATABASE_URL`, `FMS_ROS_DOMAIN_ID`, `FMS_API_HOST`, `FMS_API_PORT`, `FMS_PHYSICAL_CONTROL_ENABLED`, `FMS_ROS_BRIDGE_ENABLED`, `FMS_AUTO_TICK`, `FMS_SEED_FLEET`
- 5cm 백업과 nav2 백업에는 `FMS_PHYSICAL_SSH_PASSWORD`, `FMS_PHYSICAL_SSH_TARGETS`도 있다
- 5cm 백업에는 Nav2, 지도 변수도 있다

현재 `.env`는 ignore되어 커밋 대상이 아니다. 그러나 백업 파일이 이미 `6e856c9`와 그 이전 이력에 들어갔다면, 작업 트리에서 지워도 과거 커밋의 비밀값은 남는다. 파일 삭제를 이력 삭제라고 보지 않는다.

승인 전에 하지 않은 일:

- `git filter-repo` 또는 rebase로 이력 재작성
- force push
- SSH 비밀번호나 lease secret 교체

필요한 후속 조치는 비밀번호와 lease secret을 교체하고, 원격 이력을 지울지 별도로 정하는 것이다. 교체 전에는 그 자격으로 로그인할 수 있다고 봐야 한다.

`simulation/ros_ws/install`을 통째로 지우자 `backend/tests/test_route_planning.py`가 다음 끊어진 symlink를 찾지 못했다.

`simulation/ros_ws/install/aws_robomaker_small_house_world/share/aws_robomaker_small_house_world/launch/view_small_house.launch.py`

대상 파일은 없고, 테스트는 그 상태를 고치지 말고 유지하라고 검사한다. 그 링크만 Git에서 되돌렸다. install의 나머지 colcon 산출물은 삭제된 상태다.

## 회귀

ROS 변수를 내린 뒤 backend pytest 117개가 통과했다.

```bash
cd backend
env -u ROS_DISTRO -u ROS_DOMAIN_ID -u RMW_IMPLEMENTATION \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv/bin/pytest -q -p pytest
```

`ROS_DISTRO=jazzy`가 켜진 셸에서는 `test_unit_live_preflight_is_not_ready_and_client_fails_closed`가 다른 실패 이유를 기대하므로 실패한다. 제품 기동 실패가 아니다.

`frontend/node_modules/.bin/tsc --noEmit`은 통과했다. `FMS_NEXT_DIST_DIR=.next-verify npx next build --turbopack`도 통과했다. 빌드가 `tsconfig.json`에 넣은 `.next-verify/types/**/*.ts` include는 제거했다. 개발 서버의 `.next`는 유지했다.

지도 `1f_map_f`와 현재 `.env`, SQLite는 삭제하지 않았다. 5cm POST와 Pinky Task dispatch는 호출하지 않았다.
