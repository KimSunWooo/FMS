# Phase 2.2.1 반복 주행

시각은 KST(UTC+9)다. 원본 JSON은 `docs/phase2-2-1-logs/repeats/`다. Phase 2.2의 `docs/phase2-2-logs/repeats/` 0/5는 분모에 넣지 않았다.

## 환경

- ROS 2 Jazzy, Gazebo Harmonic, world `pinky_factory`, 로봇 1대
- `ROS_DOMAIN_ID=73`, `ROS_LOCALHOST_ONLY=1`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Nav2는 `simulation/config/nav2/fms_nav2_params.yaml`을 `params_file`로 주입했다. 주행 시작 시 라이브 값은 stateful false, xy 0.12 m, yaw 0.25 rad, planner tolerance 0.15 m, 회전 0.5 rad/s, 접근 0.03 m/s, `set_initial_pose` false
- FMS API `127.0.0.1:8741`, DB `data/fms-phase22.sqlite3`, `pose_tolerance_m` 0.25
- API 프로세스는 ROS를 source하지 않았다. 주행 전 health의 `gazebo_preflight.status`는 `not_ready`
- 각 회 전에 map TF가 있고 `/odom` twist가 정지에 가까웠다. 활성 Nav2 status 1·2·3은 없었다
- 점수 외 1건: 재시작된 Nav2는 goal status를 한 번도 발행하지 않아 에이전트가 전송을 거절한다. 현재 자세로 goal을 수락한 뒤 즉시 취소해 status 5를 만들었다. task를 만들지 않았고 성공률에 넣지 않는다. 기록은 `status-prime.json`

## 성공률

| 표본 | 완료 | 채점 | 성공률 |
| --- | --- | --- | --- |
| Phase 2.2 baseline | 0 | 5 | 0% |
| 같은 좌표 재실행 | 4 | 5 | 80% |
| 추가 독립 주행 | 7 | 7 | 100% |
| 오버레이 이후 합계 | 11 | 12 | 91.7% |

합계는 재실행 5회와 추가 7회를 더한 것이다. baseline 0/5와 섞지 않았다. 12회는 안정성 표본이며 운영 신뢰성이 아니다. goal을 보내지 않은 회차는 없다. `navigation_goal_count`는 12회 모두 1이고, 에이전트 UUID와 DB UUID가 같다.

PASS는 FMS `COMPLETED`, Nav2 status 4, map 거리 0.25 m 이하, UUID 일치, goal 행 1개다. 5회차는 이 기준에서 FAIL이다.

완료된 11회의 거리는 0.081 m에서 0.177 m다. 결과 뒤 재측정 twist는 12회 모두 0이다.

## 기존 실패 좌표 재실행

| 회 | task | execution | FMS goal | Nav2 UUID | 목표 x, y, yaw | 최종 x, y | 오차 | Nav2 | FMS |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | `7f050c8e-c9a7-4a99-9782-b53a481e1d98` | `d4eb861e-0b6b-4afb-8a0b-40f102ca74ee` | `c6a7a38a-6117-474c-8ae7-31d0b12b02f9` | `bd8fbe55-b8c3-4e61-b989-53873b72d5a7` | -0.8, 0.2, 0 | -0.957, 0.119 | 0.177 m | 4 | COMPLETED |
| 2 | `6d2892a7-7ed8-4db3-bbb2-d45ba80fb24d` | `4b4770c5-73e3-4848-b9ff-8ffcf6e49b83` | `d9efd010-b4de-4fed-9a88-46ea60763f01` | `dc08277a-6aed-40a5-83a8-68030c34e389` | 0.2, -0.8, 0 | 0.101, -0.884 | 0.130 m | 4 | COMPLETED |
| 3 | `ea464d92-d087-47f6-b6cb-4535454d5caf` | `82c3a998-4a2e-4ddc-ab64-f8f161448637` | `c96dfe82-e212-4a75-8f14-23a5af5876bb` | `e04f1736-00c1-4581-b394-10ec30dde8b2` | -0.2, 0.8, 0 | -0.121, 0.724 | 0.109 m | 4 | COMPLETED |
| 4 | `3cf5c23e-eaf7-43be-afd5-9ae8145dbdf8` | `c7006d6f-4f2a-4c05-9636-878cd0e7e8ce` | `0c9485da-a703-47c4-b10c-689c4e735a19` | `fd81f3d9-2eca-4c36-ba91-de01325ae6e3` | 0.8, -0.3, 0 | 0.688, -0.228 | 0.133 m | 4 | COMPLETED |
| 5 | `df7babef-a595-452d-9854-d2a56171fb5d` | `67fd7347-e5d9-467d-bc41-2a06f4288ee0` | `d14cd682-ac6e-4081-bb5a-86aa2ff28141` | `c95ec52e-5c07-4c63-94bf-e8680d909b46` | -0.6, -0.4, 0 | 0.699, -0.200 | 1.314 m | 6 | FAILED |

시각과 TF:

| 회 | 전송 | Nav2 결과 | map TF stamp (sim sec) |
| --- | --- | --- | --- |
| 1 | 2026-10-26 17:11:47 KST | 17:12:00 KST | 4190.420000000 |
| 2 | 17:12:20 KST | 17:13:07 KST | 4217.960000000 |
| 3 | 17:13:26 KST | 17:14:42 KST | 4260.600000000 |
| 4 | 17:15:01 KST | 17:15:32 KST | 4280.980000000 |
| 5 | 17:15:51 KST | 17:16:34 KST | 4305.660000000 |

5회차 사유는 `nav2_goal_aborted`다. 전송 전 `ComputePathToPose`가 status 6, pose 0이었다. 최종 자세는 출발 (0.681, -0.222)에서 약 3 cm 이동한 곳이다. 재전송하지 않았다.

## 추가 독립 주행

방향은 0이 아니다. X와 Y의 부호를 모두 포함한다.

| 회 | task | Nav2 UUID | 목표 x, y, yaw | 최종 x, y | 오차 | Nav2 | FMS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 6 | `a87fcd98-7c4e-4a3e-a7dd-6684283accec` | `33719503-1678-4a20-8c36-cd252bd28213` | 0, 0, 1.2 | 0.046, -0.067 | 0.081 m | 4 | COMPLETED |
| 7 | `a2388bef-8b3e-422e-95a0-d800636c990c` | `47a336f8-da0e-4fd4-bdcd-ea738829318f` | -0.4, -0.2, -1.0 | -0.312, -0.143 | 0.105 m | 4 | COMPLETED |
| 8 | `e08b7f8d-ddf7-429d-9d78-dfc3db47489a` | `9fe11d5b-b1f5-4555-91f9-d4ab94366f22` | 0.5, 0.15, 0.6 | 0.428, 0.060 | 0.115 m | 4 | COMPLETED |
| 9 | `ecdff1b0-89bd-45bc-bbaa-713143c7ac38` | `3f40b96d-552f-46d0-be6e-c3bcd317c51e` | -1.0, -0.5, 0.4 | -1.021, -0.600 | 0.102 m | 4 | COMPLETED |
| 10 | `f3096de4-b2dd-4a2f-9f62-cf93713e28e9` | `d6f2b0ad-947f-487e-9e93-fd80f6b37fe6` | 0.3, 0.25, -0.8 | 0.262, 0.138 | 0.118 m | 4 | COMPLETED |
| 11 | `4c21e8c2-12cb-4c4b-a095-de54ae485d92` | `3f3a0e86-12db-48a1-9e04-f55096a2cfa6` | -0.3, 0.35, 2.0 | -0.235, 0.257 | 0.114 m | 4 | COMPLETED |
| 12 | `38017360-e692-44c9-96fa-b801b204fa6f` | `fbeda576-05ea-4bf7-bd91-bd8c16c55129` | 0.6, -0.5, 0.3 | 0.573, -0.332 | 0.170 m | 4 | COMPLETED |

execution과 FMS goal:

| 회 | execution | FMS goal | 전송 | 결과 | TF stamp |
| --- | --- | --- | --- | --- | --- |
| 6 | `b2b8cd46-ae45-46e2-8125-0e41e78dad81` | `89459aaa-690a-497e-a321-a033be90d7ee` | 17:16:53 KST | 17:17:15 KST | 4321.800000000 |
| 7 | `74d58b33-12e7-47ca-8621-0003423a956c` | `d3701803-3be5-4621-9dd2-52b30ff78dd9` | 17:17:33 KST | 17:18:02 KST | 4341.200000000 |
| 8 | `2bfb3fa8-0a1d-47c0-a747-73258e6466cf` | `bad2fe7b-c853-4cda-90fc-f88fe7c937ee` | 17:18:21 KST | 17:18:43 KST | 4358.220000000 |
| 9 | `aade60b6-9b2a-4913-8b6f-ffbdba0ff042` | `03778d0f-21fc-4cf2-a48f-f9a9fc89b0ce` | 17:19:03 KST | 17:19:56 KST | 4389.580000000 |
| 10 | `997851cb-f1f4-46b9-abd7-5b9fa3e4d2c1` | `ef759a4e-0fa9-4a99-bbcf-16962d31dbd9` | 17:20:15 KST | 17:21:07 KST | 4420.740000000 |
| 11 | `8310c668-1765-4a86-b6f3-f92d426f427d` | `db855f47-d1d3-4181-96f0-e38358459767` | 17:21:27 KST | 17:21:52 KST | 4439.700000000 |
| 12 | `33d46ad4-f8d8-4360-8625-9ce0adb13b9b` | `ed9269de-95a2-4ed7-99fc-919259e24e4c` | 17:22:13 KST | 17:23:15 KST | 4469.460000000 |

12회차 경로 끝은 목표에서 0.05 m였다. 최종 0.170 m는 그 끝점에서 checker 0.12 m 바깥쪽에 선 거리와 맞다. 0.25 m 안이라 `COMPLETED`다. snap 0.15 m와 checker 0.12 m가 같은 방향으로 겹치면 0.27 m가 될 수 있다. 이번 12회에서는 그 조합이 `COMPLETED`로 넘어간 경우는 없다.

## 실패

5회차만 실패다. 원인 문서의 플래너 거절과 같다. 나머지 회차에서 recovery 사유는 없다.
