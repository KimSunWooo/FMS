"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { FleetTaskPanel, type RouteOverlay } from "@/components/fleet-task-panel";
import { insidePixel, pixelToWorld, worldToGrid, worldToPixel, type GridPoint } from "@/lib/map-frame";

type Pose = { x: number; y: number; yaw: number; grid?: GridPoint | null };
type RobotHome = {
  frame_id: "map";
  x: number;
  y: number;
  yaw: number;
  grid: GridPoint | null;
  pixel: { x: number; y: number } | null;
  inside_map: boolean | null;
  source: "registry_home";
};
type PhysicalRobot = {
  robot_id: string;
  name: string;
  kind: "physical" | "virtual";
  host: string | null;
  connection_status: string;
  connection_kind: string;
  connection_phase?: string;
  ros_domain_id?: number;
  physical_connection_enabled?: boolean;
  planned_physical?: boolean;
  ssh_online: boolean;
  pose: Pose | null;
  pose_checked_at?: string | null;
  pose_source?: string | null;
  pose_valid?: boolean;
  pose_freshness?: string | null;
  battery_valid?: boolean;
  battery_percentage?: number | null;
  battery_source?: string | null;
  battery_timestamp?: string | null;
  battery_freshness?: string | null;
  plan_valid?: boolean;
  plan?: { frame_id?: string; poses?: { x: number; y: number }[]; pose_count?: number } | null;
  plan_source?: string | null;
  plan_timestamp?: string | null;
  plan_freshness?: string | null;
  receiver_ready?: boolean | null;
  nav2_ready: boolean | null;
  ros_online?: boolean;
  nav2_state?: string | null;
  nav2_detail?: string | null;
  controller_ready: boolean | null;
  map_matches_fms?: boolean | null;
  live_tf_status?: string | null;
  task_assignable?: boolean;
  motion_allowed?: boolean;
  worker?: {
    connection_state?: string | null;
    tf_status?: string | null;
    last_seen?: string | null;
    worker_running?: boolean | null;
    detail?: string | null;
  } | null;
  home?: RobotHome | null;
};
type PhysicalFleet = {
  robots: PhysicalRobot[];
  configured_count: number;
  connected_count: number;
  physical_count: number;
  virtual_count: number;
  checked_at: string;
  data_source: "fleet_registry";
};
type PhysicalMap = {
  frame_id: "map";
  width_px: number;
  height_px: number;
  width_m: number;
  height_m: number;
  resolution: number;
  origin: { x: number; y: number; yaw: number };
  image_url: string;
  source: string;
  data_source: "physical_site_map_files";
  grid?: { width: number; height: number; convention: string; row_origin: string };
};

type Task = {
  task_id: string;
  status: string;
  robot_id: string | null;
  destination: { x: number; y: number; yaw: number; label: string };
  owner_epoch: number;
  navigation_goal_count: number;
  recovery_reason: string | null;
  failure_reason: string | null;
  completion_evidence: { goal_status: string; distance_m: number } | null;
};
type Zone = {
  zone_id: string;
  name: string;
  kind: string;
  bounds: { x: number; y: number; w: number; h: number };
  reservation: { reservation_status: string; robot_id: string } | null;
};
type FeedEvent = { kind: string; code: string; message: string; created_at: string | null };

async function readJson<T>(path: string, signal?: AbortSignal): Promise<T> {
  const response = await fetch(path, { cache: "no-store", signal });
  if (!response.ok) {
    let detail = `HTTP ${response.status}`;
    try {
      const data = await response.json();
      detail = typeof data?.detail === "string" ? data.detail : detail;
    } catch { /* Keep HTTP status if response body is not JSON. */ }
    throw new Error(detail);
  }
  return response.json() as Promise<T>;
}

function statusVariant(status: string): "default" | "secondary" | "destructive" | "outline" {
  if (["COMPLETED", "ssh_connected", "online"].includes(status)) return "default";
  if (["FAILED", "RECOVERY_REQUIRED", "offline", "ssh_offline"].includes(status)) return "destructive";
  if (["RUNNING", "busy"].includes(status)) return "secondary";
  return "outline";
}

const VIRTUAL_COLOR: Record<string, string> = {
  "pinky-1": "#007d82",
  "pinky-2": "#c47b00",
  "pinky-3": "#5b4db7",
};

export function FleetConsole() {
  const [fleet, setFleet] = useState<PhysicalFleet | null>(null);
  const [physicalMap, setPhysicalMap] = useState<PhysicalMap | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [mapError, setMapError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [label, setLabel] = useState("");
  const [x, setX] = useState("");
  const [y, setY] = useState("");
  const [robotId, setRobotId] = useState("");
  // FMS_PINKY1_5CM_TEST_V1: real Nav2 relative forward-test, no mock task write.
  // FMS_DIRECT_5CM_LIVE_POSE_V1: server holds the test token, not the browser.
  const [livePose, setLivePose] = useState<{
    robot_id: string;
    pose: Pose | null;
    pose_checked_at?: string | null;
    pose_source?: string;
    pose_valid?: boolean;
    battery_valid?: boolean;
    battery_percentage?: number | null;
    battery_source?: string | null;
    battery_timestamp?: string | null;
    battery_freshness?: string | null;
    live_tf_status?: string | null;
    live_tf_age_s?: number | null;
    live_tf_withheld_reason?: string | null;
    collision_monitor_active?: boolean | null;
    nav2_ready?: boolean | null;
    map_matches_fms?: boolean | null;
    map_server_yaml?: string | null;
    expected_map_yaml?: string | null;
    map_mismatch_detail?: string | null;
    five_cm_blockers?: string[];
  } | null>(null);
  const [fiveCmStatus, setFiveCmStatus] = useState<{
    state: string; detail?: string; finished_at?: string; requested_distance_m?: number;
    actual_forward_m?: number | null; map_forward_m?: number | null; odom_forward_m?: number | null;
    actual_lateral_m?: number | null; goal_status_name?: string; velocity_case?: string | null;
    start_pose?: { x: number; y: number } | null; final_pose?: { x: number; y: number } | null;
  } | null>(null);
  const [fiveCmError, setFiveCmError] = useState<string | null>(null);
  const [chainDiagnosis, setChainDiagnosis] = useState<{
    motion_command_sent?: boolean; stop_zone_would_stop?: boolean; velocity_case?: string | null; detail?: string;
    scan?: {
      received?: boolean; min_range_m?: number | null; range_min_m?: number | null; range_max_m?: number | null;
      points_within_stop_radius?: number; frame_id?: string | null;
      inside_sectors_base_frame?: { front?: number; left?: number; rear?: number; right?: number };
      nearest_base_distance_m?: number | null;
      right_returns_base_m?: { x: number; y: number; range_m: number; radius_m: number }[];
      right_return_identity?: string; self_reflection_proven?: boolean;
      lidar_in_base_m?: { x: number; y: number; z: number; yaw_rad: number };
    };
  } | null>(null);
  const [fiveCmPending, setFiveCmPending] = useState(false);
  const [routeOverlay, setRouteOverlay] = useState<RouteOverlay | null>(null);
  const [goalWarning, setGoalWarning] = useState<string | null>(null);

  // Existing FMS sections remain, but Phase-1 Mock DB/task/zone/event APIs
  // must NOT be read or rendered as if they were physical robot observations.
  const tasks: Task[] = useMemo(() => [], []);
  const zones: Zone[] = useMemo(() => [], []);
  const active = useMemo(
    () => tasks.filter((task) => !["COMPLETED", "CANCELED", "FAILED"].includes(task.status)),
    [tasks],
  );
  const history = useMemo(
    () => tasks.filter((task) => ["COMPLETED", "CANCELED", "FAILED"].includes(task.status)).slice(0, 8),
    [tasks],
  );
  const events: FeedEvent[] = useMemo(() => {
    if (!fleet) return [];
    return [{
      kind: "fleet_registry",
      code: "fleet_registry_checked",
      message: `등록 ${fleet.configured_count}대 · 실물 ${fleet.physical_count}대(SSH ${fleet.connected_count}) · 가상 ${fleet.virtual_count}대`,
      created_at: fleet.checked_at,
    }];
  }, [fleet]);

  const refresh = useCallback(async (signal?: AbortSignal) => {
    // Independent requests: an absent map should not hide authenticated robots.
    const [robotsResult, mapResult] = await Promise.allSettled([
      readJson<PhysicalFleet>("/fms-api/physical/robots", signal),
      readJson<PhysicalMap>("/fms-api/physical/map", signal),
    ]);
    if (signal?.aborted) return;
    if (robotsResult.status === "fulfilled" && Array.isArray(robotsResult.value.robots)) {
      setFleet(robotsResult.value);
      setLoadError(null);
    } else {
      setFleet(null); // Do not leave stale robots displaying as online.
      setLoadError(robotsResult.status === "rejected"
        ? String(robotsResult.reason instanceof Error ? robotsResult.reason.message : robotsResult.reason)
        : "잘못된 실물 로봇 응답");
    }
    if (mapResult.status === "fulfilled" && Number.isFinite(mapResult.value.width_px)) {
      setPhysicalMap(mapResult.value);
      setMapError(null);
    } else {
      setPhysicalMap(null);
      setMapError(mapResult.status === "rejected"
        ? String(mapResult.reason instanceof Error ? mapResult.reason.message : mapResult.reason)
        : "실제 지도 형식이 올바르지 않습니다.");
    }
    setLoading(false);
  }, []);

  useEffect(() => {
    const controller = new AbortController();
    void refresh(controller.signal);
    const timer = window.setInterval(() => void refresh(controller.signal), 5000);
    return () => {
      controller.abort();
      window.clearInterval(timer);
    };
  }, [refresh]);

  useEffect(() => {
    const controller = new AbortController();
    const poll = async () => {
      try {
        const result = await readJson<{
          state: string; detail?: string; finished_at?: string; requested_distance_m?: number;
          actual_forward_m?: number | null; map_forward_m?: number | null; odom_forward_m?: number | null;
          actual_lateral_m?: number | null; goal_status_name?: string; velocity_case?: string | null;
          start_pose?: { x: number; y: number } | null; final_pose?: { x: number; y: number } | null;
        }>(
          "/fms-api/physical/tests/pinky-1/forward-5cm", controller.signal,
        );
        if (!controller.signal.aborted) setFiveCmStatus(result);
      } catch { /* Separate from SSH/TF status; don't hide the fleet. */ }
    };
    void poll();
    const timer = window.setInterval(() => void poll(), 1500);
    return () => { controller.abort(); window.clearInterval(timer); };
  }, []);

  // Dedicated 1s TF polling. SSH/ROS health requests stay on their slower cadence.
  useEffect(() => {
    const controller = new AbortController();
    let pending = false;
    const poll = async () => {
      if (pending || controller.signal.aborted) return;
      pending = true;
      try {
        const result = await readJson<{
          robot_id: string; pose: Pose | null; pose_checked_at?: string | null; pose_source?: string;
          live_tf_status?: string | null; live_tf_age_s?: number | null; live_tf_withheld_reason?: string | null;
          collision_monitor_active?: boolean | null; nav2_ready?: boolean | null;
          map_matches_fms?: boolean | null; map_server_yaml?: string | null; expected_map_yaml?: string | null;
          map_mismatch_detail?: string | null; five_cm_blockers?: string[];
        }>(
          "/fms-api/physical/nav2/pose", controller.signal,
        );
        if (!controller.signal.aborted) setLivePose(result);
      } catch { if (!controller.signal.aborted) setLivePose(null); }
      finally { pending = false; }
    };
    void poll();
    const timer = window.setInterval(() => void poll(), 1000);
    return () => { controller.abort(); window.clearInterval(timer); };
  }, []);

  async function diagnoseVelocityChain() {
    setFiveCmError(null);
    try {
      const result = await readJson<NonNullable<typeof chainDiagnosis>>("/fms-api/physical/tests/pinky-1/forward-5cm/diagnosis");
      setChainDiagnosis(result);
    } catch (err) {
      setFiveCmError(err instanceof Error ? err.message : "속도 체인 진단 실패");
    }
  }

  async function sendFiveCmAction(cancel: boolean) {
    if (!cancel && !window.confirm("pinky-1 실기체에 Nav2 DriveOnHeading 5cm 전진 목표를 전송합니다. 실행하시겠습니까?")) return;
    setFiveCmPending(true);
    setFiveCmError(null);
    try {
      const response = await fetch("/api/physical-5cm", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-FMS-UI-Intent": "physical-5cm" },
        body: JSON.stringify({ action: cancel ? "cancel" : "start" }),
      });
      const body = await response.json();
      if (!response.ok) throw new Error(typeof body.detail === "string" ? body.detail : `HTTP ${response.status}`);
      if (!cancel) setFiveCmStatus(body);
    } catch (error) {
      setFiveCmError(error instanceof Error ? error.message : "5cm 작업 요청 실패");
    } finally { setFiveCmPending(false); }
  }

  async function chooseDestination(px: number, py: number) {
    if (!physicalMap) return;
    const world = pixelToWorld(physicalMap, px, py);
    setX(world.x.toFixed(3));
    setY(world.y.toFixed(3));
    try {
      const response = await fetch("/fms-api/planning/points/inspect", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x: world.x, y: world.y }),
      });
      const body = await response.json() as { accepted_as_goal?: boolean; reason?: string };
      setGoalWarning(body.accepted_as_goal ? null : `클릭 위치는 목적지로 사용할 수 없습니다 (${body.reason ?? "거부"}).`);
    } catch {
      setGoalWarning("목적지 검사에 실패했습니다.");
    }
  }

  const selectedGrid = useMemo(() => {
    if (!physicalMap || !x.trim() || !y.trim()) return null;
    const gx = Number(x);
    const gy = Number(y);
    if (!Number.isFinite(gx) || !Number.isFinite(gy)) return null;
    return worldToGrid(physicalMap, gx, gy);
  }, [physicalMap, x, y]);

  const selected = useMemo(() => {
    if (!physicalMap || !selectedGrid) return null;
    const { px, py } = worldToPixel(physicalMap, Number(x), Number(y));
    return insidePixel(physicalMap, px, py) ? { px, py } : null;
  }, [physicalMap, selectedGrid, x, y]);

  // FMS_LIVE_TF_POSE_OVERLAY_V1: map coordinates -> SVG image pixels.
  // The server only sends fresh TF results; time-gate again in the browser.
  const liveMarkers = useMemo(() => {
    if (!physicalMap) return [];
    const theta = physicalMap.origin.yaw;
    return (fleet?.robots ?? []).flatMap((robot) => {
      const fromLive = robot.robot_id === "pinky-1" && livePose?.robot_id === "pinky-1" && livePose.pose_valid
        ? livePose
        : null;
      const observation = fromLive ?? robot;
      const source = observation.pose_source ?? "";
      const accepted =
        source === "topic:amcl_pose" ||
        source.includes("amcl_pose") ||
        source === "tf:map->base_footprint" ||
        source.startsWith("robot_status:");
      if (!observation.pose || !observation.pose_checked_at || !accepted) return [];
      const isStale = robot.kind === "physical" && robot.pose_valid === false && !fromLive;
      if (isStale && robot.pose_freshness !== "stale") return [];
      const stamp = Date.parse(observation.pose_checked_at);
      const age = Date.now() - stamp;
      // Fresh samples: 5s gate. Stale last-value: keep visible up to 10 minutes.
      const maxAge = isStale ? 600_000 : 5000;
      if (!Number.isFinite(stamp) || age < -5000 || age > maxAge) return [];
      const { x: worldX, y: worldY, yaw } = observation.pose;
      if (![worldX, worldY].every(Number.isFinite)) return [];
      const yawValue = Number.isFinite(yaw) ? yaw : 0;
      const { px, py } = worldToPixel(physicalMap, worldX, worldY);
      if (!insidePixel(physicalMap, px, py)) return [];
      return [{ robot, px, py, rotation: -(yawValue - theta) * 180 / Math.PI, stale: isStale }];
    });
  }, [fleet, physicalMap, livePose]);

  const navPlanPolylines = useMemo(() => {
    if (!physicalMap) return [];
    return (fleet?.robots ?? []).flatMap((robot) => {
      if (!robot.plan?.poses?.length) return [];
      if (!robot.plan_valid && robot.plan_freshness !== "stale") return [];
      const points = robot.plan.poses.map((point) => {
        const pixel = worldToPixel(physicalMap, point.x, point.y);
        return `${pixel.px},${pixel.py}`;
      }).join(" ");
      if (!points) return [];
      const base = VIRTUAL_COLOR[robot.robot_id] ?? "#2f6f4e";
      return [{
        robotId: robot.robot_id,
        points,
        color: robot.plan_valid ? base : "#9aa5a0",
        stale: !robot.plan_valid,
      }];
    });
  }, [fleet, physicalMap]);

  const homeMarkers = useMemo(() => {
    if (!physicalMap) return [];
    return (fleet?.robots ?? []).flatMap((robot) => {
      if (robot.kind !== "virtual" || !robot.home || robot.home.inside_map === false) return [];
      const { px, py } = worldToPixel(physicalMap, robot.home.x, robot.home.y);
      if (!insidePixel(physicalMap, px, py)) return [];
      return [{ robot, px, py, rotation: -(robot.home.yaw - physicalMap.origin.yaw) * 180 / Math.PI }];
    });
  }, [fleet, physicalMap]);

  // Permanent reference only: never passed off as the live robot pose.
  const initialMarker = useMemo(() => {
    if (!physicalMap) return null;
    const { px, py } = worldToPixel(physicalMap, 1.696, 0.103);
    if (!insidePixel(physicalMap, px, py)) return null;
    return { px, py };
  }, [physicalMap]);

  return (
    <main className="mx-auto flex min-h-screen max-w-7xl flex-col gap-4 px-4 py-5 md:px-6">
      <header className="flex flex-col gap-3 border-b border-border pb-4 md:flex-row md:items-end md:justify-between">
        <div>
          <p className="text-xs tracking-[0.18em] text-primary uppercase">Care ward · physical fleet</p>
          <h1 className="text-2xl font-semibold">Care-Move FMS</h1>
          <p className="max-w-2xl text-sm text-muted-foreground">
            실물 Pinky-1 Task는 ROS 2 Topic과 실시간 TF로 확인하고, SSH는 접속 진단에만 씁니다.
            Pinky-2와 Pinky-3는 가상 등록 로봇입니다. 가상 로봇에는 주행 명령을 보내지 않습니다.
          </p>
        </div>
        <div className="flex flex-wrap gap-2 text-xs">
          <Badge variant="outline">RMF 미사용</Badge>
          <Badge variant="outline">실물 관제 조회</Badge>
          <Badge variant="outline">5cm Nav2 테스트 전용</Badge>
          <Badge variant="outline">실물 {fleet?.physical_count ?? 0} · SSH {fleet?.connected_count ?? 0}</Badge>
          <Badge variant="outline">가상 {fleet?.virtual_count ?? 0}</Badge>
          <Badge variant="outline">등록 {fleet?.configured_count ?? 0}대</Badge>
          <Badge variant={physicalMap ? "default" : "destructive"}>
            지도 {physicalMap ? "실제 파일" : "미확인"}
          </Badge>
        </div>
      </header>

      {loading && <p className="text-sm text-muted-foreground">실물 Pinky와 실제 지도 확인 중...</p>}
      {loadError && (
        <Card>
          <CardHeader>
            <CardTitle>SSH 로봇 목록 API 연결 오류</CardTitle>
            <CardDescription>{loadError} · Backend(8741) 및 SSH 설정을 확인하세요.</CardDescription>
          </CardHeader>
        </Card>
      )}

      <section className="grid gap-3 md:grid-cols-3">
        {(fleet?.robots ?? []).map((robot) => (
          <Card key={robot.robot_id}>
            <CardHeader>
              <CardTitle className="flex items-center justify-between gap-2">
                <span>{robot.name}</span>
                <span className="font-mono text-xs text-muted-foreground">{robot.robot_id}</span>
              </CardTitle>
              <CardDescription>
                {robot.kind === "virtual"
                  ? "Virtual / Physical connection pending"
                  : `실물 로봇 · ${robot.host ?? "SSH 대상 없음"}`}
              </CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-3">
              <p className="font-mono text-xs">ROS Domain {robot.ros_domain_id ?? "-"}</p>
              <div className="flex flex-wrap gap-1.5">
                <Badge variant={robot.kind === "virtual" ? "secondary" : "outline"}>
                  {robot.kind === "virtual" ? "가상" : "실물"}
                </Badge>
                {robot.kind === "physical" ? (
                  <Badge variant={statusVariant(robot.connection_status)}>
                    {robot.ssh_online ? "SSH 연결" : "SSH 끊김"}
                  </Badge>
                ) : (
                  <Badge variant="outline">SSH 미사용</Badge>
                )}
                <Badge variant={robot.ros_online || robot.worker?.connection_state === "ros_connected" || robot.worker?.connection_state === "nav2_ready" ? "default" : "outline"}>
                  ROS {robot.ros_online || robot.worker?.connection_state === "ros_connected" || robot.worker?.connection_state === "nav2_ready" ? "노드 확인" : robot.kind === "virtual" ? "해당 없음" : "대기"}
                </Badge>
                <Badge variant={robot.battery_valid ? "default" : "outline"}>
                  {robot.kind === "virtual"
                    ? "배터리 해당 없음"
                    : typeof robot.battery_percentage === "number"
                      ? robot.battery_valid
                        ? `배터리 ${(robot.battery_percentage * 100).toFixed(0)}%`
                        : `배터리 ${(robot.battery_percentage * 100).toFixed(0)}% · Stale`
                      : "배터리 정보 없음"}
                </Badge>
                <Badge variant={robot.nav2_ready ? "default" : "outline"}>
                  Nav2 {robot.nav2_ready ? "READY" : robot.kind === "virtual" ? "해당 없음" : "준비 대기"}
                </Badge>
              </div>
              <p className="font-mono text-xs text-muted-foreground">
                {robot.kind === "virtual" && robot.home
                  ? `등록 위치 ${robot.home.x.toFixed(2)}, ${robot.home.y.toFixed(2)} m${robot.home.grid ? ` · grid ${robot.home.grid.col.toFixed(0)}, ${robot.home.grid.row.toFixed(0)}` : ""}`
                  : robot.pose
                    ? `pose ${robot.pose.x.toFixed(2)}, ${robot.pose.y.toFixed(2)} · yaw ${typeof robot.pose.yaw === "number" ? `${(robot.pose.yaw * 180 / Math.PI).toFixed(1)}°` : "—"} · ${robot.pose_freshness ?? "valid"}`
                    : `pose 미수신 · ${robot.pose_freshness ?? "missing"}`}
              </p>
              {robot.kind === "physical" && (
                <p className="font-mono text-xs text-muted-foreground">
                  {typeof robot.battery_percentage === "number"
                    ? `배터리 ${(robot.battery_percentage * 100).toFixed(1)}% · ${robot.battery_source ?? "/battery/percent"} · ${robot.battery_freshness ?? "valid"}${robot.battery_timestamp ? ` · ${robot.battery_timestamp}` : ""}`
                    : `배터리 정보 없음 · ${robot.battery_freshness ?? "missing"}`}
                  {robot.receiver_ready != null ? ` · Receiver ${robot.receiver_ready ? "Ready" : "Down"}` : ""}
                </p>
              )}
              {robot.kind === "physical" && (
                <p className="font-mono text-xs text-muted-foreground">
                  {robot.plan
                    ? `Nav2 /plan ${robot.plan.pose_count ?? robot.plan.poses?.length ?? 0} poses · ${robot.plan_freshness ?? "valid"}`
                    : `Nav2 /plan ${robot.plan_freshness === "stale" ? "Stale" : "정보 없음"}`}
                  {robot.pose_source ? ` · pose src ${robot.pose_source}` : ""}
                </p>
              )}
              <p className="text-xs text-muted-foreground">Nav2: {robot.nav2_detail ?? "상태 점검 중"}</p>
              {robot.robot_id === "pinky-1" && livePose?.pose_valid && livePose.pose && livePose.pose_checked_at &&
                Date.now() - Date.parse(livePose.pose_checked_at) < 5000 && (
                <p className="font-mono text-xs text-primary">
                  현재 TF: X={livePose.pose.x.toFixed(3)}m, Y={livePose.pose.y.toFixed(3)}m
                  {typeof livePose.pose.yaw === "number" ? ` · yaw ${(livePose.pose.yaw * 180 / Math.PI).toFixed(1)}°` : ""}
                </p>
              )}
              {robot.robot_id === "pinky-1" && (
                <p className="font-mono text-xs text-muted-foreground">
                  {livePose?.battery_valid && typeof livePose.battery_percentage === "number"
                    ? `Telemetry 배터리 ${(livePose.battery_percentage * 100).toFixed(1)}% · ${livePose.battery_source ?? ""}`
                    : `Telemetry 배터리 ${livePose?.battery_freshness === "stale" ? "Stale" : "정보 없음"}`}
                </p>
              )}
              <p className="text-xs text-muted-foreground">
                {robot.kind === "virtual"
                  ? "Physical connection not configured. 등록 위치는 실제 TF가 아닙니다. Task 수신은 가상 기록입니다."
                  : `SSH ${robot.ssh_online ? "Connected" : "끊김"} · ROS ${robot.ros_online || robot.worker?.connection_state === "ros_connected" || robot.worker?.connection_state === "nav2_ready" ? "Connected" : "대기"} · TF ${robot.live_tf_status ?? robot.worker?.tf_status ?? "대기"} · 지도 ${robot.map_matches_fms ? "일치" : "확인 전"}`}
              </p>
              <p className="text-xs text-muted-foreground">
                Task 할당 {robot.task_assignable ? "가능" : "불가"}
                {robot.kind === "physical" ? ` · 실물 주행 ${robot.motion_allowed ? "가능" : "불가"}` : " · 실물 주행 불가"}
                {robot.worker?.detail ? ` · Worker ${robot.worker.detail}` : ""}
              </p>
              {robot.robot_id === "pinky-1" && (
                <div className="rounded-lg border border-border p-3 space-y-2">
                  <p className="text-sm font-semibold">실물 Nav2 테스트 · 5cm 전진 · ROS Domain 24</p>
                  <p className="text-xs text-muted-foreground">Nav2 DriveOnHeading으로 0.05m 직진합니다. 이동 거리는 map TF와 odometry를 함께 기록합니다. Task 취소와는 별도입니다.</p>
                  <div className="flex flex-wrap gap-2">
                    <Button type="button" onClick={() => void sendFiveCmAction(false)}
                      disabled={!robot.nav2_ready || (livePose?.five_cm_blockers?.length ?? 0) > 0 || !livePose || fiveCmPending || ["checking", "pose_verified", "accepted", "running", "cancelling"].includes(fiveCmStatus?.state ?? "")}>
                      Pinky 1 · 5cm 전진 테스트
                    </Button>
                    <Button type="button" variant="outline" onClick={() => void sendFiveCmAction(true)}
                      disabled={fiveCmPending || !["checking", "pose_verified", "accepted", "running"].includes(fiveCmStatus?.state ?? "")}>
                      테스트 Goal 취소
                    </Button>
                    <Button type="button" variant="outline" onClick={() => void diagnoseVelocityChain()}>
                      속도 체인 진단
                    </Button>
                  </div>
                  <p className="text-xs">Nav2 {robot.nav2_ready ? "READY" : "준비 대기"} · 이 표시만으로 5cm를 시작할 수 없습니다.</p>
                  <p className="text-xs">실시간 TF {livePose?.live_tf_status ?? "조회 전"}
                    {typeof livePose?.live_tf_age_s === "number" ? ` · ${livePose.live_tf_age_s.toFixed(1)}s` : ""}
                    {livePose?.pose_checked_at ? ` · ${livePose.pose_checked_at}` : ""}
                    {livePose?.live_tf_withheld_reason ? ` · ${livePose.live_tf_withheld_reason}` : ""}</p>
                  <p className="text-xs">Collision Monitor {livePose?.collision_monitor_active ? "active" : "확인 전 또는 비활성"}</p>
                  <p className="text-xs">지도 {livePose?.map_matches_fms ? "FMS 1f_map_f와 일치" : (livePose?.map_mismatch_detail ?? "확인 전")}</p>
                  {(livePose?.five_cm_blockers?.length ?? 0) > 0 && (
                    <p className="text-xs text-destructive">5cm 시작 불가: {livePose?.five_cm_blockers?.join("; ")}</p>
                  )}
                  <p className="text-xs">{["checking", "pose_verified", "accepted", "running", "cancelling"].includes(fiveCmStatus?.state ?? "") ? "현재 테스트" : "마지막 결과"}: {fiveCmStatus?.state ?? "조회 중"}
                    {fiveCmStatus?.goal_status_name ? ` · Goal ${fiveCmStatus.goal_status_name}` : ""}
                    {fiveCmStatus?.finished_at ? ` · ${fiveCmStatus.finished_at}` : ""}</p>
                  <p className="text-xs">요청 거리 5cm
                    {typeof fiveCmStatus?.map_forward_m === "number" || typeof fiveCmStatus?.actual_forward_m === "number"
                      ? ` · map TF ${((fiveCmStatus.map_forward_m ?? fiveCmStatus.actual_forward_m ?? 0) * 100).toFixed(1)}cm` : ""}
                    {typeof fiveCmStatus?.odom_forward_m === "number" ? ` · odometry ${(fiveCmStatus.odom_forward_m * 100).toFixed(1)}cm` : ""}
                    {fiveCmStatus?.velocity_case ? ` · 속도 케이스 ${fiveCmStatus.velocity_case}` : ""}</p>
                  {fiveCmStatus?.detail && <p className="text-xs">원인: {fiveCmStatus.detail}</p>}
                  {chainDiagnosis && (
                    <p className="text-xs">Collision Monitor 진단: {chainDiagnosis.stop_zone_would_stop ? "StopZone 안에 반환이 있어 정지 가능" : "이번 무주행 표본에서는 StopZone 정지 조건 미만"}
                      {chainDiagnosis.scan?.received ? ` · scan ${chainDiagnosis.scan.frame_id ?? ""} 최소 ${chainDiagnosis.scan.min_range_m?.toFixed(3) ?? "-"}m · 유효 ${chainDiagnosis.scan.range_min_m?.toFixed(3) ?? "-"}–${chainDiagnosis.scan.range_max_m?.toFixed(2) ?? "-"}m · 반경 내 ${chainDiagnosis.scan.points_within_stop_radius ?? 0}점 · 오른쪽 ${chainDiagnosis.scan.inside_sectors_base_frame?.right ?? 0}` : " · scan 없음"}
                      {chainDiagnosis.scan?.right_returns_base_m?.[0] ? ` · 가장 가까운 오른쪽 (${chainDiagnosis.scan.right_returns_base_m[0].x.toFixed(3)}, ${chainDiagnosis.scan.right_returns_base_m[0].y.toFixed(3)})m` : ""}
                      {chainDiagnosis.scan?.self_reflection_proven === false ? " · 자체 반사 여부는 확인되지 않음" : ""}
                      {chainDiagnosis.detail ? ` · ${chainDiagnosis.detail}` : ""}</p>
                  )}
                  {fiveCmError && <p className="text-xs text-destructive">{fiveCmError}</p>}
                </div>
              )}
              <div className="flex flex-wrap gap-2">
                <Button type="button" variant="outline" disabled title="실제 하드웨어 정지 API가 연결되기 전까지 사용할 수 없습니다.">
                  비상 정지 (미연동)
                </Button>
                <Button type="button" variant="secondary" disabled>정지 래치 해제 (미연동)</Button>
              </div>
            </CardContent>
          </Card>
        ))}
        {!loading && !loadError && (fleet?.robots.length ?? 0) === 0 && (
          <Card>
            <CardHeader>
              <CardTitle>등록된 로봇이 없습니다</CardTitle>
              <CardDescription>config/physical_fleet.json의 실물·가상 로봇 등록을 확인하세요.</CardDescription>
            </CardHeader>
          </Card>
        )}
      </section>

      <section className="grid gap-4 lg:grid-cols-[minmax(0,1.3fr)_minmax(0,0.7fr)]">
        <Card>
          <CardHeader>
            <CardTitle>현장 지도</CardTitle>
            <CardDescription>
              {physicalMap
                ? `${physicalMap.source} · ${physicalMap.width_m} × ${physicalMap.height_m} m · ${physicalMap.resolution} m/px · frame: ${physicalMap.frame_id}`
                : "FMS/config/maps/physical_site/1f_map_f.yaml 및 PGM 원본을 사용합니다."}
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            {physicalMap ? (
              <div className="rounded-lg border border-border overflow-hidden">
                <svg
                  role="img"
                  aria-label="기존 물리 현장 지도, 클릭하면 지도 좌표를 작업 입력칸에 기록"
                  viewBox={`0 0 ${physicalMap.width_px} ${physicalMap.height_px}`}
                  className="block h-auto w-full cursor-crosshair"
                  onClick={(event) => {
                    const rect = event.currentTarget.getBoundingClientRect();
                    if (rect.width <= 0 || rect.height <= 0) return;
                    chooseDestination(
                      ((event.clientX - rect.left) / rect.width) * physicalMap.width_px,
                      ((event.clientY - rect.top) / rect.height) * physicalMap.height_px,
                    );
                  }}
                >
                  <image href={physicalMap.image_url} width={physicalMap.width_px} height={physicalMap.height_px} />
                  {navPlanPolylines.map((path) => (
                    <polyline
                      key={`nav-plan-${path.robotId}`}
                      points={path.points}
                      fill="none"
                      stroke={path.color}
                      strokeWidth={1.0}
                      strokeDasharray="4 2"
                      opacity={path.stale ? 0.4 : 0.85}
                    />
                  ))}
                  {routeOverlay && [routeOverlay.candidates, [routeOverlay.selected]].map((group, groupIndex) => group.map((path, pathIndex) => {
                    const color = VIRTUAL_COLOR[routeOverlay.robotId ?? ""] ?? "#526173";
                    const points = path.map((point) => {
                      const pixel = worldToPixel(physicalMap, point.x, point.y);
                      return `${pixel.px},${pixel.py}`;
                    }).join(" ");
                    if (!points) return null;
                    const selectedPath = groupIndex === 1;
                    const historical = Boolean(routeOverlay.historical);
                    return (
                      <polyline key={`${groupIndex}-${pathIndex}`} points={points} fill="none"
                        stroke={historical ? "#526173" : color}
                        strokeWidth={selectedPath ? 1.4 : 0.7}
                        strokeDasharray={historical ? "3 2" : selectedPath ? undefined : "2 1.2"}
                        opacity={historical ? 0.55 : selectedPath ? 1 : 0.75} />
                    );
                  }))}
                  {routeOverlay?.selected.map((point) => {
                    const pixel = worldToPixel(physicalMap, point.x, point.y);
                    const color = VIRTUAL_COLOR[routeOverlay.robotId ?? ""] ?? "#526173";
                    return <circle key={point.waypoint_id ?? `${point.x}-${point.y}`} cx={pixel.px} cy={pixel.py} r={1.1} fill={color} />;
                  })}
                  {selected && (
                    <g aria-label="선택된 좌표(아직 주행 명령 아님)">
                      <circle cx={selected.px} cy={selected.py} r={4} fill="none" stroke="#e34d57" strokeWidth={1.5} />
                      <path d={`M ${selected.px - 7} ${selected.py} h 14 M ${selected.px} ${selected.py - 7} v 14`}
                        stroke="#e34d57" strokeWidth={0.8} />
                    </g>
                  )}
                  {initialMarker && (
                    <g aria-label="Pinky 1 최초 설정 위치(기준점)">
                      <circle cx={initialMarker.px} cy={initialMarker.py} r={2.5}
                        fill="#ffffff" stroke="#526173" strokeWidth={0.9} strokeDasharray="1.5 1" />
                    </g>
                  )}
                  {liveMarkers.filter((m) => m.robot.robot_id === "pinky-1" && initialMarker !== null).map(({robot, px, py}) => (
                    <line key={`${robot.robot_id}-offset`} x1={initialMarker!.px} y1={initialMarker!.py}
                      x2={px} y2={py} stroke="#007d82" strokeWidth={0.8} strokeDasharray="2 1" opacity={0.8} />
                  ))}
                  {homeMarkers.map(({ robot, px, py, rotation }) => {
                    const color = VIRTUAL_COLOR[robot.robot_id] ?? "#526173";
                    return (
                      <g key={`${robot.robot_id}-home`} aria-label={`${robot.name} 가상 등록 위치`}>
                        <g transform={`translate(${px} ${py}) rotate(${rotation})`}>
                          <rect x={-5} y={-5} width={10} height={10} fill={color} fillOpacity={0.18} stroke={color} strokeWidth={0.8} />
                          <path d="M 0 -4 L 0 3 M -2.2 1.2 L 0 3.4 L 2.2 1.2" fill="none" stroke={color} strokeWidth={0.8} />
                        </g>
                        <text x={Math.min(px + 8, physicalMap.width_px - 4)} y={Math.max(py - 8, 10)}
                          fontSize={5.5} fill={color} stroke="#ffffff" strokeWidth={1.5} paintOrder="stroke"
                          textAnchor={px + 8 < physicalMap.width_px - 28 ? "start" : "end"}>
                          {robot.name} 가상
                        </text>
                      </g>
                    );
                  })}
                  {liveMarkers.map(({ robot, px, py, rotation, stale }) => (
                    <g key={robot.robot_id} aria-label={`${robot.name} ${stale ? "마지막 amcl 위치(stale)" : "현재 amcl 위치"}`} opacity={stale ? 0.45 : 1}>
                      {/* Distinct from the red destination crosshair: teal arrow + label. */}
                      <circle cx={px} cy={py} r={7} fill="#09b8b4" fillOpacity={0.2} stroke="#007d82" strokeWidth={0.7} />
                      <g transform={`translate(${px} ${py}) rotate(${rotation})`}>
                        <path d="M 0 -6 L -4 4 L 0 2 L 4 4 Z" fill="#007d82" stroke="#ffffff" strokeWidth={0.65} />
                      </g>
                      <text x={Math.min(px + 9, physicalMap.width_px - 4)} y={Math.max(py - 9, 10)}
                        fontSize={5.5} fill="#003e45" stroke="#ffffff" strokeWidth={1.5} paintOrder="stroke"
                        textAnchor={px + 9 < physicalMap.width_px - 24 ? "start" : "end"}>
                        {stale ? `${robot.name} stale` : robot.name}
                      </text>
                    </g>
                  ))}
                </svg>
              </div>
            ) : (
              <p className="text-sm text-destructive">{mapError ?? "실제 지도 로딩 중..."}</p>
            )}
            <p className="text-xs text-muted-foreground">
              청록색 화살표: Pinky-1의 최근 TF(map → base_footprint). 주황·보라 사각형: Pinky-2·3 가상 등록 위치.
              빨간 십자: 선택한 목적지(주행 미실행). 선은 계획 경로이며 예약 시각과 실제 자세는 따로 표시합니다. 로봇 마커는 계획 경로를 따라 움직이지 않습니다.
              TF가 미수신·오래됨·지도 밖이면 실물 표시는 숨깁니다.
              {routeOverlay?.historical ? " 회색 점선은 종료된 Task의 저장 경로이며 현재 활성 예약이 아닙니다." : ""}
            </p>
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>작업 접수</CardTitle>
            <CardDescription>경로 예약과 Task 전달은 분리됩니다. 전달해도 Nav2 Goal은 보내지 않습니다.</CardDescription>
          </CardHeader>
          <CardContent>
            <form className="flex flex-col gap-3" onSubmit={(event) => event.preventDefault()}>
              <div className="grid gap-2">
                <Label htmlFor="label">목적지 이름</Label>
                <Input id="label" value={label} onChange={(event) => setLabel(event.target.value)} placeholder="실제 목적지 이름" />
              </div>
              <div className="grid grid-cols-2 gap-2">
                <div className="grid gap-2">
                  <Label htmlFor="x">X (m)</Label>
                  <Input id="x" value={x} onChange={(event) => setX(event.target.value)} inputMode="decimal" />
                </div>
                <div className="grid gap-2">
                  <Label htmlFor="y">Y (m)</Label>
                  <Input id="y" value={y} onChange={(event) => setY(event.target.value)} inputMode="decimal" />
                </div>
              </div>
              <p className="font-mono text-xs text-muted-foreground">
                {selectedGrid
                  ? `grid col ${selectedGrid.col.toFixed(1)}, row ${selectedGrid.row.toFixed(1)} · ${physicalMap?.grid?.convention ?? "nav2_occupancy"}`
                  : "지도를 클릭하면 map 좌표와 grid 좌표가 함께 표시됩니다."}
              </p>
              {goalWarning && <p className="text-xs text-destructive">{goalWarning}</p>}
              <FleetTaskPanel x={x} y={y} label={label} robotId={robotId} onLabel={setLabel} onRobot={setRobotId}
                onGoal={(nextX, nextY) => { setX(nextX); setY(nextY); setGoalWarning(null); }}
                onOverlay={setRouteOverlay} />
            </form>
          </CardContent>
        </Card>
      </section>

      <section className="grid gap-4 lg:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>작업 대기열 · 실행 중</CardTitle>
            <CardDescription>Fleet task 대기열은 오른쪽 작업 접수 패널에 표시합니다. 아래 목록은 기존 Mock 작업이며 비어 있습니다.</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-3">
            {active.length === 0 && <p className="text-sm text-muted-foreground">기존 Mock 작업 목록은 비어 있습니다. Fleet task는 위 작업 접수 패널의 대기열에 표시됩니다.</p>}
            {active.map((task) => (
              <article key={task.task_id} className="rounded-lg border border-border p-3">
                <div className="flex items-center justify-between">
                  <p>{task.destination.label} · {task.robot_id ?? "미배정"}</p>
                  <Badge variant={statusVariant(task.status)}>{task.status}</Badge>
                </div>
              </article>
            ))}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>구역 점유</CardTitle>
            <CardDescription>기존 표시 영역 유지 · 실제 구역/예약 상태 미연동</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-2">
            {zones.length === 0 && <p className="text-sm text-muted-foreground">실물 지도 기반 구역 데이터가 없어 표시하지 않습니다.</p>}
            {zones.map((zone) => (
              <div key={zone.zone_id} className="flex justify-between rounded-lg border border-border p-2">
                <span>{zone.name}</span>
                <Badge variant="outline">{zone.reservation?.reservation_status ?? "비어 있음"}</Badge>
              </div>
            ))}
          </CardContent>
        </Card>
      </section>

      <section className="grid gap-4 lg:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>최근 종료</CardTitle>
            <CardDescription>실제 Nav2 결과 기반 종료 기록만 표시할 예정</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-2">
            {history.length === 0 && <p className="text-sm text-muted-foreground">검증된 실물 주행 종료 기록이 없습니다.</p>}
            {history.map((task) => (
              <div key={task.task_id} className="flex justify-between text-sm">
                <span>{task.destination.label}</span>
                <Badge variant={statusVariant(task.status)}>{task.status}</Badge>
              </div>
            ))}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>이벤트</CardTitle>
            <CardDescription>실물 SSH 상태 확인 결과 · Mock 이벤트 제외</CardDescription>
          </CardHeader>
          <CardContent className="max-h-80 space-y-2 overflow-auto">
            {events.length === 0 && <p className="text-sm text-muted-foreground">표시할 실물 연결 이벤트가 없습니다.</p>}
            {events.map((item, index) => (
              <div key={`${item.created_at}-${index}`} className="border-b border-border pb-2 text-xs">
                <p className="font-medium">{item.code} <span className="text-muted-foreground">{item.kind}</span></p>
                <p className="text-muted-foreground">{item.message}</p>
              </div>
            ))}
          </CardContent>
        </Card>
      </section>

      <div className="flex flex-wrap items-center justify-between gap-2 text-xs text-muted-foreground">
        <p>마지막 SSH 확인: {fleet?.checked_at ? new Date(fleet.checked_at).toLocaleString("ko-KR") : "연결 확인 전"}</p>
        <Button type="button" variant="outline" size="sm" onClick={() => void refresh()}>새로고침</Button>
      </div>
    </main>
  );
}

// FMS_LIVE_TF_POSE_OVERLAY_V1
