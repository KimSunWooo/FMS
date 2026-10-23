"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";

type Pose = { x: number; y: number; yaw: number };
type Robot = {
  robot_id: string;
  name: string;
  mode: string;
  connection_status: string;
  operational_state: string;
  battery_percent: number;
  controller_kind: string;
  controller_ready: boolean;
  pose: Pose;
  pose_trustworthy: boolean;
  pose_source: string;
  fault_code: string | null;
  estop_latched: boolean;
  agent_online: boolean;
  active_task_id: string | null;
};
type Fleet = {
  environment_id: string;
  physical_control: string;
  rmf: string;
  ros_bridge: string;
  ros_domain_id: number;
  distance_model: string;
  gateway: { implementation: string; online: boolean; boot_id: string };
  robots: Robot[];
  task_counts: Record<string, number>;
};
type Task = {
  task_id: string;
  status: string;
  robot_id: string | null;
  destination: { x: number; y: number; yaw: number; label: string };
  owner_epoch: number;
  goal_id: string | null;
  execution_id: string | null;
  state_version: number;
  failure_reason: string | null;
  recovery_reason: string | null;
  navigation_goal_count: number;
  completion_evidence: { goal_status: string; distance_m: number; goal_id: string } | null;
};
type Zone = {
  zone_id: string;
  name: string;
  kind: string;
  bounds: { x: number; y: number; w: number; h: number };
  reservation: {
    reservation_id: string;
    robot_id: string;
    reservation_status: string;
    hold_reason: string;
  } | null;
};
type Floor = { width_m: number; height_m: number; note: string };
type FeedEvent = { kind: string; code: string; message: string; created_at: string | null; task_id?: string };
type Health = { status: string; motion_backend: string; gateway_boot_id: string; database: string };

const PRESETS = [
  { label: "개방 구역", x: 5, y: 6.5 },
  { label: "복도", x: 3, y: 3.6 },
  { label: "교차로", x: 8, y: 4 },
  { label: "엘리베이터 대기", x: 10, y: 3.7 },
  { label: "도킹", x: 1.2, y: 1.2 },
];

const ROBOT_COLOR: Record<string, string> = {
  "pinky-1": "#2ec4b6",
  "pinky-2": "#e2b340",
  "pinky-3": "#79b8ff",
};

async function readJson(path: string, init?: RequestInit) {
  const response = await fetch(path, {
    ...init,
    headers: { "Content-Type": "application/json", ...(init?.headers ?? {}) },
  });
  const text = await response.text();
  const body = text ? JSON.parse(text) : null;
  if (!response.ok) {
    const message = body?.error?.message || body?.detail || response.statusText;
    throw new Error(typeof message === "string" ? message : "요청이 거절됐습니다.");
  }
  return body;
}

function statusVariant(status: string): "default" | "secondary" | "destructive" | "outline" {
  if (status === "COMPLETED" || status === "online" || status === "idle") return "default";
  if (status === "FAILED" || status === "RECOVERY_REQUIRED" || status === "fault" || status === "offline") {
    return "destructive";
  }
  if (status === "RUNNING" || status === "busy") return "secondary";
  return "outline";
}

export function FleetConsole() {
  const [fleet, setFleet] = useState<Fleet | null>(null);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [zones, setZones] = useState<Zone[]>([]);
  const [floor, setFloor] = useState<Floor | null>(null);
  const [events, setEvents] = useState<FeedEvent[]>([]);
  const [health, setHealth] = useState<Health | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [formError, setFormError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);
  const [label, setLabel] = useState("개방 구역");
  const [x, setX] = useState("5");
  const [y, setY] = useState("6.5");
  const [robotId, setRobotId] = useState("");
  const [armedStop, setArmedStop] = useState<string | null>(null);
  const [ack, setAck] = useState<Record<string, boolean>>({});
  const [watchId, setWatchId] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    try {
      const [fleetBody, taskBody, zoneBody, eventBody, healthBody] = await Promise.all([
        readJson("/fms-api/fleet/status"),
        readJson("/fms-api/tasks"),
        readJson("/fms-api/traffic/zones"),
        readJson("/fms-api/fleet/events?limit=40"),
        readJson("/fms-api/system/health"),
      ]);
      setFleet(fleetBody);
      setTasks(taskBody.tasks);
      setZones(zoneBody.zones);
      setFloor(zoneBody.floor);
      setEvents(eventBody.events);
      setHealth(healthBody);
      setLoadError(null);
    } catch (error) {
      setLoadError(error instanceof Error ? error.message : "FMS API에 연결하지 못했습니다.");
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    void refresh();
    const timer = window.setInterval(() => void refresh(), 2000);
    return () => window.clearInterval(timer);
  }, [refresh]);

  const active = useMemo(
    () => tasks.filter((task) => !["COMPLETED", "CANCELED", "FAILED"].includes(task.status)),
    [tasks],
  );
  const history = useMemo(
    () => tasks.filter((task) => ["COMPLETED", "CANCELED", "FAILED"].includes(task.status)).slice(0, 8),
    [tasks],
  );
  const watched = tasks.find((task) => task.task_id === watchId) ?? null;

  async function createTask(event: React.FormEvent) {
    event.preventDefault();
    setFormError(null);
    setNotice(null);
    try {
      const body = await readJson("/fms-api/tasks", {
        method: "POST",
        body: JSON.stringify({
          destination: { x: Number(x), y: Number(y), yaw: 0, label },
          priority: 0,
          robot_id: robotId || null,
        }),
      });
      setWatchId(body.task.task_id);
      setNotice("접수됐습니다. 이 응답은 도착이 아닙니다. 아래 상태가 COMPLETED로 바뀌는지 확인하세요.");
      await refresh();
    } catch (error) {
      setFormError(error instanceof Error ? error.message : "작업을 만들지 못했습니다.");
    }
  }

  async function mutate(path: string, payload?: unknown) {
    setFormError(null);
    try {
      await readJson(path, { method: "POST", body: payload === undefined ? undefined : JSON.stringify(payload) });
      await refresh();
    } catch (error) {
      setFormError(error instanceof Error ? error.message : "요청이 거절됐습니다.");
    }
  }

  return (
    <main className="mx-auto flex min-h-screen max-w-7xl flex-col gap-4 px-4 py-5 md:px-6">
      <header className="flex flex-col gap-3 border-b border-border pb-4 md:flex-row md:items-end md:justify-between">
        <div>
          <p className="text-xs tracking-[0.18em] text-primary uppercase">Care ward · mock fleet</p>
          <h1 className="text-2xl font-semibold">Care-Move FMS</h1>
          <p className="max-w-2xl text-sm text-muted-foreground">
            Pinky #1–#3 관제 화면입니다. 조회는 작업 상태를 바꾸지 않습니다. 이 빌드는 실물 모터와 ROS 명령을
            보내지 않습니다.
          </p>
        </div>
        <div className="flex flex-wrap gap-2 text-xs">
          <Badge variant="outline">RMF {fleet?.rmf ?? "…"}</Badge>
          <Badge variant="outline">실물 {fleet?.physical_control ?? "blocked"}</Badge>
          <Badge variant="outline">ROS {fleet?.ros_bridge ?? "disabled"}</Badge>
          <Badge variant="outline">domain {fleet?.ros_domain_id ?? 73}</Badge>
          <Badge variant={health?.database === "ok" ? "default" : "destructive"}>
            DB {health?.database ?? "…"}
          </Badge>
        </div>
      </header>

      {loading && !fleet && <p className="text-sm text-muted-foreground">함대 상태를 불러오는 중입니다.</p>}
      {loadError && (
        <Card>
          <CardHeader>
            <CardTitle>API에 연결하지 못했습니다</CardTitle>
            <CardDescription>{loadError} 백엔드가 8741에서 실행 중인지 확인하세요.</CardDescription>
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
                {robot.controller_kind === "mock" ? "Mock 제어기 · ROS 미연결" : robot.controller_kind}
              </CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-3">
              <div className="flex flex-wrap gap-1.5">
                <Badge variant={statusVariant(robot.connection_status)}>{robot.connection_status}</Badge>
                <Badge variant={statusVariant(robot.operational_state)}>{robot.operational_state}</Badge>
                <Badge variant="outline">배터리 {Math.round(robot.battery_percent)}%</Badge>
                <Badge variant={robot.agent_online ? "default" : "destructive"}>
                  agent {robot.agent_online ? "up" : "down"}
                </Badge>
              </div>
              <p className="font-mono text-xs text-muted-foreground">
                pose {robot.pose.x.toFixed(2)}, {robot.pose.y.toFixed(2)} ·{" "}
                {robot.pose_trustworthy ? "위치 신뢰" : "위치 불확실"} · {robot.pose_source}
              </p>
              <p className="text-xs text-muted-foreground">
                현재 작업 {robot.active_task_id ? robot.active_task_id.slice(0, 8) : "없음"}
                {robot.fault_code ? ` · fault ${robot.fault_code}` : ""}
              </p>
              <div className="flex flex-wrap gap-2">
                {armedStop === robot.robot_id ? (
                  <Button
                    type="button"
                    variant="destructive"
                    onClick={() => {
                      setArmedStop(null);
                      void mutate(`/fms-api/robots/${robot.robot_id}/emergency-stop`, {
                        reason: "operator estop from dashboard",
                      });
                    }}
                  >
                    정지 확정
                  </Button>
                ) : (
                  <Button type="button" variant="outline" onClick={() => setArmedStop(robot.robot_id)}>
                    비상 정지
                  </Button>
                )}
                {robot.estop_latched && (
                  <Button
                    type="button"
                    variant="secondary"
                    onClick={() => void mutate(`/fms-api/robots/${robot.robot_id}/estop/clear`, { acknowledge: true })}
                  >
                    정지 래치 해제
                  </Button>
                )}
                {!robot.pose_trustworthy && (
                  <Button type="button" variant="secondary" onClick={() => void mutate(`/fms-api/robots/${robot.robot_id}/mock-observe`)}>
                    모의 위치 재확인
                  </Button>
                )}
              </div>
            </CardContent>
          </Card>
        ))}
        {!loading && fleet && fleet.robots.length === 0 && (
          <Card>
            <CardHeader>
              <CardTitle>등록된 로봇이 없습니다</CardTitle>
              <CardDescription>시드가 꺼져 있으면 Pinky 3대가 표시되지 않습니다.</CardDescription>
            </CardHeader>
          </Card>
        )}
      </section>

      <section className="grid gap-4 lg:grid-cols-[minmax(0,1.3fr)_minmax(0,0.7fr)]">
        <Card>
          <CardHeader>
            <CardTitle>병동 스케치</CardTitle>
            <CardDescription>
              {floor?.note ?? "구역 스케치"} · 배차 비용은 {fleet?.distance_model ?? "euclidean"} 이며 장애물 경로가
              아닙니다.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <WardMap robots={fleet?.robots ?? []} zones={zones} tasks={active} width={floor?.width_m ?? 12} height={floor?.height_m ?? 8} />
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>작업 접수</CardTitle>
            <CardDescription>접수 성공과 목적지 도착은 별개입니다.</CardDescription>
          </CardHeader>
          <CardContent>
            <form className="flex flex-col gap-3" onSubmit={createTask}>
              <div className="flex flex-wrap gap-2">
                {PRESETS.map((preset) => (
                  <Button
                    key={preset.label}
                    type="button"
                    size="sm"
                    variant="outline"
                    onClick={() => {
                      setLabel(preset.label);
                      setX(String(preset.x));
                      setY(String(preset.y));
                    }}
                  >
                    {preset.label}
                  </Button>
                ))}
              </div>
              <div className="grid gap-2">
                <Label htmlFor="label">목적지 이름</Label>
                <Input id="label" value={label} onChange={(event) => setLabel(event.target.value)} required />
              </div>
              <div className="grid grid-cols-2 gap-2">
                <div className="grid gap-2">
                  <Label htmlFor="x">X (m)</Label>
                  <Input id="x" value={x} onChange={(event) => setX(event.target.value)} inputMode="decimal" required />
                </div>
                <div className="grid gap-2">
                  <Label htmlFor="y">Y (m)</Label>
                  <Input id="y" value={y} onChange={(event) => setY(event.target.value)} inputMode="decimal" required />
                </div>
              </div>
              <div className="grid gap-2">
                <Label htmlFor="robot">로봇 지정 (비우면 자동 배차)</Label>
                <Input
                  id="robot"
                  placeholder="pinky-1"
                  value={robotId}
                  onChange={(event) => setRobotId(event.target.value)}
                />
              </div>
              <Button type="submit">작업 접수</Button>
              {notice && <p className="text-sm text-primary">{notice}</p>}
              {watched && (
                <p className="font-mono text-xs">
                  추적 {watched.task_id.slice(0, 8)} · {watched.status}
                  {watched.robot_id ? ` · ${watched.robot_id}` : ""}
                  {watched.completion_evidence
                    ? ` · goal ${watched.completion_evidence.goal_status} · ${watched.completion_evidence.distance_m.toFixed(2)} m`
                    : " · 완료 근거 없음"}
                </p>
              )}
              {formError && <p className="text-sm text-destructive">{formError}</p>}
            </form>
          </CardContent>
        </Card>
      </section>

      <section className="grid gap-4 lg:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>작업 대기열 · 실행 중</CardTitle>
            <CardDescription>{active.length === 0 ? "진행 중인 작업이 없습니다." : `${active.length}건`}</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-3">
            {active.length === 0 && (
              <p className="text-sm text-muted-foreground">
                목적지를 지정하면 스케줄러가 온라인 Mock 로봇에 배정합니다. 배정이 거절되면 작업은 PENDING으로 남습니다.
              </p>
            )}
            {active.map((task) => (
              <article key={task.task_id} className="rounded-lg border border-border p-3">
                <div className="flex flex-wrap items-center justify-between gap-2">
                  <div>
                    <p className="font-medium">{task.destination.label}</p>
                    <p className="font-mono text-xs text-muted-foreground">
                      {task.task_id.slice(0, 8)} · {task.robot_id ?? "미배정"} · epoch {task.owner_epoch} · goal{" "}
                      {task.navigation_goal_count}
                    </p>
                  </div>
                  <Badge variant={statusVariant(task.status)}>{task.status}</Badge>
                </div>
                {task.recovery_reason && (
                  <p className="mt-2 text-xs text-destructive">복구 사유: {task.recovery_reason}</p>
                )}
                <div className="mt-3 flex flex-wrap gap-2">
                  {task.status !== "RECOVERY_REQUIRED" && task.status !== "CANCEL_REQUESTED" && (
                    <Button type="button" size="sm" variant="outline" onClick={() => void mutate(`/fms-api/tasks/${task.task_id}/cancel`)}>
                      취소 요청
                    </Button>
                  )}
                  {task.status === "RECOVERY_REQUIRED" && (
                    <>
                      <label className="flex items-center gap-2 text-xs">
                        <input
                          type="checkbox"
                          checked={Boolean(ack[task.task_id])}
                          onChange={(event) => setAck((current) => ({ ...current, [task.task_id]: event.target.checked }))}
                        />
                        위치가 확인되지 않았음을 인정
                      </label>
                      <Button
                        type="button"
                        size="sm"
                        variant="secondary"
                        onClick={() =>
                          void mutate(`/fms-api/tasks/${task.task_id}/resolve`, {
                            outcome: "CANCELED",
                            reason: "operator closed recovery from dashboard",
                            acknowledge_position_unknown: Boolean(ack[task.task_id]),
                          })
                        }
                      >
                        복구 건 취소로 닫기
                      </Button>
                    </>
                  )}
                </div>
              </article>
            ))}
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>구역 점유</CardTitle>
            <CardDescription>만료 시각만으로 구역을 비우지 않습니다. blocked 는 사람이 확인하기 전까지 유지됩니다.</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-2">
            {zones.map((zone) => (
              <div key={zone.zone_id} className="flex items-center justify-between gap-3 rounded-lg border border-border px-3 py-2">
                <div>
                  <p className="text-sm font-medium">
                    {zone.zone_id} · {zone.name}
                  </p>
                  <p className="text-xs text-muted-foreground">{zone.kind}</p>
                </div>
                {zone.reservation ? (
                  <Badge variant={zone.reservation.reservation_status === "blocked" ? "destructive" : "secondary"}>
                    {zone.reservation.reservation_status} · {zone.reservation.robot_id}
                  </Badge>
                ) : (
                  <Badge variant="outline">비어 있음</Badge>
                )}
              </div>
            ))}
          </CardContent>
        </Card>
      </section>

      <section className="grid gap-4 lg:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>최근 종료</CardTitle>
            <CardDescription>COMPLETED 는 Nav 결과와 위치 대조가 있을 때만 표시됩니다.</CardDescription>
          </CardHeader>
          <CardContent className="flex flex-col gap-2">
            {history.length === 0 && <p className="text-sm text-muted-foreground">종료된 작업이 없습니다.</p>}
            {history.map((task) => (
              <div key={task.task_id} className="flex items-start justify-between gap-3 text-sm">
                <div>
                  <p>{task.destination.label}</p>
                  <p className="font-mono text-xs text-muted-foreground">
                    {task.robot_id ?? "-"} · {task.completion_evidence ? `goal ${task.completion_evidence.goal_status}` : task.failure_reason || task.status}
                  </p>
                </div>
                <Badge variant={statusVariant(task.status)}>{task.status}</Badge>
              </div>
            ))}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>이벤트</CardTitle>
            <CardDescription>
              Gateway {fleet?.gateway.online ? "online" : "offline"} · boot {fleet?.gateway.boot_id.slice(0, 8) ?? "-"} ·{" "}
              {fleet?.gateway.implementation}
            </CardDescription>
          </CardHeader>
          <CardContent className="max-h-80 space-y-2 overflow-auto">
            {events.length === 0 && <p className="text-sm text-muted-foreground">아직 이벤트가 없습니다.</p>}
            {events.map((item, index) => (
              <div key={`${item.created_at}-${index}`} className="border-b border-border pb-2 text-xs">
                <p className="font-medium">
                  {item.code} <span className="text-muted-foreground">{item.kind}</span>
                </p>
                <p className="text-muted-foreground">{item.message}</p>
              </div>
            ))}
          </CardContent>
        </Card>
      </section>
    </main>
  );
}

function WardMap({
  robots,
  zones,
  tasks,
  width,
  height,
}: {
  robots: Robot[];
  zones: Zone[];
  tasks: Task[];
  width: number;
  height: number;
}) {
  const scale = 100;
  const sy = (y: number) => (height - y) * scale;
  return (
    <svg viewBox={`0 0 ${width * scale} ${height * scale}`} className="h-auto w-full rounded-lg bg-[#101820]">
      {zones.map((zone) => {
        const held = zone.reservation?.reservation_status === "held";
        const blocked = zone.reservation?.reservation_status === "blocked";
        return (
          <g key={zone.zone_id}>
            <rect
              x={zone.bounds.x * scale}
              y={sy(zone.bounds.y + zone.bounds.h)}
              width={zone.bounds.w * scale}
              height={zone.bounds.h * scale}
              fill={blocked ? "rgba(240,113,120,0.35)" : held ? "rgba(226,179,64,0.28)" : "rgba(46,196,182,0.12)"}
              stroke={blocked ? "#f07178" : held ? "#e2b340" : "#2ec4b6"}
              strokeWidth={2}
            />
            <text x={zone.bounds.x * scale + 8} y={sy(zone.bounds.y + zone.bounds.h) + 18} fill="#d5dde6" fontSize={14}>
              {zone.name}
            </text>
          </g>
        );
      })}
      {tasks.map((task) => (
        <circle
          key={task.task_id}
          cx={task.destination.x * scale}
          cy={sy(task.destination.y)}
          r={5}
          fill="none"
          stroke="#f07178"
          strokeDasharray="3 3"
        />
      ))}
      {robots.map((robot) => (
        <g key={robot.robot_id}>
          <circle
            cx={robot.pose.x * scale}
            cy={sy(robot.pose.y)}
            r={10}
            fill={ROBOT_COLOR[robot.robot_id] ?? "#ffffff"}
            stroke={robot.pose_trustworthy ? "#081018" : "#f07178"}
            strokeDasharray={robot.pose_trustworthy ? undefined : "2 2"}
            strokeWidth={2}
          />
          <text x={robot.pose.x * scale + 14} y={sy(robot.pose.y) + 4} fill="#f4f7fb" fontSize={13}>
            {robot.name}
          </text>
        </g>
      ))}
    </svg>
  );
}
