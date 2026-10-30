"use client";

import { useCallback, useEffect, useState } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";

const ROBOT_COLOR: Record<string, string> = {
  "pinky-1": "#007d82",
  "pinky-2": "#c47b00",
  "pinky-3": "#5b4db7",
};

export type PathWaypoint = { x: number; y: number; waypoint_id?: string };
export type RouteOverlay = {
  robotId: string | null;
  selected: PathWaypoint[];
  candidates: PathWaypoint[][];
  waits: PathWaypoint[];
  historical?: boolean;
};

type Facility = { id: string; name: string; x: number | null; y: number | null; selectable: boolean; detail: string | null };
type Availability = {
  robot_id: string;
  kind: string;
  assignable: boolean;
  reason: string;
  queue_length: number;
  ssh_online?: boolean;
  inspection?: { suggestions?: { x: number; y: number }[] } | null;
};
type FleetTask = {
  task_id: string;
  status: string;
  priority: number;
  assignment_mode: string;
  requested_robot_id?: string | null;
  assigned_robot_id: string | null;
  goal: { x: number; y: number; label: string };
  selected_path_id: string | null;
  planned_path: PathWaypoint[];
  candidate_paths: { path_id: string; rank: number; length_m: number; cost: number; waypoints: PathWaypoint[] }[];
  estimated_start_time: string | null;
  estimated_arrival_time: string | null;
  delivery_status: string;
  ack_status: string;
  ack_channel: string;
  ack: { receiver?: string; accepted?: boolean; payload_hash?: string } | null;
  predecessor_task_id?: string | null;
  start?: { x: number; y: number; source: string } | null;
  assignment_reason: string | null;
  failure_reason: string | null;
  motion_verified?: boolean;
  cancellation?: {
    outcome?: string;
    reservations_released?: number;
    receiver_error?: string | null;
    receiver?: { kind?: string; channel?: string; accepted?: boolean } | null;
  } | null;
  is_archived?: boolean;
  archived_at?: string | null;
  archived_by?: string;
  active_reservation_count?: number;
  schedule: {
    depart_s?: number;
    conflicts_avoided_by_delay?: boolean;
    collision_safety_verified?: boolean;
    time_basis?: string;
    needs_replan?: boolean;
    replan_attempts?: number;
    options?: { depart_s: number; arrival_epoch: number; travel_s: number; selected?: boolean }[];
  };
  actual_motion_verified: boolean;
  assigned_ros_domain_id?: number | null;
  transport?: {
    mode?: string;
    ros_domain_id?: number | null;
    command_topic?: string;
    ack_topic?: string;
    cancel_topic?: string;
    cancel_ack_topic?: string;
    publish_state?: string;
    receiver_ready?: boolean | null;
    received?: boolean;
    executed?: boolean;
    completed?: boolean;
  };
};

type Scope = "active" | "history" | "archived";
const ACTIVE_STATUSES = ["RESERVED", "WAITING", "DISPATCHING", "ACKNOWLEDGED", "CANCEL_PENDING"];
const HISTORY_STATUSES = ["CANCELLED", "FAILED", "UNREACHABLE"];
const FINISHED = new Set(["CANCELLED", "FAILED", "UNREACHABLE"]);
const CANCELLABLE = new Set(["FAILED", "CANCELLED"]);

function needsPhysicalAck(task: FleetTask): boolean {
  return task.ack_channel === "physical" && ["DELIVERED", "ACKNOWLEDGED", "CANCEL_PENDING"].includes(task.status);
}

async function readBody(response: Response): Promise<Record<string, unknown>> {
  try {
    return await response.json() as Record<string, unknown>;
  } catch {
    return {};
  }
}

export function FleetTaskPanel({
  x, y, label, robotId, onLabel, onRobot, onGoal, onOverlay,
}: {
  x: string;
  y: string;
  label: string;
  robotId: string;
  onLabel: (value: string) => void;
  onRobot: (value: string) => void;
  onGoal: (x: string, y: string) => void;
  onOverlay: (overlay: RouteOverlay | null) => void;
}) {
  const [mode, setMode] = useState<"manual" | "automatic">("manual");
  const [priority, setPriority] = useState("5");
  const [taskType, setTaskType] = useState("navigate");
  const [facilities, setFacilities] = useState<Facility[]>([]);
  const [availability, setAvailability] = useState<Availability[]>([]);
  const [tasks, setTasks] = useState<FleetTask[]>([]);
  const [total, setTotal] = useState(0);
  const [scope, setScope] = useState<Scope>("active");
  const [statusFilter, setStatusFilter] = useState("");
  const [robotFilter, setRobotFilter] = useState("");
  const [draftQuery, setDraftQuery] = useState("");
  const [query, setQuery] = useState("");
  const [page, setPage] = useState(1);
  const [pageSize, setPageSize] = useState(10);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [selected, setSelected] = useState<FleetTask | null>(null);
  const [cancelTarget, setCancelTarget] = useState<FleetTask | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [pending, setPending] = useState(false);

  const refresh = useCallback(async () => {
    const params = new URLSearchParams({
      scope,
      page: String(page),
      page_size: String(pageSize),
    });
    if (statusFilter) params.set("status", statusFilter);
    if (robotFilter) params.set("robot_id", robotFilter);
    if (query) params.set("q", query);
    const [taskResponse, facilityResponse, availabilityResponse] = await Promise.all([
      fetch(`/fms-api/fleet-tasks/browse?${params}`, { cache: "no-store" }),
      fetch("/fms-api/fleet-tasks/facilities", { cache: "no-store" }),
      fetch("/fms-api/fleet-tasks/availability", { cache: "no-store" }),
    ]);
    if (taskResponse.ok) {
      const body = await taskResponse.json() as { tasks: FleetTask[]; total: number };
      const rows = body.tasks ?? [];
      if (page > 1 && rows.length === 0 && (body.total ?? 0) > 0) {
        setPage(Math.max(1, Math.ceil(body.total / pageSize)));
        return;
      }
      setTasks(rows);
      setTotal(body.total ?? 0);
    }
    if (facilityResponse.ok) {
      const body = await facilityResponse.json() as { facilities: Facility[] };
      setFacilities(body.facilities ?? []);
    }
    if (availabilityResponse.ok) {
      const body = await availabilityResponse.json() as { robots: Availability[] };
      setAvailability(body.robots ?? []);
    }
    if (selectedId) {
      const one = await fetch(`/fms-api/fleet-tasks/${selectedId}`, { cache: "no-store" });
      if (one.ok) setSelected(await one.json() as FleetTask);
    }
  }, [page, pageSize, query, robotFilter, scope, selectedId, statusFilter]);

  useEffect(() => {
    void refresh();
    const timer = window.setInterval(() => void refresh(), 5000);
    return () => window.clearInterval(timer);
  }, [refresh]);

  useEffect(() => {
    if (!selected) {
      onOverlay(null);
      return;
    }
    const historical = FINISHED.has(selected.status);
    const hidePath = selected.status === "CANCEL_PENDING";
    onOverlay({
      robotId: selected.assigned_robot_id,
      historical,
      selected: hidePath ? [] : (selected.planned_path ?? []),
      candidates: hidePath || historical ? [] : (selected.candidate_paths ?? [])
        .filter((path) => path.path_id !== selected.selected_path_id)
        .map((path) => path.waypoints),
      waits: hidePath || historical || !selected.schedule?.conflicts_avoided_by_delay
        ? []
        : (selected.planned_path.slice(0, 1) ?? []),
    });
  }, [selected, onOverlay]);

  function show(task: FleetTask) {
    setSelectedId(task.task_id);
    setSelected(task);
  }

  async function createAndPlan() {
    setPending(true);
    setError(null);
    setMessage(null);
    const goalX = Number(x);
    const goalY = Number(y);
    if (!Number.isFinite(goalX) || !Number.isFinite(goalY)) {
      setError("목적지 좌표가 없습니다. 지도를 클릭하거나 등록된 시설을 선택하세요.");
      setPending(false);
      return;
    }
    const inspected = await fetch("/fms-api/planning/points/inspect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: goalX, y: goalY }),
    });
    const inspection = await readBody(inspected);
    if (!inspected.ok || inspection.accepted_as_goal !== true) {
      setError(`목적지를 사용할 수 없습니다 (${String(inspection.reason ?? inspection.detail ?? inspected.status)}).`);
      setPending(false);
      return;
    }
    const created = await fetch("/fms-api/fleet-tasks", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        assignment_mode: mode,
        robot_id: mode === "manual" ? robotId : null,
        goal: { x: goalX, y: goalY },
        priority: Number(priority) || 5,
        task_type: taskType,
        label,
      }),
    });
    const createdBody = await readBody(created);
    if (!created.ok) {
      setError(String(createdBody.detail ?? `생성 실패 HTTP ${created.status}`));
      setPending(false);
      return;
    }
    const taskId = String(createdBody.task_id);
    const planned = await fetch(`/fms-api/fleet-tasks/${taskId}/plan`, { method: "POST" });
    const plannedBody = await readBody(planned) as unknown as FleetTask & { detail?: string };
    if (!planned.ok) {
      setError(String(plannedBody.detail ?? `계획 실패 HTTP ${planned.status}`));
      setPending(false);
      await refresh();
      return;
    }
    setSelectedId(taskId);
    setSelected(plannedBody);
    setScope("active");
    setStatusFilter("");
    setPage(1);
    setMessage(plannedBody.status === "RESERVED"
      ? `${plannedBody.assigned_robot_id}에 예약했습니다. 전달 전에는 로봇이 움직이지 않습니다.`
      : `계획 상태 ${plannedBody.status}. ${plannedBody.failure_reason ?? plannedBody.assignment_reason ?? ""}`);
    setPending(false);
    await refresh();
  }

  async function choosePath(task: FleetTask, pathId: string) {
    setPending(true);
    setError(null);
    const response = await fetch(`/fms-api/fleet-tasks/${task.task_id}/select`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path_id: pathId }),
    });
    const body = await readBody(response);
    if (!response.ok) {
      setError(`${String(body.detail ?? "경로 확정 거부")} 재계획이 필요합니다.`);
    } else {
      setSelected(body as FleetTask);
    }
    setPending(false);
    await refresh();
    setSelectedId(task.task_id);
  }

  async function dispatch(task: FleetTask) {
    const target = task.assigned_robot_id ?? "미배정";
    const physical = target === "pinky-1";
    const prompt = physical
      ? `pinky-1에 수신 전용 Task를 전송합니다.\n목적지 ${task.goal.x}, ${task.goal.y}\n경로 ${task.planned_path.length}개 waypoint\n주행 명령은 전송하지 않습니다.`
      : `${target} 가상 수신으로 기록합니다. 실물 ACK가 아닙니다.`;
    if (!window.confirm(prompt)) return;
    setPending(true);
    setError(null);
    const response = await fetch(`/fms-api/fleet-tasks/${task.task_id}/dispatch`, { method: "POST" });
    const body = await readBody(response) as unknown as FleetTask & { detail?: string };
    if (!response.ok) {
      setError(String(body.detail ?? `전달 실패 HTTP ${response.status}`));
    } else if (body.ack_channel === "physical" && body.ack_status === "ACKNOWLEDGED") {
      setMessage(`실물 수신 ACK · ${body.ack?.receiver ?? ""} · hash ${body.ack?.payload_hash?.slice(0, 12) ?? ""}`);
    } else if (body.ack_channel === "simulated") {
      setMessage("가상 수신만 기록했습니다. 실물 ACK가 아닙니다.");
    } else {
      setMessage(`전달 결과 ${body.status}. ${body.failure_reason ?? ""}`);
    }
    setPending(false);
    await refresh();
  }

  function openCancel(task: FleetTask) {
    if (pending || CANCELLABLE.has(task.status)) return;
    show(task);
    setCancelTarget(task);
  }

  async function cancel(task: FleetTask) {
    if (pending) return;
    setPending(true);
    setError(null);
    const response = await fetch(`/fms-api/fleet-tasks/${task.task_id}/cancel`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ reason: "operator" }),
    });
    const body = await readBody(response) as unknown as FleetTask & { detail?: string };
    if (!response.ok) {
      setError(String(body.detail ?? `취소 실패 HTTP ${response.status}`));
      setPending(false);
      return;
    }
    if (body.status === "CANCEL_PENDING") {
      setError(body.cancellation?.receiver_error ?? "Receiver 취소 ACK가 없어 취소를 보류했습니다.");
      setMessage("취소 보류. 자동으로 성공 처리하지 않았습니다.");
    } else if (body.cancellation?.receiver?.channel === "physical" && body.cancellation.receiver.kind === "cancel_ack") {
      setMessage(`실물 취소 ACK · 예약 해제 ${body.cancellation.reservations_released ?? 0}건`);
    } else if (body.cancellation?.receiver?.channel === "simulated") {
      setMessage(`가상 수신 기록만 취소했습니다. 예약 해제 ${body.cancellation.reservations_released ?? 0}건`);
    } else {
      setMessage(`취소됨 · 예약 해제 ${body.cancellation?.reservations_released ?? 0}건`);
    }
    setSelected(body);
    setPending(false);
    setCancelTarget(null);
    await refresh();
    setSelectedId(task.task_id);
  }

  async function setHidden(task: FleetTask, hidden: boolean) {
    if (pending) return;
    setPending(true);
    setError(null);
    const response = await fetch(`/fms-api/fleet-tasks/${task.task_id}/${hidden ? "archive" : "restore"}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ archived_by: "operator" }),
    });
    const body = await readBody(response) as unknown as FleetTask & { detail?: string };
    if (!response.ok) {
      setError(String(body.detail ?? `보관 변경 실패 HTTP ${response.status}`));
    } else {
      setMessage(hidden ? "목록에서 숨겼습니다. Task와 ACK 기록은 유지됩니다." : "숨긴 Task를 이력에 다시 표시합니다.");
      setSelected(body);
    }
    setPending(false);
    await refresh();
  }

  async function approveHome(robot: Availability) {
    const suggestion = robot.inspection?.suggestions?.[0];
    if (!suggestion) {
      setError(`${robot.robot_id}의 대체 좌표 제안이 없습니다.`);
      return;
    }
    if (!window.confirm(`${robot.robot_id} 등록 위치를 ${suggestion.x}, ${suggestion.y}로 바꿀까요?`)) return;
    const response = await fetch(`/fms-api/fleet-tasks/robots/${robot.robot_id}/home`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: suggestion.x, y: suggestion.y, yaw: 0, approve: true }),
    });
    const body = await readBody(response);
    setError(response.ok ? null : String(body.detail ?? "등록 위치 변경 실패"));
    setMessage(response.ok ? `${robot.robot_id} 등록 위치를 검증 후 저장했습니다.` : null);
    await refresh();
  }

  const dispatchable = selected?.status === "RESERVED" && !selected?.schedule?.needs_replan;
  const cancelDisabled = !selected || pending || CANCELLABLE.has(selected.status) || Boolean(selected.is_archived);
  const pageCount = Math.max(1, Math.ceil(total / pageSize));
  const statusOptions = scope === "active" ? ACTIVE_STATUSES : HISTORY_STATUSES;

  return (
    <div className="flex flex-col gap-3">
      <div className="grid gap-2">
        <Label htmlFor="assignment-mode">배정</Label>
        <select id="assignment-mode" className="h-9 rounded-md border border-input bg-background px-2 text-sm"
          value={mode} onChange={(event) => setMode(event.target.value as "manual" | "automatic")}>
          <option value="manual">수동 배정</option>
          <option value="automatic">자동 배정</option>
        </select>
      </div>
      <div className="grid gap-2">
        <Label htmlFor="robot">대상 로봇</Label>
        <select id="robot" className="h-9 rounded-md border border-input bg-background px-2 text-sm"
          value={robotId} disabled={mode === "automatic"} onChange={(event) => onRobot(event.target.value)}>
          <option value="">선택</option>
          <option value="pinky-1">pinky-1 실물</option>
          <option value="pinky-2">pinky-2 가상</option>
          <option value="pinky-3">pinky-3 가상</option>
        </select>
      </div>
      <div className="grid gap-2">
        <Label htmlFor="facility">시설</Label>
        <select id="facility" className="h-9 rounded-md border border-input bg-background px-2 text-sm" defaultValue=""
          onChange={(event) => {
            const facility = facilities.find((item) => item.id === event.target.value);
            if (!facility || !facility.selectable || facility.x === null || facility.y === null) return;
            onLabel(facility.name);
            onGoal(String(facility.x), String(facility.y));
          }}>
          <option value="">지도 클릭 또는 시설 선택</option>
          {facilities.map((facility) => (
            <option key={facility.id} value={facility.id} disabled={!facility.selectable}>
              {facility.selectable ? facility.name : `${facility.name} · 좌표 미등록`}
            </option>
          ))}
        </select>
      </div>
      <div className="grid grid-cols-2 gap-2">
        <div className="grid gap-2">
          <Label htmlFor="priority">우선순위</Label>
          <Input id="priority" value={priority} onChange={(event) => setPriority(event.target.value)} inputMode="numeric" />
        </div>
        <div className="grid gap-2">
          <Label htmlFor="task-type">Task type</Label>
          <Input id="task-type" value={taskType} onChange={(event) => setTaskType(event.target.value)} />
        </div>
      </div>
      <Button type="button" disabled={pending || (mode === "manual" && !robotId)} onClick={() => void createAndPlan()}>
        경로 계산 및 예약
      </Button>
      <p className="text-xs text-muted-foreground">이 버튼은 Task를 만들고 경로를 예약합니다. 로봇에는 전송하지 않습니다.</p>
      {availability.filter((robot) => robot.kind === "virtual" && !robot.assignable).map((robot) => (
        <div key={robot.robot_id} className="rounded-md border border-border p-2 text-xs">
          <p>{robot.robot_id} 초기 위치가 이동 불가 ({robot.reason}). 자동 배정에서 제외됩니다.</p>
          <Button type="button" variant="outline" className="mt-2" onClick={() => void approveHome(robot)}>
            제안 위치 승인
          </Button>
        </div>
      ))}
      {error && <p className="text-xs text-destructive">{error}</p>}
      {message && <p className="text-xs text-primary">{message}</p>}
      <div className="flex flex-col gap-2">
        <p className="text-sm font-medium">Task 목록</p>
        <div className="flex flex-wrap gap-1">
          {([
            ["active", "진행 중"],
            ["history", "완료·취소 이력"],
            ["archived", "숨긴 Task"],
          ] as const).map(([value, labelText]) => (
            <Button key={value} type="button" size="sm" variant={scope === value ? "default" : "outline"}
              onClick={() => { setScope(value); setStatusFilter(""); setPage(1); }}>
              {labelText}
            </Button>
          ))}
        </div>
        <div className="grid grid-cols-2 gap-2">
          <select aria-label="상태 필터" className="h-8 rounded-md border border-input bg-background px-2 text-xs"
            value={statusFilter} onChange={(event) => { setStatusFilter(event.target.value); setPage(1); }}>
            <option value="">{scope === "active" ? "전체 진행 중" : "전체 상태"}</option>
            {statusOptions.map((status) => <option key={status} value={status}>{status}</option>)}
          </select>
          <select aria-label="로봇 필터" className="h-8 rounded-md border border-input bg-background px-2 text-xs"
            value={robotFilter} onChange={(event) => { setRobotFilter(event.target.value); setPage(1); }}>
            <option value="">전체 로봇</option>
            <option value="pinky-1">Pinky-1</option>
            <option value="pinky-2">Pinky-2</option>
            <option value="pinky-3">Pinky-3</option>
          </select>
        </div>
        <div className="flex gap-2">
          <Input aria-label="Task ID 검색" value={draftQuery} placeholder="Task ID"
            onChange={(event) => setDraftQuery(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === "Enter") {
                event.preventDefault();
                setQuery(draftQuery.trim());
                setPage(1);
              }
            }} />
          <Button type="button" variant="outline" onClick={() => { setQuery(draftQuery.trim()); setPage(1); }}>검색</Button>
        </div>
        <p className="text-xs text-muted-foreground">전체 {total}건 · {page} / {pageCount} 페이지</p>
        {tasks.length === 0 && <p className="text-xs text-muted-foreground">이 조건의 Task가 없습니다.</p>}
        {tasks.map((task) => {
          const canCancel = !CANCELLABLE.has(task.status) && !task.is_archived;
          const canHide = FINISHED.has(task.status) && !task.is_archived;
          return (
            <article key={task.task_id} className="rounded-lg border border-border p-2">
              <div className="flex items-center justify-between gap-2">
                <span className="font-mono text-xs" style={{ color: ROBOT_COLOR[task.assigned_robot_id ?? ""] ?? "inherit" }}>
                  {task.assigned_robot_id ?? "미배정"}
                </span>
                <Badge variant="outline">{task.status}</Badge>
              </div>
              <p className="mt-1 text-xs text-muted-foreground">
                {task.goal.label || `${task.goal.x.toFixed(2)}, ${task.goal.y.toFixed(2)}`} · 우선순위 {task.priority}
                · 예약 {task.active_reservation_count ?? 0}건
                · ACK {task.ack_channel === "physical" ? task.ack_status : task.ack_channel === "simulated" ? "가상 수신" : "없음"}
              </p>
              <p className="font-mono text-[10px] text-muted-foreground">{task.task_id}</p>
              <div className="mt-2 flex flex-wrap gap-1">
                <Button type="button" size="xs" variant="outline" onClick={() => show(task)}>상세</Button>
                <Button type="button" size="xs" variant="outline" onClick={() => show(task)}>경로</Button>
                {canCancel && (
                  <Button type="button" size="xs" variant="outline" disabled={pending} onClick={() => openCancel(task)}>
                    {task.status === "CANCEL_PENDING" ? "취소 재시도" : "취소"}
                  </Button>
                )}
                {canHide && (
                  <Button type="button" size="xs" variant="outline" disabled={pending} onClick={() => void setHidden(task, true)}>
                    목록에서 숨기기
                  </Button>
                )}
                {task.is_archived && (
                  <Button type="button" size="xs" variant="outline" disabled={pending} onClick={() => void setHidden(task, false)}>
                    다시 표시
                  </Button>
                )}
              </div>
            </article>
          );
        })}
        <div className="flex items-center justify-between gap-2">
          <Button type="button" size="sm" variant="outline" disabled={page <= 1} onClick={() => setPage((current) => current - 1)}>이전</Button>
          <select aria-label="페이지 크기" className="h-8 rounded-md border border-input bg-background px-2 text-xs"
            value={pageSize} onChange={(event) => { setPageSize(Number(event.target.value)); setPage(1); }}>
            <option value={10}>10건</option>
            <option value={20}>20건</option>
            <option value={50}>50건</option>
          </select>
          <Button type="button" size="sm" variant="outline" disabled={page >= pageCount} onClick={() => setPage((current) => current + 1)}>다음</Button>
        </div>
      </div>
      {selected && (
        <div className="rounded-lg border border-border p-2 text-xs">
          <p className="font-mono">{selected.task_id}</p>
          <p>우선순위 {selected.priority} · {selected.assigned_robot_id ?? "미배정"} · {selected.status}{selected.is_archived ? " · 숨김" : ""}</p>
          <p>이전 작업 {selected.predecessor_task_id ?? "없음"} · 이동 확인 {selected.motion_verified ? "시뮬레이션 확인" : "미확인"}</p>
          <p>전송 {selected.transport?.mode === "simulated" ? "가상 수신" : selected.transport?.mode === "ssh" ? "SSH legacy" : "ROS 2 Topic"}
            {selected.transport?.ros_domain_id != null ? ` · Domain ${selected.transport.ros_domain_id}` : ""}
            {selected.transport?.command_topic ? ` · ${selected.transport.command_topic}` : ""}
          </p>
          <p>Publish {selected.transport?.publish_state ?? "not_sent"} · ACK {selected.transport?.received ? "수신" : "없음"}
            · Receiver {selected.transport?.receiver_ready ? "ready" : "미확인"}
            · 실행 {selected.transport?.executed ? "예" : "아니오"} · 완료 {selected.transport?.completed ? "예" : "아니오"}
          </p>
          <p>시작 {selected.start ? `${selected.start.source} (${selected.start.x.toFixed(2)}, ${selected.start.y.toFixed(2)})` : "미정"}</p>
          <p>목적지 {selected.goal.label || `${selected.goal.x.toFixed(2)}, ${selected.goal.y.toFixed(2)}`}</p>
          <p>계획 경로 {selected.selected_path_id ?? "없음"} · {selected.planned_path.length} waypoint
            {FINISHED.has(selected.status) ? " · 이력 경로" : ""}
            {selected.schedule?.time_basis ? ` · 예약 시각 ${selected.schedule.time_basis}` : ""}
          </p>
          <p>예상 출발(UTC) {selected.estimated_start_time ?? "-"}</p>
          <p>예상 도착(UTC) {selected.estimated_arrival_time ?? "-"}</p>
          <p>{selected.schedule?.needs_replan ? `재계획 필요 · 시도 ${selected.schedule.replan_attempts ?? 0}` : "재계획 대기 아님"}</p>
          <p>{selected.assignment_reason}</p>
          <p>활성 예약 {selected.active_reservation_count ?? 0}건 · 계획 충돌 회피 {selected.schedule?.conflicts_avoided_by_delay ? "대기 후 출발" : "추가 대기 없음"} · 실측 안전 검증 아님</p>
          <p>취소 {selected.status === "CANCEL_PENDING" ? "보류" : selected.status === "CANCELLED" ? "완료" : selected.status === "FAILED" ? "불가" : "가능"}
            {selected.cancellation ? ` · 예약 해제 ${selected.cancellation.reservations_released ?? 0}건 · ${selected.cancellation.outcome ?? ""}` : ""}
          </p>
          {selected.cancellation?.receiver_error && <p className="text-destructive">{selected.cancellation.receiver_error}</p>}
          {selected.failure_reason && <p className="text-destructive">실패 이력: {selected.failure_reason}</p>}
          {selected.status !== "CANCELLED" && selected.status !== "FAILED" && selected.candidate_paths.map((path) => (
            <button key={path.path_id} type="button" className="mt-1 block w-full rounded border border-border px-2 py-1 text-left"
              onClick={() => void choosePath(selected, path.path_id)}>
              {path.path_id} · {path.length_m} m · 비용 {path.cost}
              {selected.schedule?.options?.find((option) => option.selected && path.path_id === selected.selected_path_id)
                ? ` · 도착 +${selected.schedule.options.find((option) => option.selected)?.travel_s.toFixed(1)}s`
                : ""}
              {path.path_id === selected.selected_path_id ? " · 선택됨" : " · 이 경로로 재검증"}
            </button>
          ))}
          <div className="mt-2 flex flex-wrap gap-2">
            <Button type="button" disabled={!dispatchable || pending} onClick={() => void dispatch(selected)}>
              Task 전달
            </Button>
            <Button type="button" variant="outline" disabled={cancelDisabled} onClick={() => openCancel(selected)}>
              {selected.status === "CANCEL_PENDING" ? "취소 재시도" : "취소"}
            </Button>
            {FINISHED.has(selected.status) && !selected.is_archived && (
              <Button type="button" variant="outline" disabled={pending} onClick={() => void setHidden(selected, true)}>
                목록에서 숨기기
              </Button>
            )}
            {selected.is_archived && (
              <Button type="button" variant="outline" disabled={pending} onClick={() => void setHidden(selected, false)}>
                다시 표시
              </Button>
            )}
          </div>
        </div>
      )}
      {cancelTarget && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/40 p-4">
          <div role="dialog" aria-modal="true" aria-labelledby="cancel-task-title"
            className="w-full max-w-md rounded-lg border border-border bg-background p-4 text-sm shadow-lg">
            <p id="cancel-task-title" className="font-medium">Task 취소 확인</p>
            <p className="mt-2 font-mono text-xs">{cancelTarget.task_id}</p>
            <p>로봇 {cancelTarget.assigned_robot_id ?? cancelTarget.requested_robot_id ?? "미배정"}</p>
            <p>상태 {cancelTarget.status}</p>
            <p>목적지 {cancelTarget.goal.label || `${cancelTarget.goal.x.toFixed(2)}, ${cancelTarget.goal.y.toFixed(2)}`}</p>
            <p>해제될 활성 예약 {cancelTarget.active_reservation_count ?? 0}건. 다른 Task의 예약은 유지됩니다.</p>
            <p>{needsPhysicalAck(cancelTarget)
              ? "실물 Receiver 취소 ACK가 확인되기 전에는 취소 완료로 표시하지 않습니다."
              : "실물 Receiver 취소 ACK는 필요하지 않습니다."}
            </p>
            <div className="mt-3 flex gap-2">
              <Button type="button" disabled={pending} onClick={() => void cancel(cancelTarget)}>취소 요청</Button>
              <Button type="button" variant="outline" disabled={pending} onClick={() => setCancelTarget(null)}>닫기</Button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
