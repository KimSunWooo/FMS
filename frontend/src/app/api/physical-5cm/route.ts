/**
 * Server-only same-origin local FMS UI proxy.
 * Browser never receives X-FMS-Operator-Token. Backend retains full authorization.
 */
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { NextRequest, NextResponse } from "next/server";

export const runtime = "nodejs";
export const dynamic = "force-dynamic";

function jsonError(status: number, detail: string) {
  return NextResponse.json({ detail }, { status, headers: { "Cache-Control": "no-store" } });
}

export async function POST(request: NextRequest) {
  // Only the locally opened FMS dashboard is allowed to invoke this route.
  // Never expose this dev UI on the LAN without separate operator login.
  const host = request.headers.get("host") || "";
  if (!/^(localhost|127\.0\.0\.1)(:\d+)?$/i.test(host)) return jsonError(403, "Local FMS dashboard only");
  const origin = request.headers.get("origin") || "";
  if (!origin || ![ `http://${host}`, `https://${host}` ].includes(origin)) return jsonError(403, "Invalid request origin");
  const site = request.headers.get("sec-fetch-site");
  if (site && site !== "same-origin") return jsonError(403, "Cross-site physical control denied");
  if (request.headers.get("x-fms-ui-intent") !== "physical-5cm") return jsonError(403, "UI request header required");
  let action: string;
  try {
    const body: unknown = await request.json();
    action = typeof body === "object" && body !== null && "action" in body ? String(body.action) : "";
  } catch { return jsonError(400, "Invalid request"); }
  if (action !== "start" && action !== "cancel") return jsonError(400, "Invalid action");

  let token = "";
  try {
    // The Next dev server runs with cwd=FMS/frontend. Read FMS/.env on server only.
    const env = await readFile(resolve(process.cwd(), "..", ".env"), "utf8");
    const match = env.match(/^FMS_PHYSICAL_5CM_TEST_TOKEN=([A-Za-z0-9_-]+)\s*$/m);
    token = match?.[1] ?? "";
  } catch { /* Respond with a generic configuration error; never echo secrets. */ }
  if (!token) return jsonError(503, "FMS physical test token unavailable on frontend server");

  const suffix = action === "cancel" ? "/cancel" : "";
  try {
    const res = await fetch(`http://127.0.0.1:8741/api/physical/tests/pinky-1/forward-5cm${suffix}`, {
      method: "POST",
      headers: { "X-FMS-Operator-Token": token },
      cache: "no-store",
      signal: AbortSignal.timeout(18000),
    });
    const reply: unknown = await res.json().catch(() => ({ detail: `Backend HTTP ${res.status}` }));
    return NextResponse.json(reply, { status: res.status, headers: { "Cache-Control": "no-store" } });
  } catch {
    return jsonError(502, "Local FMS backend not reachable");
  }
}
