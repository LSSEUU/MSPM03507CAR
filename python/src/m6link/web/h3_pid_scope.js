"use strict";

const ui = Object.fromEntries([
  "connectionChip", "phaseChip", "visionChip", "faultChip", "liveX", "liveV",
  "notice", "revision", "kpInput", "kdInput", "applyKp", "applyKd", "runQuota",
  "startButton", "stopButton", "estopButton", "duration", "captureDir", "markerInput",
  "markerButton", "pauseButton", "clearButton", "cursorReadout", "positionStat",
  "velocityStat", "tiltStat", "actuatorMeta", "actuatorStat", "assessment",
  "metricPeak", "metricOvershoot", "metricSettle", "metricCrossings", "metricTilt",
  "metricGap", "tuningHint"
].map(id => [id, document.getElementById(id)]));

const COLORS = {
  text: "#b9c8da", muted: "#7588a3", grid: "rgba(137,158,188,.14)",
  cyan: "#45d3e8", violet: "#ae87ff", amber: "#ffc35c", red: "#ff6678",
  green: "#64dc9b", target: "#d2dcec"
};

const app = {
  cursor: 0,
  samples: [],
  markers: [],
  windowSeconds: 15,
  paused: false,
  pauseEndMs: null,
  hoverTime: null,
  busy: false,
  state: null,
  charts: []
};

function fmt(value, digits = 1) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "—";
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    cache: "no-store",
    headers: {"Content-Type": "application/json"},
    ...options
  });
  const body = await response.json();
  if (!response.ok || body.ok === false) throw new Error(body.error || `HTTP ${response.status}`);
  return body;
}

function chip(element, text, kind = "muted", dot = false) {
  element.className = `chip ${kind}`;
  element.innerHTML = `${dot ? "<i></i>" : ""}${text}`;
}

function showMessage(message, kind = "") {
  ui.notice.textContent = message || "—";
  ui.notice.className = `notice ${kind}`;
}

function currentSample() {
  return app.state?.latest && Object.keys(app.state.latest).length ? app.state.latest : null;
}

function updateState(state) {
  app.state = state;
  const latest = currentSample();
  chip(ui.connectionChip, state.connected ? "SERIAL ONLINE" : "SERIAL OFFLINE",
       state.connected ? "good" : "bad", true);
  const ready = Boolean(state.ready);
  const centered = Boolean(state.centered);
  const phaseText = state.phase === "idle" && !centered ? "IDLE · CENTER BALL" :
    String(state.phase || "—").toUpperCase();
  const phaseKind = state.phase === "running" ? "active" :
    (["fault", "error", "emergency"].includes(state.phase) ? "bad" :
      (state.phase === "returning" || (state.phase === "idle" && !centered) ? "warn" : "muted"));
  chip(ui.phaseChip, phaseText, phaseKind);
  const visionGood = latest && (Number(latest.flags) & 1) && Number(latest.vision_age_ms) <= 150;
  chip(ui.visionChip, latest ? `VISION ${latest.vision_age_ms} ms` : "VISION —",
       visionGood ? "good" : "warn");
  const fault = Number(state.h3_fault || latest?.fault || 0);
  chip(ui.faultChip, `FAULT ${fault ? `0x${fault.toString(16).toUpperCase()}` : "0"}`,
       fault ? "bad" : "good");
  ui.liveX.textContent = latest ? fmt(latest.x_mm_x10 / 10, 1) : "—";
  ui.liveV.textContent = latest ? fmt(latest.vx_mm_s, 0) : "—";
  ui.revision.textContent = `rev ${state.active_revision || "—"}`;
  if (document.activeElement !== ui.kpInput && state.profile?.kp_mdeg_per_mm !== undefined)
    ui.kpInput.value = state.profile.kp_mdeg_per_mm;
  if (document.activeElement !== ui.kdInput && state.profile?.kd_mdeg_per_mm_s !== undefined)
    ui.kdInput.value = state.profile.kd_mdeg_per_mm_s;
  ui.runQuota.textContent = `${state.remaining_runs || 0} / 8 剩余`;
  ui.duration.textContent = `${fmt(state.trial_duration_s, 1)} s`;
  ui.captureDir.textContent = state.capture_dir || "—";
  ui.captureDir.title = state.capture_dir || "";

  const running = ["running", "returning"].includes(state.phase);
  const canControl = state.connected && !["fault", "error", "emergency"].includes(state.phase);
  ui.startButton.disabled = app.busy || !canControl || running || !ready || !centered;
  ui.stopButton.disabled = app.busy || !state.connected || !running;
  ui.applyKp.disabled = app.busy || !canControl || running;
  ui.applyKd.disabled = app.busy || !canControl || running;
  ui.markerButton.disabled = app.busy || !state.connected;

  const centerWait = state.connected && state.phase === "idle" && !centered ?
    `CENTER BALL：视觉位置 ${latest ? fmt(latest.x_mm_x10 / 10, 1) : "—"} mm；请放到 0±3 mm 并稳定 ${fmt(Math.max(0, 1000 - Number(state.center_stable_ms || 0)) / 1000, 1)} 秒` : "";
  showMessage(state.error || centerWait || state.message || state.wait_reason,
    ["fault", "error", "emergency"].includes(state.phase) ? "error" :
      (state.phase === "idle" && state.message ? "success" : ""));
  updateMetrics(state.metrics || {});
  updateChartStats(latest);
}

function updateMetrics(metrics) {
  ui.metricPeak.textContent = metrics.detected ? fmt(metrics.peak_mm, 1) : "—";
  ui.metricOvershoot.textContent = metrics.detected ? fmt(metrics.opposite_overshoot_mm, 1) : "—";
  ui.metricSettle.textContent = metrics.settle_ms ?? "—";
  ui.metricCrossings.textContent = metrics.detected ? metrics.zero_crossings : "—";
  ui.metricTilt.textContent = metrics.samples ? fmt(metrics.max_tilt_deg, 2) : "—";
  ui.metricGap.textContent = metrics.samples ? metrics.gap_count : "—";
  if (!metrics.detected) {
    ui.assessment.textContent = "等待轻拨";
    ui.tuningHint.textContent = "轻拨约 ±10 mm 后立即松手；位置曲线用于看回零，速度与倾角用于区分 Kp/Kd 问题。";
  } else if (!metrics.settled) {
    ui.assessment.textContent = "响应尚未稳定";
    ui.tuningHint.textContent = "若位置反复穿越 0 且幅度衰减慢，优先小幅增加 Kd；若始终回得慢且不过冲，再小幅增加 Kp。";
  } else if (metrics.opposite_overshoot_mm > 3 || metrics.zero_crossings >= 3) {
    ui.assessment.textContent = "过冲 / 振荡偏大";
    ui.tuningHint.textContent = "先保持 Kp 不变，将 Kd 小步增加（例如 +4）；左右各复测一次，确认过冲下降且执行器没有明显滞后。";
  } else if (metrics.settle_ms > 2500 && metrics.opposite_overshoot_mm < 1.5) {
    ui.assessment.textContent = "稳定但回零偏慢";
    ui.tuningHint.textContent = "可保持 Kd 不变，将 Kp 小步增加（例如 +5）；若出现多次过零，再退回 Kp 或增加 Kd。";
  } else {
    ui.assessment.textContent = "基线响应可用";
    ui.tuningHint.textContent = "记录这一轮方向与手感，再做相反方向拨动。两侧都稳定后，才比较单项参数改动。";
  }
  if (metrics.gap_count) ui.tuningHint.textContent += ` 本轮有 ${metrics.gap_count} 个遥测 gap，判读需谨慎。`;
}

function updateChartStats(latest) {
  if (!latest) return;
  ui.positionStat.textContent = `error ${fmt(latest.error_mm_x10 / 10, 1)} mm`;
  ui.velocityStat.textContent = `|v| ${fmt(Math.abs(latest.vx_mm_s), 0)} mm/s`;
  ui.tiltStat.textContent = `PD ${fmt(latest.raw_pd_mdeg / 1000, 2)}° · actual ${fmt(latest.tilt_mdeg / 1000, 2)}°`;
  ui.actuatorMeta.textContent = `方向 ${latest.actuator_direction} · RPM ${latest.actuator_rpm}`;
  ui.actuatorStat.textContent = `lag ${Math.abs(latest.actuator_target_counts - latest.actuator_counts)} count`;
}

function niceStep(span, target = 5) {
  const raw = Math.max(span / target, 1e-9);
  const power = 10 ** Math.floor(Math.log10(raw));
  const scaled = raw / power;
  const nice = scaled <= 1 ? 1 : (scaled <= 2 ? 2 : (scaled <= 5 ? 5 : 10));
  return nice * power;
}

class ScopeChart {
  constructor(canvas, config) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.config = config;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    canvas.addEventListener("mousemove", event => this.pointer(event));
    canvas.addEventListener("mouseleave", () => { app.hoverTime = null; drawAll(); });
    this.resize();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const ratio = Math.max(1, window.devicePixelRatio || 1);
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.width = rect.width;
    this.height = rect.height;
    this.draw();
  }

  pointer(event) {
    const rect = this.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const range = visibleRange();
    const left = 54, right = 14;
    const fraction = Math.max(0, Math.min(1, (x - left) / Math.max(1, rect.width - left - right)));
    app.hoverTime = range.start + fraction * (range.end - range.start);
    drawAll();
  }

  draw() {
    const ctx = this.ctx, width = this.width || 0, height = this.height || 0;
    if (!width || !height) return;
    ctx.clearRect(0, 0, width, height);
    const plot = {left: 54, right: width - 14, top: 13, bottom: height - 27};
    const range = visibleRange();
    const rows = app.samples.filter(row => row.time_ms >= range.start && row.time_ms <= range.end);
    const values = [];
    for (const row of rows) for (const line of this.config.lines) {
      const value = line.value(row); if (Number.isFinite(value)) values.push(value);
    }
    let [minY, maxY] = this.config.range(values);
    if (minY === maxY) { minY -= 1; maxY += 1; }
    const y = value => plot.bottom - (value - minY) / (maxY - minY) * (plot.bottom - plot.top);
    const x = value => plot.left + (value - range.start) / Math.max(1, range.end - range.start) * (plot.right - plot.left);

    if (this.config.band) {
      const [lo, hi] = this.config.band;
      ctx.fillStyle = "rgba(100,220,155,.08)";
      ctx.fillRect(plot.left, y(hi), plot.right - plot.left, y(lo) - y(hi));
    }
    ctx.font = "10px ui-monospace, Consolas, monospace";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    const yStep = niceStep(maxY - minY);
    for (let tick = Math.ceil(minY / yStep) * yStep; tick <= maxY + yStep * .01; tick += yStep) {
      const py = y(tick);
      ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(plot.left, py); ctx.lineTo(plot.right, py); ctx.stroke();
      ctx.fillStyle = COLORS.muted; ctx.fillText(Number(tick.toFixed(3)).toString(), plot.left - 7, py);
    }
    ctx.textAlign = "center"; ctx.textBaseline = "top";
    const tickMs = Math.max(1000, Math.ceil(app.windowSeconds / 6) * 1000);
    for (let tick = Math.ceil(range.start / tickMs) * tickMs; tick <= range.end; tick += tickMs) {
      const px = x(tick);
      ctx.strokeStyle = COLORS.grid; ctx.beginPath(); ctx.moveTo(px, plot.top); ctx.lineTo(px, plot.bottom); ctx.stroke();
      ctx.fillStyle = COLORS.muted; ctx.fillText(`${((tick - range.end) / 1000).toFixed(0)}s`, px, plot.bottom + 7);
    }
    for (const line of this.config.lines) {
      ctx.strokeStyle = line.color; ctx.lineWidth = line.width || 1.7;
      ctx.setLineDash(line.dash || []); ctx.beginPath();
      let started = false;
      for (const row of rows) {
        const value = line.value(row); if (!Number.isFinite(value)) continue;
        const px = x(row.time_ms), py = y(value);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      if (started) ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const marker of app.markers) if (marker.time >= range.start && marker.time <= range.end) {
      const px = x(marker.time); ctx.strokeStyle = COLORS.amber; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, plot.top); ctx.lineTo(px, plot.bottom); ctx.stroke();
    }
    if (app.hoverTime !== null && app.hoverTime >= range.start && app.hoverTime <= range.end) {
      const px = x(app.hoverTime); ctx.strokeStyle = "rgba(231,238,249,.55)"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, plot.top); ctx.lineTo(px, plot.bottom); ctx.stroke();
      const nearest = nearestSample(app.hoverTime);
      if (nearest) {
        for (const line of this.config.lines) {
          const value = line.value(nearest);
          ctx.fillStyle = line.color; ctx.beginPath(); ctx.arc(px, y(value), 3, 0, Math.PI * 2); ctx.fill();
        }
      }
    }
  }
}

function paddedSymmetric(values, minimum) {
  const extent = Math.max(minimum, ...values.map(Math.abs), 0) * 1.12;
  return [-extent, extent];
}

function paddedRange(values, minimumSpan = 100) {
  if (!values.length) return [-minimumSpan / 2, minimumSpan / 2];
  let min = Math.min(...values), max = Math.max(...values);
  const pad = Math.max((max - min) * .12, minimumSpan * .1);
  if (max - min < minimumSpan) { const center = (min + max) / 2; min = center - minimumSpan / 2; max = center + minimumSpan / 2; }
  return [min - pad, max + pad];
}

function visibleRange() {
  const latest = app.samples.length ? app.samples[app.samples.length - 1].time_ms : 0;
  const end = app.paused && app.pauseEndMs !== null ? app.pauseEndMs : latest;
  return {start: end - app.windowSeconds * 1000, end};
}

function nearestSample(timeMs) {
  let best = null, distance = Infinity;
  for (const row of app.samples) {
    const candidate = Math.abs(row.time_ms - timeMs);
    if (candidate < distance) { distance = candidate; best = row; }
  }
  return best;
}

function updateCursorText() {
  if (app.hoverTime === null) { ui.cursorReadout.textContent = "移动鼠标读取同一时刻"; return; }
  const row = nearestSample(app.hoverTime);
  if (!row) return;
  ui.cursorReadout.textContent = `t ${row.time_ms} ms · x ${fmt(row.x_mm_x10 / 10, 1)} mm · v ${row.vx_mm_s} · tilt ${fmt(row.tilt_mdeg / 1000, 2)}°`;
}

function drawAll() { for (const chart of app.charts) chart.draw(); updateCursorText(); }

function initCharts() {
  app.charts = [
    new ScopeChart(document.getElementById("positionChart"), {
      lines: [
        {color: COLORS.cyan, value: row => row.x_mm_x10 / 10, width: 2},
        {color: COLORS.target, value: row => row.target_mm_x10 / 10, dash: [6, 5]}
      ], band: [-3, 3], range: values => paddedSymmetric(values.concat([-3, 3]), 15)
    }),
    new ScopeChart(document.getElementById("velocityChart"), {
      lines: [{color: COLORS.amber, value: row => row.vx_mm_s, width: 2}],
      band: [-5, 5], range: values => paddedSymmetric(values.concat([-5, 5]), 25)
    }),
    new ScopeChart(document.getElementById("tiltChart"), {
      lines: [
        {color: COLORS.violet, value: row => row.raw_pd_mdeg / 1000, width: 1.5},
        {color: COLORS.cyan, value: row => row.tilt_mdeg / 1000, width: 2},
        {color: COLORS.red, value: row => row.forced_tilt_mdeg / 1000, dash: [5, 4]}
      ], range: values => paddedSymmetric(values, 5)
    }),
    new ScopeChart(document.getElementById("actuatorChart"), {
      lines: [
        {color: COLORS.cyan, value: row => row.actuator_counts, width: 2},
        {color: COLORS.target, value: row => row.actuator_target_counts, dash: [6, 4]}
      ], range: values => paddedRange(values, 400)
    })
  ];
}

async function pollState() {
  try { const body = await api("/api/state"); updateState(body.state); }
  catch (error) { showMessage(`状态读取失败：${error.message}`, "error"); }
}

async function pollSamples() {
  try {
    const body = await api(`/api/samples?after=${app.cursor}`);
    if (body.truncated) app.samples = [];
    if (body.samples.length) {
      app.samples.push(...body.samples);
      if (app.samples.length > 12000) app.samples.splice(0, app.samples.length - 12000);
      app.cursor = body.cursor;
      drawAll();
    }
  } catch (error) { showMessage(`波形读取失败：${error.message}`, "error"); }
}

async function command(path, body = {}) {
  if (app.busy) return;
  app.busy = true;
  if (app.state) updateState(app.state);
  try {
    const result = await api(path, {method: "POST", body: JSON.stringify(body)});
    if (result.state) updateState(result.state);
    await pollState();
  } catch (error) {
    showMessage(error.message, "error");
  } finally {
    app.busy = false;
    if (app.state) updateState(app.state);
  }
}

ui.startButton.addEventListener("click", () => command("/api/start"));
ui.stopButton.addEventListener("click", () => command("/api/stop"));
ui.estopButton.addEventListener("click", () => {
  if (window.confirm("确认发送原始急停 S？这可能锁存 fault，界面不会自动清除。")) command("/api/estop");
});
ui.applyKp.addEventListener("click", () => command("/api/parameter", {
  parameter: "kp_mdeg_per_mm", value: Number(ui.kpInput.value)
}));
ui.applyKd.addEventListener("click", () => command("/api/parameter", {
  parameter: "kd_mdeg_per_mm_s", value: Number(ui.kdInput.value)
}));
ui.markerButton.addEventListener("click", async () => {
  const label = ui.markerInput.value.trim() || "手动标记";
  const latest = app.samples[app.samples.length - 1];
  await command("/api/marker", {label});
  if (latest) app.markers.push({time: latest.time_ms, label});
  ui.markerInput.value = ""; drawAll();
});
ui.markerInput.addEventListener("keydown", event => { if (event.key === "Enter") ui.markerButton.click(); });
ui.pauseButton.addEventListener("click", () => {
  app.paused = !app.paused;
  app.pauseEndMs = app.paused && app.samples.length ? app.samples[app.samples.length - 1].time_ms : null;
  ui.pauseButton.textContent = app.paused ? "继续画面" : "暂停画面";
  ui.pauseButton.classList.toggle("active", app.paused); drawAll();
});
ui.clearButton.addEventListener("click", () => {
  app.samples = []; app.markers = []; app.hoverTime = null; drawAll();
});
document.querySelectorAll(".window").forEach(button => button.addEventListener("click", () => {
  app.windowSeconds = Number(button.dataset.window);
  document.querySelectorAll(".window").forEach(item => item.classList.toggle("active", item === button));
  drawAll();
}));

initCharts();
pollState(); pollSamples();
setInterval(pollState, 500);
setInterval(pollSamples, 100);
