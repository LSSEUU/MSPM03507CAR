"use strict";

const COLORS = {
  cyan: "#22d3ee", orange: "#fb923c", blue: "#60a5fa", amber: "#fbbf24",
  green: "#34d399", teal: "#2dd4bf", yellow: "#fde047", violet: "#a78bfa", pink: "#f472b6"
};
const history = { core: [], wheel: [], line: [], health: [] };
const latest = {};
let cursor = 0;
let windowSeconds = 10;
let paused = false;
let dirty = true;
let fetching = false;
let lastServerOk = 0;
let counts = {};
let gaps = {};
let controlToken = "";
let controlState = null;
let actionsInitialized = false;
let currentArmToken = null;
let armedDeadline = 0;
let parameterData = [];
let toastTimer = 0;
let controlFetching = false;
let parametersLoading = false;
let calibrationState = { available: false, active: false };
let calibrationFetching = false;
let recordingState = { recording: false };
let recordingFetching = false;
const LAYOUT_STORAGE_KEY = "m6.dashboard.layout.v1";
const AUTO_RECORDING_STORAGE_KEY = "m6.dashboard.auto-recording.v1";
const MANUAL_SETTINGS_STORAGE_KEY = "m6.dashboard.manual-drive.v1";
const MICRO_TREND_MS = 5000;
const manualDrive = {
  enabled: false, controllerId: null, sequence: 0, keys: new Set(),
  sending: false, pending: false, lastTarget: [0, 0], lastBootId: null,
};
const CAMERA_BUFFER_LIMIT = 32 * 1024;
const cameraView = {
  bytes: [], baseOffset: 0, autoScrollPaused: false,
  status: { available: false, enabled: false },
};
const playback = {
  active: false, sessionId: null, manifest: null, records: [], position: 0,
  after: 0, hasMore: false, loading: false, currentMs: 0, durationMs: 0,
  speed: 1, playing: false, lastFrame: 0, generation: 0,
};
const stepperView = {
  available: false, snapshot: null, fetching: false, busy: false,
  samples: [], lastSequence: -1,
};

const ACTION_LABELS = {
  WHEEL_SPEED_TEST: "轮速测试",
  HEADING_HOLD: "航向保持直行",
  HEADING_TURN: "定角转向",
  HEADING_RATE: "恒角速度转向",
  LINE_FOLLOW: "循迹运行",
  MOTOR_SELF_TEST: "电机自检",
};

const $ = (id) => document.getElementById(id);
const numeric = (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const fixed = (value, digits = 0) => value == null ? "—" : numeric(value).toFixed(digits);
const hex32 = (value) => value == null ? "—" : `0x${(numeric(value) >>> 0).toString(16).padStart(8, "0").toUpperCase()}`;
const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

function formatDuration(milliseconds, precise = false) {
  const value = Math.max(0, numeric(milliseconds));
  const minutes = Math.floor(value / 60000);
  const seconds = Math.floor(value / 1000) % 60;
  const fraction = precise ? `.${Math.floor(value % 1000).toString().padStart(3, "0")}` : "";
  return `${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}${fraction}`;
}

function formatBytes(bytes) {
  let value = Math.max(0, numeric(bytes));
  const units = ["B", "KB", "MB", "GB"];
  let index = 0;
  while (value >= 1024 && index < units.length - 1) { value /= 1024; index += 1; }
  return `${value.toFixed(index === 0 ? 0 : 1)} ${units[index]}`;
}

function autoRecordingEnabled() {
  try { return window.localStorage.getItem(AUTO_RECORDING_STORAGE_KEY) === "true"; }
  catch (_error) { return false; }
}

function storedLayoutMode() {
  try { return window.localStorage.getItem(LAYOUT_STORAGE_KEY) === "cockpit" ? "cockpit" : "scroll"; }
  catch (_error) { return "scroll"; }
}

function setLayoutMode(mode, persist = true) {
  const selected = mode === "cockpit" ? "cockpit" : "scroll";
  document.body.dataset.layout = selected;
  $("layoutScrollButton").setAttribute("aria-pressed", String(selected === "scroll"));
  $("layoutCockpitButton").setAttribute("aria-pressed", String(selected === "cockpit"));
  if (persist) {
    try { window.localStorage.setItem(LAYOUT_STORAGE_KEY, selected); } catch (_error) { /* storage may be disabled */ }
  }
  dirty = true;
}

function loadManualSettings() {
  let settings = {max_speed_mm_s:160, steering_percent:65};
  try { settings = {...settings, ...JSON.parse(window.localStorage.getItem(MANUAL_SETTINGS_STORAGE_KEY) || "{}")} } catch (_error) { /* use defaults */ }
  $("manualMaxSpeed").value = String(clamp(numeric(settings.max_speed_mm_s, 160), 50, 400));
  $("manualSteering").value = String(clamp(numeric(settings.steering_percent, 65), 10, 100));
  updateManualSettingLabels();
}

function saveManualSettings() {
  try {
    window.localStorage.setItem(MANUAL_SETTINGS_STORAGE_KEY, JSON.stringify({
      max_speed_mm_s: numeric($("manualMaxSpeed").value, 160),
      steering_percent: numeric($("manualSteering").value, 65),
    }));
  } catch (_error) { /* storage may be disabled */ }
}

function updateManualSettingLabels() {
  $("manualMaxSpeedValue").textContent = `${$("manualMaxSpeed").value} mm/s`;
  $("manualSteeringValue").textContent = `${$("manualSteering").value}%`;
}

function manualComponents() {
  return {
    throttle: (manualDrive.keys.has("w") ? 1 : 0) - (manualDrive.keys.has("s") ? 1 : 0),
    steering: (manualDrive.keys.has("a") ? 1 : 0) - (manualDrive.keys.has("d") ? 1 : 0),
  };
}

function renderManualKeys() {
  document.querySelectorAll("[data-manual-key]").forEach((key) => {
    const pressed = manualDrive.keys.has(key.dataset.manualKey);
    key.classList.toggle("pressed", pressed);
    key.setAttribute("aria-pressed", String(pressed));
  });
}

function setManualMode(mode) {
  const manual = mode === "manual";
  $("actionModePanel").hidden = manual;
  $("manualDrivePanel").hidden = !manual;
  $("actionModeButton").classList.toggle("active", !manual);
  $("manualModeButton").classList.toggle("active", manual);
  $("actionModeButton").setAttribute("aria-selected", String(!manual));
  $("manualModeButton").setAttribute("aria-selected", String(manual));
  if (!manual && manualDrive.enabled) disableManualDrive("切换到动作测试");
}

function manualInputAllowed(event) {
  if (!manualDrive.enabled || playback.active || event.ctrlKey || event.altKey || event.metaKey) return false;
  if (document.querySelector("dialog[open]")) return false;
  const target = event.target;
  if (target?.closest?.("input, textarea, select, [contenteditable='true'], dialog[open]")) return false;
  return ["w", "a", "s", "d"].includes(String(event.key).toLowerCase());
}

function setManualLocalDisabled(reason = "未启用") {
  manualDrive.enabled = false;
  manualDrive.controllerId = null;
  manualDrive.keys.clear();
  manualDrive.pending = false;
  manualDrive.lastTarget = [0, 0];
  renderManualKeys();
  $("manualEnableButton").textContent = "启用键盘遥控";
  $("manualEnableButton").classList.remove("enabled");
  $("manualConnection").textContent = reason;
  $("manualTargets").textContent = "0 / 0 mm/s";
  updateArmButton();
}

async function sendManualCommand() {
  if (!manualDrive.enabled || !manualDrive.controllerId || playback.active) return;
  if (manualDrive.sending) { manualDrive.pending = true; return; }
  manualDrive.sending = true;
  const input = manualComponents();
  const body = {
    controller_id: manualDrive.controllerId,
    sequence: ++manualDrive.sequence,
    throttle: input.throttle,
    steering: input.steering,
    max_speed_mm_s: numeric($("manualMaxSpeed").value, 160),
    steering_percent: numeric($("manualSteering").value, 65),
  };
  try {
    const result = await command("/api/control/manual/command", body);
    manualDrive.lastTarget = result.target_mm_s || [0, 0];
    $("manualTargets").textContent = `${manualDrive.lastTarget.join(" / ")} mm/s`;
    $("manualConnection").textContent = input.throttle || input.steering ? "指令持续发送中" : "已启用 · 车辆停止";
  } catch (error) {
    setManualLocalDisabled("控制链路失效");
    showToast(`键盘遥控已退出：${error.message}`, true);
  } finally {
    manualDrive.sending = false;
    if (manualDrive.pending && manualDrive.enabled) {
      manualDrive.pending = false;
      sendManualCommand();
    }
  }
}

async function disableManualDrive(reason = "已关闭", notifyServer = true) {
  const controllerId = manualDrive.controllerId;
  if (!manualDrive.enabled && !controllerId) return;
  setManualLocalDisabled(reason);
  if (!notifyServer || !controllerId || !controlToken || playback.active) return;
  try { await command("/api/control/manual/disable", {controller_id:controllerId}); }
  catch (error) { showToast(`停车/释放请求失败：${error.message}`, true); }
}

function updateManualState(state) {
  const manual = state.manual_drive || {};
  const available = Boolean(manual.available);
  const faulted = Boolean(numeric(state.status?.fault_bits)) || Boolean(state.status_error);
  $("manualCapabilityBadge").className = `badge ${available ? "ok" : "bad"}`;
  $("manualCapabilityBadge").textContent = available ? "手动驾驶能力已协商" : "未协商手动驾驶能力";
  $("manualEnableButton").disabled = playback.active || !available || faulted || !controlToken;
  if (manualDrive.enabled) {
    $("manualEnableButton").disabled = playback.active;
    $("manualEnableButton").textContent = "停止并关闭遥控";
    $("manualEnableButton").classList.add("enabled");
  }
  const device = manual.device || {};
  const source = device.source?.name || device.source_name || (manualDrive.enabled ? "HOST_RPC" : "NONE");
  $("manualSource").textContent = source;
  $("manualAck").textContent = manual.ack_age_ms == null ? "—" : `${manual.ack_age_ms} ms 前`;
  if (manual.error) $("manualConnection").textContent = `异常：${manual.error}`;
  if (!manualDrive.enabled && manual.enabled) $("manualConnection").textContent = "其他网页控制器已启用";
}

function openParameterDialog() {
  const dialog = $("parameterDialog");
  if (!dialog.open) dialog.showModal();
  if (!parameterData.length) loadParameters(false);
}

function initializeSensors() {
  const root = $("sensorBars");
  for (let i = 0; i < 8; i += 1) {
    const sensor = document.createElement("div");
    sensor.className = "sensor";
    sensor.id = `sensor${i}`;
    sensor.innerHTML = `<span class="sensor-value">—</span><div class="sensor-track"><div class="sensor-fill"></div></div><span class="sensor-channel">CH${i}</span>`;
    root.appendChild(sensor);
  }
}

function appendEvent(event) {
  if (event.stream === "vision_uart_rx") {
    appendCameraEvent(event);
    return;
  }
  if (!history[event.stream]) return;
  history[event.stream].push(event);
  latest[event.stream] = event;
}

function clearHistory(resetCursor = false) {
  Object.keys(history).forEach((stream) => { history[stream].length = 0; });
  Object.keys(latest).forEach((stream) => { delete latest[stream]; });
  clearCameraDisplay();
  if (resetCursor) cursor = 0;
  dirty = true;
}

function cameraBytesFromHex(value) {
  const hex = typeof value === "string" ? value.trim() : "";
  if (!hex || hex.length % 2 || !/^[0-9a-fA-F]+$/.test(hex)) return [];
  const bytes = new Array(hex.length / 2);
  for (let index = 0; index < bytes.length; index += 1)
    bytes[index] = Number.parseInt(hex.slice(index * 2, index * 2 + 2), 16);
  return bytes;
}

function safeCameraText(bytes) {
  const decoded = new TextDecoder("utf-8", { fatal: false }).decode(Uint8Array.from(bytes));
  let output = "";
  for (const character of decoded) {
    const code = character.codePointAt(0);
    if (character === "\n" || character === "\r" || character === "\t" || code >= 0xa0) output += character;
    else if (code >= 0x20 && code < 0x7f) output += character;
    else if (code <= 0xff) output += `\\x${code.toString(16).padStart(2, "0").toUpperCase()}`;
    else output += `\\u${code.toString(16).padStart(4, "0").toUpperCase()}`;
  }
  return output;
}

function cameraHexDump(bytes, baseOffset) {
  const lines = [];
  for (let start = 0; start < bytes.length; start += 16) {
    const row = bytes.slice(start, start + 16);
    const offset = (baseOffset + start).toString(16).padStart(8, "0").toUpperCase();
    const hex = row.map((value) => value.toString(16).padStart(2, "0").toUpperCase()).join(" ").padEnd(47, " ");
    const ascii = row.map((value) => value >= 0x20 && value <= 0x7e ? String.fromCharCode(value) : ".").join("");
    lines.push(`${offset}  ${hex}  |${ascii}|`);
  }
  return lines.join("\n");
}

function renderCameraData() {
  const textPanel = $("cameraText");
  const hexPanel = $("cameraHex");
  const textScroll = textPanel.scrollTop;
  const hexScroll = hexPanel.scrollTop;
  const hasData = cameraView.bytes.length > 0;
  textPanel.textContent = hasData ? safeCameraText(cameraView.bytes) : "等待 UART1 RX 数据…";
  hexPanel.textContent = hasData ? cameraHexDump(cameraView.bytes, cameraView.baseOffset) : "等待 UART1 RX 数据…";
  if (cameraView.autoScrollPaused) {
    textPanel.scrollTop = textScroll;
    hexPanel.scrollTop = hexScroll;
  } else {
    textPanel.scrollTop = textPanel.scrollHeight;
    hexPanel.scrollTop = hexPanel.scrollHeight;
  }
}

function updateCameraStatus(status = cameraView.status) {
  cameraView.status = { ...cameraView.status, ...(status || {}) };
  const current = cameraView.status;
  const losses = numeric(current.rx_overflow) + numeric(current.tx_overflow) +
    numeric(current.bridge_dropped) + numeric(current.hardware_errors);
  const badge = $("cameraBadge");
  badge.className = `badge ${!current.available ? "waiting" : (losses ? "bad" : (current.enabled ? "ok" : "waiting"))}`;
  badge.textContent = playback.active ? "历史回放 · 只读" :
    (!current.available ? "固件不支持" : (current.enabled ? "监视已启用" : "监视未启用"));
  $("cameraMessage").textContent = current.message || (current.enabled ? "尚未收到数据" : "UART1 监视未启用");
  $("cameraRxTx").textContent = `${numeric(current.rx_bytes).toLocaleString()} / ${numeric(current.tx_bytes).toLocaleString()} B`;
  $("cameraOverflow").textContent = `${numeric(current.rx_overflow).toLocaleString()} / ${numeric(current.tx_overflow).toLocaleString()}`;
  $("cameraBridgeDrop").textContent = numeric(current.bridge_dropped).toLocaleString();
  $("cameraHardwareErrors").textContent = numeric(current.hardware_errors).toLocaleString();
  const nowMs = playback.active ? playback.currentMs : numeric(current.dashboard_now_ms, 0);
  const lastMs = current.last_received_ms;
  $("cameraAge").textContent = lastMs == null ? "—" : `${Math.max(0, nowMs - numeric(lastMs))} ms`;
}

function appendCameraEvent(event) {
  const values = event.values || {};
  const incoming = cameraBytesFromHex(values.data_hex);
  if (incoming.length) {
    cameraView.bytes.push(...incoming);
    if (cameraView.bytes.length > CAMERA_BUFFER_LIMIT) {
      const removed = cameraView.bytes.length - CAMERA_BUFFER_LIMIT;
      cameraView.bytes.splice(0, removed);
      cameraView.baseOffset += removed;
    }
  }
  updateCameraStatus({
    available: true, enabled: true, message: event.gap ? "最近收到数据，期间存在遗漏" : "最近收到 UART1 数据",
    last_received_ms: event.received_ms, dashboard_now_ms: event.received_ms,
    rx_bytes: values.rx_bytes, tx_bytes: values.tx_bytes,
    rx_overflow: values.rx_overflow, tx_overflow: values.tx_overflow,
    bridge_dropped: values.bridge_dropped, hardware_errors: values.hardware_errors,
  });
  renderCameraData();
}

function clearCameraDisplay() {
  cameraView.bytes.length = 0;
  cameraView.baseOffset = 0;
  renderCameraData();
}

function trimHistory(nowMs) {
  const keepMs = Math.max(30, windowSeconds) * 1000 + 2000;
  Object.values(history).forEach((events) => {
    while (events.length && nowMs - events[0].received_ms > keepMs) events.shift();
  });
}

async function poll() {
  if (playback.active) return;
  if (fetching) return;
  fetching = true;
  try {
    let more = true;
    let rounds = 0;
    while (more && rounds < 4) {
      const response = await fetch(`/api/state?after=${cursor}&limit=2000`, { cache: "no-store" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const state = await response.json();
      if (playback.active) return;
      if (state.reset) clearHistory(false);
      state.events.forEach(appendEvent);
      cursor = state.cursor;
      more = state.has_more;
      counts = state.counts || {};
      gaps = state.gaps || {};
      lastServerOk = performance.now();
      updateConnection(state);
      cameraView.status = { available: false, enabled: false };
      updateCameraStatus({ ...(state.camera || {}), dashboard_now_ms: numeric(state.uptime_ms) });
      rounds += 1;
    }
    if (!paused) dirty = true;
  } catch (_error) {
    updateConnection(null);
  } finally {
    fetching = false;
  }
}

function updateConnection(state) {
  const connected = Boolean(state && state.connected);
  const reconnecting = Boolean(state && state.reconnecting);
  $("connectionDot").className = `status-dot ${connected ? "ok" : (reconnecting ? "waiting" : "bad")}`;
  $("connectionText").textContent = connected
    ? (state.connection_message || "串口数据在线")
    : (state ? (state.connection_message || "串口已断开") : "本地服务不可达");
  if (state) {
    const id = state.identity || {};
    const bootId = id.boot_id ?? null;
    if (manualDrive.lastBootId !== null && bootId !== null && bootId !== manualDrive.lastBootId && manualDrive.enabled)
      disableManualDrive("车辆已重启，请重新启用", false);
    if (bootId !== null) manualDrive.lastBootId = bootId;
    const reconnectText = numeric(state.reconnect_count) ? ` · 已自动重连 ${numeric(state.reconnect_count)} 次` : "";
    $("deviceText").textContent = `${id.port || "未知端口"} · ${id.firmware_id || id.firmware || "未知固件"}${reconnectText}`;
    $("sampleCount").textContent = Object.values(counts).reduce((a, b) => a + numeric(b), 0).toLocaleString();
    $("gapCount").textContent = Object.values(gaps).reduce((a, b) => a + numeric(b), 0).toLocaleString();
  }
  if (!connected && manualDrive.enabled) disableManualDrive("连接中断，固件租约将停车", false);
}

function showToast(message, error = false) {
  const toast = $("toast");
  toast.textContent = message;
  toast.className = `toast${error ? " error" : ""}`;
  toast.hidden = false;
  window.clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => { toast.hidden = true; }, error ? 6000 : 3500);
}

async function command(path, body = {}, button = null) {
  if (playback.active && !path.startsWith("/api/recordings/")) throw new Error("历史回放为只读模式，请先退出回放");
  if (!controlToken) throw new Error("控制服务尚未就绪");
  const previousText = button?.textContent;
  if (button) { button.disabled = true; button.textContent = "处理中…"; }
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-M6-Control-Token": controlToken },
      body: JSON.stringify(body),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error?.message || `HTTP ${response.status}`);
    return payload;
  } finally {
    if (button) { button.disabled = playback.active; button.textContent = previousText; }
  }
}

async function pollControl() {
  if (controlFetching) return;
  controlFetching = true;
  try {
    const response = await fetch("/api/control", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    controlToken = state.control_token;
    controlState = state;
    updateControlState(state);
  } catch (_error) {
    $("controlBadge").className = "badge bad";
    $("controlBadge").textContent = "控制服务不可用";
    $("armButton").disabled = true;
  } finally {
    controlFetching = false;
  }
}

function updateStepperControls() {
  const disabled = playback.active || !stepperView.available || stepperView.busy || !controlToken;
  document.querySelectorAll("#stepperTest button, #stepperTest input, #stepperTest select")
    .forEach((control) => { control.disabled = disabled; });
}

function renderStepper(state) {
  stepperView.available = Boolean(state?.available);
  const snapshot = state?.snapshot || null;
  stepperView.snapshot = snapshot;
  const capability = $("stepperCapabilityBadge");
  if (!stepperView.available) {
    capability.className = "badge bad";
    capability.textContent = "固件不支持";
    $("stepperLinkBadge").className = "badge bad";
    $("stepperLinkBadge").textContent = "不可用";
    $("stepperControlMessage").textContent = state?.error || "当前固件未协商步进扩展，车辆功能不受影响。";
    updateStepperControls();
    return;
  }
  capability.className = "badge ok";
  capability.textContent = "UART3 能力已协商";
  if (!snapshot) {
    $("stepperLinkBadge").className = "badge waiting";
    $("stepperLinkBadge").textContent = "等待首个应答";
    $("stepperControlMessage").textContent = state?.error || "等待步进电机串口应答。";
    updateStepperControls();
    return;
  }
  const link = snapshot.link_state || "offline";
  const linkBadge = $("stepperLinkBadge");
  linkBadge.className = `badge ${link === "online" ? "ok" : link === "stale" ? "waiting" : "bad"}`;
  linkBadge.textContent = link === "online" ? "在线" : link === "stale" ? "状态陈旧" : "离线 / 无应答";
  const speedValid = Boolean(numeric(snapshot.valid_flags) & 1);
  const positionValid = Boolean(numeric(snapshot.valid_flags) & 2);
  $("stepperRpm").textContent = speedValid ? String(numeric(snapshot.speed_rpm)) : "—";
  $("stepperPositionRaw").textContent = positionValid ? String(numeric(snapshot.position_raw)) : "—";
  $("stepperRevolutions").textContent = positionValid ? fixed(snapshot.position_revolutions, 5) : "—";
  $("stepperDegrees").textContent = positionValid ? fixed(snapshot.position_degrees, 2) : "—";
  const stateLabels = {idle:"空闲", waiting_ack:"等待应答", running:"运行中", completed:"已完成", error:"错误"};
  const kindLabels = {none:"无", velocity:"速度测试", position:"位置测试"};
  $("stepperRunState").textContent = `${stateLabels[snapshot.state_name] || snapshot.state_name} / ${kindLabels[snapshot.kind_name] || snapshot.kind_name}`;
  $("stepperRemaining").textContent = [1, 2].includes(numeric(snapshot.state)) ? `${numeric(snapshot.remaining_ms)} ms` : "—";
  $("stepperMotorFlags").textContent = snapshot.motor_flags?.length ? snapshot.motor_flags.join("、") : (numeric(snapshot.valid_flags) & 4 ? "无已置位状态" : "未读取");
  $("stepperHomingFlags").textContent = snapshot.homing_flags?.length ? snapshot.homing_flags.join("、") : (numeric(snapshot.valid_flags) & 8 ? "无已置位状态" : "未读取");
  $("stepperLastResponse").textContent = numeric(snapshot.response_age_ms) === 0xFFFFFFFF
    ? "尚无应答" : `功能 0x${numeric(snapshot.last_function).toString(16).padStart(2, "0").toUpperCase()} / 类型 ${numeric(snapshot.last_response)} / ${numeric(snapshot.response_age_ms)} ms 前`;
  const errors = numeric(snapshot.tx_overflow) + numeric(snapshot.rx_overflow) + numeric(snapshot.hardware_errors) + numeric(snapshot.invalid_frames) + numeric(snapshot.checksum_errors);
  $("stepperErrors").textContent = `${errors}（TX拥塞 ${numeric(snapshot.tx_overflow)} / RX溢出 ${numeric(snapshot.rx_overflow)} / 硬件 ${numeric(snapshot.hardware_errors)} / 格式 ${numeric(snapshot.invalid_frames)} / 校验 ${numeric(snapshot.checksum_errors)}）`;
  $("stepperTxRx").textContent = `${numeric(snapshot.tx_bytes)} / ${numeric(snapshot.rx_bytes)} bytes · 抢占 ${numeric(snapshot.preempted_frames)} 帧`;
  $("stepperStopReason").textContent = snapshot.stop_reason_name || "—";
  $("stepperControlMessage").className = `stepper-control-message${state?.error || snapshot.state_name === "error" ? " error" : ""}`;
  $("stepperControlMessage").textContent = state?.error || (snapshot.state_name === "running"
    ? `${kindLabels[snapshot.kind_name]}正在固件内运行；车辆急停不会停止它。`
    : `命令 ${numeric(snapshot.command_id)} · 测试 ${numeric(snapshot.test_id)} · ${stateLabels[snapshot.state_name] || snapshot.state_name}`);
  if (numeric(snapshot.response_sequence) !== stepperView.lastSequence && (speedValid || positionValid)) {
    stepperView.lastSequence = numeric(snapshot.response_sequence);
    stepperView.samples.push({at: performance.now(), speed: numeric(snapshot.speed_rpm), position: numeric(snapshot.position_raw)});
  }
  const cutoff = performance.now() - 10000;
  stepperView.samples = stepperView.samples.filter((sample) => sample.at >= cutoff);
  $("stepperSpeedTrendText").textContent = speedValid ? `${numeric(snapshot.speed_rpm)} RPM` : "暂无数据";
  $("stepperPositionTrendText").textContent = positionValid ? `${numeric(snapshot.position_raw)} counts` : "暂无数据";
  dirty = true;
  updateStepperControls();
}

async function pollStepper() {
  if (stepperView.fetching || playback.active) return;
  stepperView.fetching = true;
  try {
    const response = await fetch("/api/stepper?refresh=1", {cache:"no-store"});
    const state = await response.json();
    if (!response.ok || !state.ok) throw new Error(state.error?.message || `HTTP ${response.status}`);
    renderStepper(state);
  } catch (error) {
    $("stepperLinkBadge").className = "badge bad";
    $("stepperLinkBadge").textContent = "状态接口不可用";
    $("stepperControlMessage").className = "stepper-control-message error";
    $("stepperControlMessage").textContent = error.message;
  } finally { stepperView.fetching = false; }
}

async function stepperCommand(path, body, button, successMessage) {
  stepperView.busy = true;
  updateStepperControls();
  try {
    const result = await command(path, body, button);
    renderStepper({available:true, snapshot:result.snapshot});
    showToast(successMessage);
    await pollStepper();
  } catch (error) {
    showToast(error.message, true);
    $("stepperControlMessage").className = "stepper-control-message error";
    $("stepperControlMessage").textContent = error.message;
  } finally {
    stepperView.busy = false;
    updateStepperControls();
  }
}

function updateControlState(state) {
  if (!actionsInitialized) initializeActions(state.actions || []);
  const status = state.status || {};
  const faults = numeric(status.fault_bits);
  const actionState = status.action_state_name || status.action_state?.name || "—";
  const actionKind = status.action_kind_name || status.action_kind?.name || "—";
  $("controlActionState").textContent = actionState;
  $("controlActionKind").textContent = ACTION_LABELS[actionKind] || actionKind;
  $("controlTargets").textContent = Array.isArray(status.target_mm_s) ? status.target_mm_s.join(" / ") : "—";
  $("controlMeasured").textContent = Array.isArray(status.measured_mm_s) ? status.measured_mm_s.join(" / ") : "—";
  const faultDetails = Array.isArray(state.fault_details) ? state.fault_details : [];
  $("controlFaults").textContent = faultDetails.length
    ? `${hex32(faults)} · ${faultDetails.map((fault) => fault.title).join("、")}`
    : hex32(faults);
  $("controlRevisions").textContent = `${status.active_revision ?? "—"} / ${status.stage_revision ?? "—"}`;
  const badge = $("controlBadge");
  const active = ["ARMED", "RUNNING", "RAMP_DOWN"].includes(actionState);
  if (playback.active && active) {
    exitPlayback();
    showToast("检测到车辆动作状态，已自动退出回放并恢复急停控制", true);
    return;
  }
  badge.className = `badge ${faults || state.status_error ? "bad" : "ok"}`;
  badge.textContent = state.status_error ? "状态读取失败，仅急停可用" : (faults ? "Safety 故障" : (active ? `动作 ${actionState}` : "可安全 Arm"));
  renderFaultDetails(faultDetails, faults, state.status_error);
  syncArmedState(state.armed);
  updateManualState(state);
  updateArmButton();
  if (playback.active) applyPlaybackLock(true);
}

function calibrationStep(stateName, failedStage = 0) {
  if (["WAIT_WHITE", "CAPTURE_WHITE"].includes(stateName)) return "white";
  if (["WAIT_BLACK", "CAPTURE_BLACK"].includes(stateName)) return "black";
  if (stateName === "FAILED") {
    if (failedStage === 1) return "white";
    if (failedStage === 2) return "black";
    return "save";
  }
  if (stateName === "REVIEW") return "review";
  if (["SAVING", "COMPLETE"].includes(stateName)) return "save";
  return "white";
}

function setCalibrationInterlock(active) {
  document.body.classList.toggle("calibration-active", active);
  const selector = ".control-section button:not(.estop), .control-section input, .control-section select, #parameterDialog button, #parameterDialog input, #parameterDialog select";
  document.querySelectorAll(selector).forEach((element) => {
    if (active) {
      if (element.dataset.calibrationDisabled == null)
        element.dataset.calibrationDisabled = element.disabled ? "1" : "0";
      element.disabled = true;
    } else if (element.dataset.calibrationDisabled != null) {
      element.disabled = element.dataset.calibrationDisabled === "1";
      delete element.dataset.calibrationDisabled;
    }
  });
}

function renderCalibration(state) {
  calibrationState = state || { available: false, active: false };
  const available = Boolean(calibrationState.available);
  const stateName = calibrationState.state_name || "IDLE";
  const ownerName = calibrationState.owner_name || "NONE";
  const active = Boolean(calibrationState.active);
  const hostOwner = ownerName === "HOST";
  const readonly = playback.active || (active && !hostOwner);
  $("openCalibrationButton").disabled = !available || playback.active;
  $("calibrationSummary").textContent = available
    ? `${calibrationState.active_source_name || "COMPILED"} · generation ${calibrationState.generation ?? "—"}`
    : "固件未提供自动标定能力";
  setCalibrationInterlock(active);
  const current = calibrationStep(stateName, numeric(calibrationState.failed_stage));
  const order = ["white", "black", "review", "save"];
  const currentIndex = order.indexOf(current);
  document.querySelectorAll("#calibrationSteps li").forEach((item) => {
    const index = order.indexOf(item.dataset.step);
    item.classList.toggle("current", index === currentIndex);
    item.classList.toggle("complete", index < currentIndex || stateName === "COMPLETE");
    item.setAttribute("aria-current", index === currentIndex ? "step" : "false");
  });
  const progress = numeric(calibrationState.progress);
  const required = numeric(calibrationState.required, 64);
  const progressElement = $("calibrationProgress");
  progressElement.max = required;
  progressElement.value = progress;
  progressElement.textContent = `${progress} / ${required}`;
  progressElement.setAttribute("aria-valuenow", String(progress));
  progressElement.setAttribute("aria-valuemax", String(required));
  $("calibrationProgressText").textContent = `${progress} / ${required}`;
  $("calibrationOwnerText").textContent = `所有者：${ownerName === "HOST" ? "当前网页" : ownerName === "REMOTE" ? "红外遥控器（只读查看）" : "无"}`;
  const titles = {
    IDLE: ["可以开始赛前标定", "确认车辆静止、无故障，先把全部 8 路探头放在白底上。"],
    WAIT_WHITE: ["第 1 步：准备白底", "全部探头覆盖同一块浅色背景后开始采集。"],
    CAPTURE_WHITE: ["正在采集白底", "保持车辆和光源完全不动，采集会自动完成。"],
    WAIT_BLACK: ["第 2 步：准备黑线", "把全部探头完整压在足够宽的黑线上，然后继续。"],
    CAPTURE_BLACK: ["正在采集黑线", "保持车辆静止，避免手或阴影遮挡探头。"],
    REVIEW: ["第 3 步：全部通道通过", "请复核白值、黑值、跨度和噪声；确认后才会写入 Flash。"],
    SAVING: ["正在写入并回读 Flash", "旧标定仍在使用，验证成功后才切换。"],
    COMPLETE: ["标定已保存并应用", "新的阈值已生效，循迹会等待新采样重新锁线。"],
    FAILED: ["本次标定未通过", "按通道查看原因，修正放置或光线后重试当前阶段。"],
    CANCELLED: ["标定已取消", "旧标定保持不变，可随时重新开始。"],
  };
  const message = titles[stateName] || ["标定状态未知", "请刷新后重试。"];
  $("calibrationStatusTitle").textContent = message[0];
  $("calibrationStatusDetail").textContent = message[1];
  const rows = $("calibrationRows");
  rows.innerHTML = "";
  for (let index = 0; index < 8; index += 1) {
    const failure = (calibrationState.failures || []).find((item) => numeric(item.channel) === index + 1);
    const row = document.createElement("tr");
    const values = [
      [`CH${index + 1}`, "通道"],
      [calibrationState.white_adc?.[index] ?? "—", "白底"],
      [calibrationState.black_adc?.[index] ?? "—", "黑线"],
      [calibrationState.span_adc?.[index] ?? "—", "跨度"],
      [calibrationState.max_noise_adc?.[index] ?? "—", "最大噪声"],
      [failure ? failure.reasons.join("；") : ((numeric(calibrationState.valid_mask) & (1 << index)) ? "通过" : "等待采样"), "检查结果"],
    ];
    values.forEach(([value, label], column) => {
      const cell = document.createElement("td");
      cell.dataset.label = label;
      cell.textContent = value;
      if (column === 5) cell.className = failure ? "channel-fail" : "channel-ok";
      row.appendChild(cell);
    });
    rows.appendChild(row);
  }
  const failures = calibrationState.failures || [];
  $("calibrationFailureSummary").hidden = !failures.length && !calibrationState.scan_error;
  $("calibrationFailureSummary").textContent = calibrationState.scan_error
    ? "扫描链路发生错误：请检查灰度 ADC 后重试当前阶段。"
    : failures.map((item) => `CH${item.channel}：${item.reasons.join("；")}`).join("\n");
  const primary = $("calibrationPrimaryButton");
  const actions = {
    IDLE: ["开始向导", "start"], CANCELLED: ["重新开始", "start"], COMPLETE: ["重新标定", "start"],
    WAIT_WHITE: ["采集白底 64 帧", "capture-white"], CAPTURE_WHITE: ["白底采集中…", null],
    WAIT_BLACK: ["采集黑线 64 帧", "capture-black"], CAPTURE_BLACK: ["黑线采集中…", null],
    REVIEW: ["确认并写入 Flash", "confirm"], SAVING: ["正在写入 Flash…", null],
    FAILED: [calibrationState.failed_stage === 1 ? "重试白底" : calibrationState.failed_stage === 2 ? "重试黑线" : "重试写入 Flash", calibrationState.failed_stage === 1 ? "capture-white" : calibrationState.failed_stage === 2 ? "capture-black" : "confirm"],
  };
  const [label, action] = actions[stateName] || ["刷新状态", null];
  primary.textContent = label;
  primary.dataset.action = action || "";
  primary.disabled = !action || readonly || !available;
  $("cancelCalibrationButton").disabled = !active || !hostOwner || playback.active;
}

async function pollCalibration(force = false) {
  if (calibrationFetching) return;
  calibrationFetching = true;
  try {
    const response = await fetch(`/api/calibration${force ? "?refresh=1" : ""}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    renderCalibration(await response.json());
  } catch (error) {
    $("calibrationStatusTitle").textContent = "标定状态读取失败";
    $("calibrationStatusDetail").textContent = error.message;
  } finally { calibrationFetching = false; }
}

async function runCalibrationAction(action, button) {
  const body = {expected_generation: calibrationState.generation};
  if (action === "confirm") body.confirm = "WRITE_GRAYSCALE_CALIBRATION_TO_FLASH";
  try {
    renderCalibration(await command(`/api/calibration/${action}`, body, button));
    showToast(action === "confirm" ? "灰度标定已写入 Flash 并应用" : "标定步骤已接受");
  } catch (error) {
    showToast(`标定操作失败：${error.message}`, true);
    await pollCalibration(true);
  }
}

function renderFaultDetails(details, faultBits, statusError) {
  const panel = $("faultPanel");
  panel.hidden = !faultBits;
  const list = $("faultList");
  list.replaceChildren();
  details.forEach((fault) => {
    const item = document.createElement("article");
    item.className = "fault-item";
    const heading = document.createElement("strong");
    heading.textContent = fault.title;
    const code = document.createElement("code");
    code.textContent = `${fault.hex} · ${fault.name}`;
    heading.appendChild(code);
    const cause = document.createElement("p");
    cause.textContent = `原因：${fault.cause}`;
    const recovery = document.createElement("p");
    recovery.textContent = `处理：${fault.recovery}`;
    item.append(heading, cause, recovery);
    list.appendChild(item);
  });
  $("clearFaultButton").disabled = playback.active || !faultBits || Boolean(statusError) || !controlToken;
}

function initializeActions(actions) {
  const select = $("actionKind");
  select.replaceChildren();
  actions.forEach((action) => {
    const option = document.createElement("option");
    option.value = action.name;
    option.textContent = ACTION_LABELS[action.name] || action.name;
    option.disabled = !action.available;
    if (!action.available) option.textContent += `（${action.unavailable_reason || "不可用"}）`;
    select.appendChild(option);
  });
  const available = actions.find((action) => action.available);
  if (available) select.value = available.name;
  actionsInitialized = true;
  $("actionAvailability").className = `badge ${available ? "ok" : "bad"}`;
  $("actionAvailability").textContent = available ? "动作能力已协商" : "没有可用动作";
  updateActionFields();
}

function updateActionFields() {
  const kind = $("actionKind").value;
  if (kind !== "LINE_FOLLOW") $("lineContinuous").checked = false;
  document.querySelectorAll(".action-input").forEach((field) => {
    const visible = field.dataset.kinds.split(" ").includes(kind);
    field.hidden = !visible;
    field.querySelectorAll("input, select").forEach((input) => { input.disabled = !visible; });
  });
  $("actionDuration").min = kind === "HEADING_TURN" ? "500" : "100";
  const hints = {
    WHEEL_SPEED_TEST: "独立验证左右轮速度环，可选择单轮或双轮。",
    HEADING_HOLD: "以相同左右轮速度直行，并使用陀螺仪保持航向。",
    HEADING_TURN: "原地完成指定角度；未协商可配置转角时仅支持 ±90°。",
    HEADING_RATE: "按给定角速度旋转，到持续时间后停止。",
    LINE_FOLLOW: "使用当前循迹参数运行；可按持续时间停止，或持续到手动取消、急停、丢线及传感器故障。",
  };
  $("actionHint").textContent = hints[kind] || "当前动作不可用";
  updateContinuousOption();
  updateArmButton();
}

function updateContinuousOption() {
  const enabled = $("actionKind").value === "LINE_FOLLOW" && $("lineContinuous").checked;
  $("actionDuration").disabled = enabled;
  $("actionDuration").closest(".form-field").classList.toggle("duration-disabled", enabled);
  $("lineContinuousOption").classList.toggle("continuous-enabled", enabled);
}

function updateArmButton() {
  const faults = numeric(controlState?.status?.fault_bits);
  const actionState = controlState?.status?.action_state_name || controlState?.status?.action_state?.name;
  const actionActive = ["ARMED", "RUNNING", "RAMP_DOWN"].includes(actionState);
  $("armButton").disabled = playback.active || manualDrive.enabled || !controlToken || !$("actionKind").value || !$("safetyAck").checked || Boolean(faults) || Boolean(controlState?.status_error) || actionActive || currentArmToken !== null;
}

function actionRequest() {
  const kind = $("actionKind").value;
  const wheelMask = numeric($("wheelMask").value, 3);
  const request = {
    kind,
    duration_ms: numeric($("actionDuration").value),
    wheel_mask: wheelMask,
    target_l_mm_s: 0,
    target_r_mm_s: 0,
    angle_deg: 0,
    yaw_rate_deg_s: 0,
    safety_ack: $("safetyAck").checked,
    continuous: kind === "LINE_FOLLOW" && $("lineContinuous").checked,
  };
  if (kind === "WHEEL_SPEED_TEST") {
    request.target_l_mm_s = wheelMask & 1 ? numeric($("targetL").value) : 0;
    request.target_r_mm_s = wheelMask & 2 ? numeric($("targetR").value) : 0;
  } else if (kind === "HEADING_HOLD") {
    request.target_l_mm_s = request.target_r_mm_s = numeric($("holdSpeed").value);
  } else if (kind === "HEADING_TURN") {
    request.angle_deg = numeric($("turnAngle").value);
  } else if (kind === "HEADING_RATE") {
    request.yaw_rate_deg_s = numeric($("yawRate").value);
  }
  return request;
}

function syncArmedState(armed) {
  if (!armed) {
    currentArmToken = null;
    armedDeadline = 0;
    $("confirmActionBar").hidden = true;
    $("confirmActionBar").classList.remove("continuous-confirm");
    $("confirmActionButton").textContent = "Confirm 开始运动";
    updateArmButton();
    return;
  }
  currentArmToken = armed.arm_token;
  armedDeadline = performance.now() + numeric(armed.expires_in_ms);
  $("confirmActionBar").hidden = false;
  const continuous = armed.kind_name === "LINE_FOLLOW" && numeric(armed.spec?.duration_ms, -1) === 0;
  $("confirmActionBar").classList.toggle("continuous-confirm", continuous);
  $("confirmActionButton").textContent = continuous ? "Confirm 持续循迹" : "Confirm 开始运动";
  $("armSummary").textContent = continuous
    ? "自动循迹 · 不按时间停止；关闭网页也会继续运行"
    : `${ACTION_LABELS[armed.kind_name] || armed.kind_name} · 最后确认不会修改参数`;
  updateCountdown();
  updateArmButton();
}

function updateCountdown() {
  if (currentArmToken === null) return;
  const remaining = Math.max(0, armedDeadline - performance.now());
  $("armCountdown").textContent = `${(remaining / 1000).toFixed(1)} s`;
  $("confirmActionButton").disabled = remaining <= 0;
  if (remaining <= 0) {
    currentArmToken = null;
    $("confirmActionBar").hidden = true;
    $("safetyAck").checked = false;
    updateArmButton();
  }
}

async function loadParameters(force = false) {
  if (parametersLoading) return;
  parametersLoading = true;
  const button = $("loadParametersButton");
  const prior = button.textContent;
  button.disabled = true;
  button.textContent = "读取中…";
  $("parameterPanel").hidden = false;
  try {
    const response = await fetch(`/api/parameters${force ? "?refresh=1" : ""}`, { cache: "no-store" });
    const state = await response.json();
    if (!response.ok || !state.ok) throw new Error(state.error?.message || `HTTP ${response.status}`);
    parameterData = state.parameters;
    renderParameters(state.summary);
    showToast(`已读取 ${parameterData.length} 个参数`);
  } catch (error) {
    $("parameterRows").innerHTML = `<tr><td colspan="5" class="empty-cell">参数读取失败：${escapeText(error.message)}</td></tr>`;
    showToast(error.message, true);
  } finally {
    button.disabled = false;
    button.textContent = prior;
    parametersLoading = false;
  }
}

function escapeText(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"})[character]);
}

function sameValue(left, right) {
  if (typeof left === "boolean" || typeof right === "boolean") return Boolean(left) === Boolean(right);
  return Math.abs(numeric(left) - numeric(right)) < 1e-6;
}

function renderParameters(summary) {
  $("activeRevision").textContent = summary.active_revision;
  $("stageRevision").textContent = summary.stage_revision;
  $("generation").textContent = summary.generation;
  const dirty = parameterData.some((parameter) => !sameValue(parameter.active, parameter.staged));
  $("dirtyState").className = `badge ${dirty ? "bad" : "ok"}`;
  $("dirtyState").textContent = dirty ? "存在未应用暂存值" : "Active 与 Stage 一致";
  const body = $("parameterRows");
  body.replaceChildren();
  parameterData.forEach((parameter) => body.appendChild(parameterRow(parameter)));
  filterParameters();
}

function parameterRow(parameter) {
  const row = document.createElement("tr");
  row.dataset.key = parameter.key;
  row.dataset.group = parameter.group;
  row.classList.toggle("dirty", !sameValue(parameter.active, parameter.staged));
  const cells = ["参数", "当前值", "暂存值", "范围 / 单位", "操作"].map((label) => {
    const cell = document.createElement("td"); cell.dataset.label = label; row.appendChild(cell); return cell;
  });
  const key = document.createElement("div"); key.className = "parameter-key";
  const keyName = document.createElement("strong"); keyName.textContent = parameter.key; key.appendChild(keyName);
  if (parameter.relations.length) { const relation = document.createElement("small"); relation.textContent = parameter.relations.join("；"); key.appendChild(relation); }
  cells[0].appendChild(key);
  cells[1].className = "active-value"; cells[1].textContent = formatParameter(parameter.active);
  let input;
  if (parameter.type === "BOOL") {
    input = document.createElement("select");
    input.innerHTML = '<option value="true">true</option><option value="false">false</option>';
    input.value = String(Boolean(parameter.staged));
  } else {
    input = document.createElement("input"); input.type = "number";
    input.min = parameter.minimum; input.max = parameter.maximum;
    input.step = Math.abs(numeric(parameter.maximum) - numeric(parameter.minimum)) <= 20 ? "0.01" : "1";
    input.value = parameter.staged;
  }
  input.setAttribute("aria-label", `${parameter.key} 暂存值`);
  cells[2].appendChild(input);
  const range = document.createElement("span"); range.className = "range-text";
  range.textContent = `${formatParameter(parameter.minimum)} … ${formatParameter(parameter.maximum)} · ${parameter.unit}`;
  cells[3].appendChild(range);
  const stage = document.createElement("button"); stage.type = "button"; stage.className = "secondary"; stage.textContent = "暂存";
  stage.addEventListener("click", async () => {
    const value = parameter.type === "BOOL" ? input.value === "true" : Number(input.value);
    if (!input.reportValidity()) return;
    try {
      const result = await command("/api/parameters/stage", {key: parameter.key, value}, stage);
      parameter.staged = result.result.staged;
      row.classList.toggle("dirty", !sameValue(parameter.active, parameter.staged));
      $("stageRevision").textContent = result.result.stage_revision;
      const dirty = parameterData.some((item) => !sameValue(item.active, item.staged));
      $("dirtyState").className = `badge ${dirty ? "bad" : "ok"}`;
      $("dirtyState").textContent = dirty ? "存在未应用暂存值" : "Active 与 Stage 一致";
      showToast(`${parameter.key} 已暂存，尚未应用`);
    } catch (error) { showToast(error.message, true); }
  });
  cells[4].appendChild(stage);
  return row;
}

function formatParameter(value) {
  if (typeof value === "boolean") return String(value);
  const number = numeric(value);
  return Number.isInteger(number) ? String(number) : number.toFixed(4).replace(/0+$/, "").replace(/\.$/, "");
}

function filterParameters() {
  const group = $("parameterGroup").value;
  const query = $("parameterSearch").value.trim().toLowerCase();
  document.querySelectorAll("#parameterRows tr[data-key]").forEach((row) => {
    row.hidden = (group !== "all" && row.dataset.group !== group) || (query && !row.dataset.key.toLowerCase().includes(query));
  });
}

async function parameterOperation(path, body, button, message) {
  try {
    await command(path, body, button);
    showToast(message);
    await loadParameters(true);
  } catch (error) { showToast(error.message, true); }
}

function values(stream) { return latest[stream]?.values || {}; }

function updateNumbers(nowMs) {
  const core = values("core");
  const wheel = values("wheel");
  const line = values("line");
  const health = values("health");
  $("speedL").textContent = fixed(wheel.filtered_l_mm_s ?? core.speed_l_mm_s);
  $("speedR").textContent = fixed(wheel.filtered_r_mm_s ?? core.speed_r_mm_s);
  $("linePosition").textContent = fixed(line.position_x1000 == null ? null : line.position_x1000 / 1000, 3);
  $("lineConfidence").textContent = fixed(line.confidence_x1000 == null ? null : line.confidence_x1000 / 1000, 3);
  $("yaw").textContent = fixed(core.yaw_cdeg == null ? null : core.yaw_cdeg / 100, 2);
  const faults = core.fault_bits ?? health.fault_bits;
  $("faultBits").textContent = hex32(faults);
  $("faultBits").style.color = numeric(faults) ? "#f87171" : "";

  const adc = Array.isArray(line.adc) ? line.adc : [];
  const mask = numeric(line.active_mask);
  $("maskText").textContent = line.active_mask == null ? "mask —" : `mask ${hex32(mask).slice(-4)}`;
  for (let i = 0; i < 8; i += 1) {
    const sensor = $(`sensor${i}`);
    const value = numeric(adc[i]);
    sensor.classList.toggle("active", Boolean(mask & (1 << i)));
    sensor.querySelector(".sensor-value").textContent = adc[i] == null ? "—" : value;
    sensor.querySelector(".sensor-fill").style.height = `${Math.min(100, Math.max(0, value / 4095 * 100))}%`;
  }

  const taskMiss = Array.isArray(health.task_miss) ? health.task_miss : [];
  const healthBad = numeric(health.fault_bits) !== 0 || taskMiss.some((value) => numeric(value) !== 0) || numeric(health.uart_tx_drop) || numeric(health.uart_rx_drop);
  const badge = $("healthBadge");
  if (!latest.health) { badge.className = "badge waiting"; badge.textContent = "等待 HEALTH"; }
  else { badge.className = `badge ${healthBad ? "bad" : "ok"}`; badge.textContent = healthBad ? "存在异常" : "状态正常"; }
  $("systemState").textContent = health.system_state ?? "—";
  $("taskMiss").textContent = taskMiss.length ? taskMiss.join(" / ") : "—";
  $("uartDrop").textContent = health.uart_tx_drop == null ? "—" : `${health.uart_tx_drop} / ${health.uart_rx_drop}`;
  $("telemetryDrop").textContent = health.p2_drop == null ? "—" : `${health.p2_drop} / ${health.p3_drop}`;
  $("telemetryQueue").textContent = health.telemetry_queue ?? "—";
  const age = ["core", "wheel", "line"].map((stream) => latest[stream] ? `${Math.max(0, nowMs - latest[stream].received_ms)} ms` : "—");
  $("dataAge").textContent = age.join(" / ");
}

function canvasSetup(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function drawStepperTrend(canvasId, key, color) {
  const canvas = $(canvasId);
  const {ctx, width, height} = canvasSetup(canvas);
  const now = performance.now();
  const points = stepperView.samples.filter((sample) => sample.at >= now - 10000);
  ctx.clearRect(0, 0, width, height);
  const pad = {left:48, right:10, top:12, bottom:24};
  const plotW = Math.max(1, width - pad.left - pad.right);
  const plotH = Math.max(1, height - pad.top - pad.bottom);
  let minimum = Math.min(0, ...points.map((point) => point[key]));
  let maximum = Math.max(0, ...points.map((point) => point[key]));
  if (minimum === maximum) { minimum -= 1; maximum += 1; }
  const margin = Math.max(1, (maximum - minimum) * .08);
  minimum -= margin; maximum += margin;
  ctx.font = "9px ui-monospace, Consolas, monospace";
  ctx.textBaseline = "middle";
  for (let index = 0; index <= 3; index += 1) {
    const y = pad.top + plotH * index / 3;
    const value = maximum - (maximum - minimum) * index / 3;
    ctx.strokeStyle = "#1c2a3d"; ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
    ctx.fillStyle = "#728096"; ctx.textAlign = "right"; ctx.fillText(formatAxis(value), pad.left - 6, y);
  }
  for (let seconds = 0; seconds <= 10; seconds += 5) {
    const x = width - pad.right - seconds / 10 * plotW;
    ctx.fillStyle = "#728096"; ctx.textAlign = "center"; ctx.fillText(seconds ? `-${seconds}s` : "现在", x, height - 9);
  }
  if (minimum < 0 && maximum > 0) {
    const y = pad.top + maximum / (maximum - minimum) * plotH;
    ctx.strokeStyle = "#475569"; ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
  }
  if (!points.length) {
    ctx.fillStyle = "#64748b"; ctx.textAlign = "center"; ctx.fillText("等待步进反馈", pad.left + plotW / 2, pad.top + plotH / 2);
    return;
  }
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
  points.forEach((point, index) => {
    const x = pad.left + clamp((point.at - (now - 10000)) / 10000, 0, 1) * plotW;
    const y = pad.top + (maximum - point[key]) / (maximum - minimum) * plotH;
    if (index) ctx.lineTo(x, y); else ctx.moveTo(x, y);
  });
  ctx.stroke();
}

function collectSeries(definition, startMs) {
  const result = [];
  const events = history[definition.stream] || [];
  for (const event of events) {
    if (event.received_ms < startMs) continue;
    const raw = event.values[definition.key];
    if (raw == null || !Number.isFinite(Number(raw))) continue;
    result.push([event.received_ms, Number(raw) * (definition.scale ?? 1)]);
  }
  return result;
}

function niceRange(series, forcedRange) {
  if (forcedRange) return forcedRange;
  let min = Infinity, max = -Infinity;
  series.forEach((entry) => entry.points.forEach((point) => { min = Math.min(min, point[1]); max = Math.max(max, point[1]); }));
  if (!Number.isFinite(min)) return [-1, 1];
  if (min > 0) min = 0;
  if (max < 0) max = 0;
  if (min === max) { const pad = Math.max(1, Math.abs(min) * .1); min -= pad; max += pad; }
  const pad = (max - min) * .08;
  return [min - pad, max + pad];
}

function drawChart(canvasId, definitions, nowMs, options = {}) {
  const canvas = $(canvasId);
  const { ctx, width, height } = canvasSetup(canvas);
  const hasSecondary = definitions.some((definition) => definition.secondary);
  const pad = { left: 52, right: hasSecondary ? 52 : 16, top: 16, bottom: 26 };
  const plotW = Math.max(1, width - pad.left - pad.right);
  const plotH = Math.max(1, height - pad.top - pad.bottom);
  const startMs = nowMs - windowSeconds * 1000;
  const series = definitions.map((definition) => ({ ...definition, points: collectSeries(definition, startMs) }));
  const [yMin, yMax] = niceRange(series.filter((item) => !item.secondary), options.range);
  const secondary = series.filter((item) => item.secondary);
  const [sMin, sMax] = niceRange(secondary, options.secondaryRange);
  ctx.clearRect(0, 0, width, height);
  ctx.font = "10px ui-monospace, Consolas, monospace";
  ctx.textBaseline = "middle";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + plotH * i / 4;
    ctx.strokeStyle = "#1c2a3d";
    ctx.beginPath(); ctx.moveTo(pad.left, y + .5); ctx.lineTo(width - pad.right, y + .5); ctx.stroke();
    const value = yMax - (yMax - yMin) * i / 4;
    ctx.fillStyle = "#728096"; ctx.textAlign = "right"; ctx.fillText(formatAxis(value), pad.left - 8, y);
    if (hasSecondary) {
      const secondaryValue = sMax - (sMax - sMin) * i / 4;
      ctx.textAlign = "left";
      ctx.fillText(formatAxis(secondaryValue), width - pad.right + 8, y);
    }
  }
  for (let i = 0; i <= windowSeconds; i += windowSeconds <= 5 ? 1 : 2) {
    const x = width - pad.right - plotW * i / windowSeconds;
    ctx.strokeStyle = "#132034"; ctx.beginPath(); ctx.moveTo(x + .5, pad.top); ctx.lineTo(x + .5, height - pad.bottom); ctx.stroke();
    ctx.fillStyle = "#728096"; ctx.textAlign = "center"; ctx.fillText(i === 0 ? "现在" : `-${i}s`, x, height - 11);
  }
  if (yMin < 0 && yMax > 0) {
    const zeroY = pad.top + (yMax / (yMax - yMin)) * plotH;
    ctx.strokeStyle = "#475569"; ctx.beginPath(); ctx.moveTo(pad.left, zeroY); ctx.lineTo(width - pad.right, zeroY); ctx.stroke();
  }
  series.forEach((item) => {
    if (item.points.length < 1) return;
    const min = item.secondary ? sMin : yMin;
    const max = item.secondary ? sMax : yMax;
    ctx.strokeStyle = item.color;
    ctx.lineWidth = item.width || 1.6;
    ctx.setLineDash(item.dash ? [5, 4] : []);
    ctx.beginPath();
    item.points.forEach(([time, value], index) => {
      const x = pad.left + (time - startMs) / (windowSeconds * 1000) * plotW;
      const y = pad.top + (max - value) / (max - min) * plotH;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  ctx.setLineDash([]);
  if (!series.some((item) => item.points.length)) {
    ctx.fillStyle = "#64748b"; ctx.textAlign = "center"; ctx.font = "12px Segoe UI, sans-serif";
    ctx.fillText("等待遥测数据", pad.left + plotW / 2, pad.top + plotH / 2);
  }
}

function roundedRectPath(ctx, x, y, width, height, radius) {
  const r = Math.min(radius, width / 2, height / 2);
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + width, y, x + width, y + height, r);
  ctx.arcTo(x + width, y + height, x, y + height, r);
  ctx.arcTo(x, y + height, x, y, r);
  ctx.arcTo(x, y, x + width, y, r);
  ctx.closePath();
}

function drawMicroTrend(ctx, area, series, startMs, nowMs, range, label) {
  const [minimum, maximum] = range;
  roundedRectPath(ctx, area.x, area.y, area.width, area.height, 5);
  ctx.fillStyle = "#020817";
  ctx.fill();
  ctx.strokeStyle = "#1c2a3d";
  ctx.stroke();
  if (minimum < 0 && maximum > 0) {
    const zeroY = area.y + (maximum / (maximum - minimum)) * area.height;
    ctx.strokeStyle = "#334155";
    ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(area.x, zeroY); ctx.lineTo(area.x + area.width, zeroY); ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.save();
  roundedRectPath(ctx, area.x, area.y, area.width, area.height, 5);
  ctx.clip();
  series.forEach((item) => {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = item.width || 1.5;
    ctx.setLineDash(item.dash ? [4, 4] : []);
    ctx.beginPath();
    item.points.forEach(([time, value], index) => {
      const x = area.x + clamp((time - startMs) / Math.max(1, nowMs - startMs), 0, 1) * area.width;
      const y = area.y + (maximum - clamp(value, minimum, maximum)) / (maximum - minimum) * area.height;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  ctx.restore();
  ctx.setLineDash([]);
  ctx.fillStyle = "#728096";
  ctx.font = "9px ui-monospace, Consolas, monospace";
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(label, area.x + 6, area.y + 4);
  ctx.textAlign = "right";
  ctx.fillText("5s → 现在", area.x + area.width - 6, area.y + 4);
}

function drawLineVisual(canvasId, nowMs) {
  const canvas = $(canvasId);
  const { ctx, width, height } = canvasSetup(canvas);
  const line = values("line");
  const hasData = latest.line && line.position_x1000 != null;
  const position = clamp(numeric(line.position_x1000) / 1000, -1, 1);
  const rawPosition = clamp(numeric(line.raw_position_x1000 ?? line.position_x1000) / 1000, -1, 1);
  const confidence = clamp(numeric(line.confidence_x1000) / 1000, 0, 1);
  const visualBottom = Math.max(92, Math.round(height * .61));
  const trend = { x: 12, y: visualBottom + 5, width: Math.max(1, width - 24), height: Math.max(34, height - visualBottom - 12) };
  const track = { x: 22, y: 36, width: Math.max(1, width - 44), height: Math.max(36, visualBottom - 58) };
  ctx.clearRect(0, 0, width, height);
  roundedRectPath(ctx, track.x, track.y, track.width, track.height, 9);
  ctx.fillStyle = "#050c18"; ctx.fill();
  ctx.strokeStyle = "#334155"; ctx.lineWidth = 1; ctx.stroke();
  for (let i = 0; i <= 4; i += 1) {
    const x = track.x + track.width * i / 4;
    ctx.strokeStyle = i === 2 ? "#64748b" : "#243145";
    ctx.setLineDash(i === 2 ? [5, 4] : []);
    ctx.beginPath(); ctx.moveTo(x, track.y + 5); ctx.lineTo(x, track.y + track.height - 5); ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.fillStyle = "#728096"; ctx.font = "9px ui-monospace, Consolas, monospace"; ctx.textBaseline = "middle";
  ctx.textAlign = "left"; ctx.fillText("左 -1", track.x, track.y - 10);
  ctx.textAlign = "center"; ctx.fillText("轨迹中心", track.x + track.width / 2, track.y - 10);
  ctx.textAlign = "right"; ctx.fillText("+1 右", track.x + track.width, track.y - 10);
  if (hasData) {
    const rawX = track.x + (rawPosition + 1) / 2 * track.width;
    const carX = track.x + (position + 1) / 2 * track.width;
    ctx.strokeStyle = COLORS.teal; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(rawX, track.y + 3); ctx.lineTo(rawX, track.y + track.height - 3); ctx.stroke();
    ctx.setLineDash([]);
    ctx.save(); ctx.translate(carX, track.y + track.height / 2);
    ctx.fillStyle = COLORS.green; ctx.shadowColor = "#34d39980"; ctx.shadowBlur = 10;
    roundedRectPath(ctx, -11, -17, 22, 34, 5); ctx.fill();
    ctx.shadowBlur = 0; ctx.fillStyle = "#052e2b";
    ctx.beginPath(); ctx.moveTo(0, -11); ctx.lineTo(-5, -3); ctx.lineTo(5, -3); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  const confidenceWidth = Math.max(1, track.width * confidence);
  roundedRectPath(ctx, track.x, track.y + track.height + 7, track.width, 5, 3); ctx.fillStyle = "#1c2a3d"; ctx.fill();
  roundedRectPath(ctx, track.x, track.y + track.height + 7, confidenceWidth, 5, 3); ctx.fillStyle = confidence >= .5 ? COLORS.yellow : "#f87171"; ctx.fill();
  ctx.fillStyle = "#cbd5e1"; ctx.textAlign = "right"; ctx.textBaseline = "bottom"; ctx.font = "10px ui-monospace, Consolas, monospace";
  ctx.fillText(hasData ? `位置 ${position.toFixed(3)} · 置信度 ${confidence.toFixed(3)}` : "等待 LINE 数据", width - 22, 25);
  const startMs = nowMs - MICRO_TREND_MS;
  drawMicroTrend(ctx, trend, [
    { points: collectSeries({stream:"line", key:"position_x1000", scale:.001}, startMs), color: COLORS.green, width: 1.8 },
    { points: collectSeries({stream:"line", key:"raw_position_x1000", scale:.001}, startMs), color: COLORS.teal, dash: true },
  ], startMs, nowMs, [-1.05, 1.05], "位置微趋势");
}

function drawGyroVisual(canvasId, nowMs) {
  const canvas = $(canvasId);
  const { ctx, width, height } = canvasSetup(canvas);
  const core = values("core");
  const hasData = latest.core && core.yaw_cdeg != null;
  const yaw = numeric(core.yaw_cdeg) / 100;
  const yawRate = numeric(core.yaw_rate_cdeg_s) / 100;
  const visualBottom = Math.max(96, Math.round(height * .63));
  const radius = clamp(Math.min((visualBottom - 24) / 2, width * .14), 27, 58);
  const centerX = Math.max(radius + 14, Math.min(width * .27, 132));
  const centerY = Math.max(radius + 10, visualBottom / 2 + 3);
  ctx.clearRect(0, 0, width, height);
  ctx.save(); ctx.translate(centerX, centerY);
  ctx.strokeStyle = "#334155"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(0, 0, radius, 0, Math.PI * 2); ctx.stroke();
  for (let i = 0; i < 24; i += 1) {
    const angle = i / 24 * Math.PI * 2 - Math.PI / 2;
    const inner = radius - (i % 6 === 0 ? 8 : 4);
    ctx.strokeStyle = i % 6 === 0 ? "#94a3b8" : "#334155";
    ctx.lineWidth = i % 6 === 0 ? 1.5 : 1;
    ctx.beginPath(); ctx.moveTo(Math.cos(angle) * inner, Math.sin(angle) * inner); ctx.lineTo(Math.cos(angle) * radius, Math.sin(angle) * radius); ctx.stroke();
  }
  ctx.fillStyle = "#94a3b8"; ctx.font = "9px ui-monospace, Consolas, monospace"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  [["N",0,-radius+13],["E",radius-13,0],["S",0,radius-13],["W",-radius+13,0]].forEach(([label,x,y]) => ctx.fillText(label, x, y));
  if (hasData) {
    ctx.rotate(yaw * Math.PI / 180);
    ctx.fillStyle = COLORS.violet; ctx.shadowColor = "#a78bfa80"; ctx.shadowBlur = 9;
    ctx.beginPath(); ctx.moveTo(0, -radius + 13); ctx.lineTo(-8, 13); ctx.lineTo(0, 8); ctx.lineTo(8, 13); ctx.closePath(); ctx.fill();
  }
  ctx.restore();
  const infoX = Math.min(width - 96, centerX + radius + 20);
  ctx.textAlign = "left"; ctx.textBaseline = "middle";
  ctx.fillStyle = "#94a3b8"; ctx.font = "10px Segoe UI, sans-serif"; ctx.fillText("偏航角", infoX, Math.max(23, centerY - 25));
  ctx.fillStyle = COLORS.violet; ctx.font = "700 21px ui-monospace, Consolas, monospace"; ctx.fillText(hasData ? `${yaw.toFixed(2)}°` : "—", infoX, Math.max(43, centerY - 5));
  ctx.fillStyle = "#94a3b8"; ctx.font = "10px Segoe UI, sans-serif"; ctx.fillText("角速度", infoX, centerY + 22);
  ctx.fillStyle = Math.abs(yawRate) > 120 ? "#f87171" : COLORS.pink; ctx.font = "700 14px ui-monospace, Consolas, monospace"; ctx.fillText(hasData ? `${yawRate.toFixed(1)}°/s` : "—", infoX, centerY + 39);
  const ratePoints = collectSeries({stream:"core", key:"yaw_rate_cdeg_s", scale:.01}, nowMs - MICRO_TREND_MS);
  const peak = Math.max(30, ...ratePoints.map((point) => Math.abs(point[1])));
  const limit = Math.ceil(peak / 10) * 10;
  drawMicroTrend(ctx, {x:12, y:visualBottom + 4, width:Math.max(1, width - 24), height:Math.max(34, height - visualBottom - 11)}, [
    { points: ratePoints, color: COLORS.pink, width: 1.8 },
  ], nowMs - MICRO_TREND_MS, nowMs, [-limit, limit], `角速度 ±${limit}°/s`);
}

function formatAxis(value) {
  const abs = Math.abs(value);
  if (abs >= 1000) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(0);
  return value.toFixed(1);
}

function applyPlaybackLock(locked) {
  const selectors = [
    "#vehicleControl button", "#vehicleControl input", "#vehicleControl select",
    "#stepperTest button", "#stepperTest input", "#stepperTest select",
    "#parameterDialog button", "#parameterDialog input", "#parameterDialog select",
  ];
  document.querySelectorAll(selectors.join(",")).forEach((control) => { control.disabled = locked; });
  $("openParametersButton").disabled = locked;
  $("startRecordingButton").disabled = locked || recordingState.recording;
  if (!locked) {
    updateActionFields();
    updateArmButton();
    if (controlState) renderFaultDetails(controlState.fault_details || [], numeric(controlState.status?.fault_bits), controlState.status_error);
    updateStepperControls();
  }
}

function updateRecordingStatus(state) {
  recordingState = state || {recording:false};
  const active = Boolean(recordingState.recording);
  $("recordingStatus").hidden = !active;
  $("startRecordingButton").hidden = active;
  $("addMarkerButton").hidden = !active;
  $("stopRecordingButton").hidden = !active;
  $("startRecordingButton").disabled = playback.active || active;
  if (active) {
    $("recordingElapsed").textContent = formatDuration(recordingState.elapsed_ms);
    $("recordingSize").textContent = `${formatBytes(recordingState.bytes)}${recordingState.post_roll ? " · 后录" : ""}`;
    $("recordingStatus").title = `${recordingState.name || "录制"} · ${recordingState.trigger === "action" ? "动作联动" : "手动"}`;
  }
}

async function pollRecordingStatus() {
  if (recordingFetching) return;
  recordingFetching = true;
  try {
    const response = await fetch("/api/recordings/status", {cache:"no-store"});
    const state = await response.json();
    if (!response.ok || !state.ok) throw new Error(state.error?.message || `HTTP ${response.status}`);
    updateRecordingStatus(state);
  } catch (_error) {
    // The live telemetry dashboard remains useful if the recording disk is unavailable.
  } finally { recordingFetching = false; }
}

function recordingActionName(action) {
  const raw = action?.kind_name || action?.kind?.name || action?.kind;
  return ACTION_LABELS[raw] || raw || "—";
}

async function loadRecordingLibrary() {
  const root = $("recordingLibraryList");
  root.replaceChildren();
  const loading = document.createElement("p");
  loading.className = "empty-library";
  loading.textContent = "正在读取会话…";
  root.appendChild(loading);
  try {
    const response = await fetch("/api/recordings", {cache:"no-store"});
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error?.message || `HTTP ${response.status}`);
    root.replaceChildren();
    if (!payload.recordings.length) {
      const empty = document.createElement("p");
      empty.className = "empty-library";
      empty.textContent = "还没有仪表盘录制。";
      root.appendChild(empty);
      return;
    }
    payload.recordings.forEach((item) => root.appendChild(recordingCard(item)));
  } catch (error) {
    root.replaceChildren();
    const failed = document.createElement("p");
    failed.className = "empty-library";
    failed.textContent = `读取失败：${error.message}`;
    root.appendChild(failed);
  }
}

function recordingCard(item) {
  const card = document.createElement("article");
  card.className = `recording-card${item.complete ? "" : " incomplete"}`;
  const identity = document.createElement("div");
  const title = document.createElement("h3"); title.textContent = item.name || item.session_id;
  const notes = document.createElement("p"); notes.textContent = item.notes || item.session_id;
  identity.append(title, notes);
  const date = document.createElement("div"); date.className = "recording-card-data";
  const dateLabel = document.createElement("small"); dateLabel.textContent = "开始时间";
  const dateValue = document.createElement("strong");
  const parsedDate = new Date(item.start_time_utc);
  dateValue.textContent = Number.isNaN(parsedDate.getTime()) ? "—" : parsedDate.toLocaleString("zh-CN", {month:"2-digit", day:"2-digit", hour:"2-digit", minute:"2-digit"});
  date.append(dateLabel, dateValue);
  const duration = document.createElement("div"); duration.className = "recording-card-data";
  const durationLabel = document.createElement("small"); durationLabel.textContent = "时长 / 大小";
  const durationValue = document.createElement("strong"); durationValue.textContent = `${formatDuration(item.duration_ms)} · ${formatBytes(item.bytes)}`;
  duration.append(durationLabel, durationValue);
  const trigger = document.createElement("div"); trigger.className = "recording-card-data";
  const triggerLabel = document.createElement("small"); triggerLabel.textContent = item.trigger === "action" ? "联动动作" : "触发方式";
  const triggerValue = document.createElement("strong"); triggerValue.textContent = item.trigger === "action" ? recordingActionName(item.action) : "手动";
  trigger.append(triggerLabel, triggerValue);
  const actions = document.createElement("div"); actions.className = "recording-card-actions";
  const integrity = document.createElement("span"); integrity.className = item.complete ? "integrity-ok" : "integrity-bad"; integrity.textContent = item.complete ? "完整" : "未完整"; integrity.title = item.stop_reason || "";
  const replay = document.createElement("button"); replay.type = "button"; replay.textContent = "回放"; replay.disabled = recordingState.recording;
  replay.addEventListener("click", () => openPlayback(item.session_id));
  const download = document.createElement("a"); download.textContent = "下载"; download.href = `/api/recordings/${encodeURIComponent(item.session_id)}/download`; download.download = `m6-${item.session_id}.zip`;
  actions.append(integrity, replay, download);
  card.append(identity, date, duration, trigger, actions);
  return card;
}

function playbackTelemetryRecord(record) {
  return {
    cursor: record.record_index,
    received_ms: record.elapsed_ms,
    stream: record.stream,
    sample_id: record.sample_id,
    time_ms: record.device_time_ms,
    gap: Boolean(record.gap),
    missing: record.gap ? 1 : 0,
    values: record.values || {},
  };
}

function processPlaybackRecords(untilMs, announceMarkers = true) {
  while (playback.position < playback.records.length && numeric(playback.records[playback.position].elapsed_ms) <= untilMs) {
    const record = playback.records[playback.position];
    playback.position += 1;
    if (record.record_type === "telemetry" && history[record.stream]) {
      appendEvent(playbackTelemetryRecord(record));
      counts[record.stream] = numeric(counts[record.stream]) + 1;
      if (record.gap) gaps[record.stream] = numeric(gaps[record.stream]) + 1;
    } else if (record.record_type === "uart_data" && record.stream === "vision_uart_rx") {
      appendEvent(playbackTelemetryRecord(record));
      counts[record.stream] = numeric(counts[record.stream]) + 1;
      if (record.gap) gaps[record.stream] = numeric(gaps[record.stream]) + 1;
    } else if (announceMarkers && record.event === "marker") {
      showToast(`回放标记：${record.values?.label || "未命名"}`);
    }
  }
  $("sampleCount").textContent = Object.values(counts).reduce((a, b) => a + numeric(b), 0).toLocaleString();
  $("gapCount").textContent = Object.values(gaps).reduce((a, b) => a + numeric(b), 0).toLocaleString();
}

async function fetchPlaybackPage({fromMs = 0, after = 0, generation = playback.generation} = {}) {
  const response = await fetch(`/api/recordings/${encodeURIComponent(playback.sessionId)}/timeline?from_ms=${Math.max(0, Math.round(fromMs))}&after=${Math.max(0, after)}&limit=5000`, {cache:"no-store"});
  const payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(payload.error?.message || `HTTP ${response.status}`);
  if (!playback.active || generation !== playback.generation) return null;
  playback.after = payload.after;
  playback.hasMore = payload.has_more;
  return payload.records;
}

async function ensurePlaybackBuffer() {
  if (!playback.active || playback.loading || !playback.hasMore) return;
  const last = playback.records.at(-1);
  if (last && numeric(last.elapsed_ms) - playback.currentMs > 15000) return;
  playback.loading = true;
  const generation = playback.generation;
  try {
    const records = await fetchPlaybackPage({after:playback.after, generation});
    if (records) playback.records.push(...records);
  } catch (error) {
    playback.playing = false;
    showToast(`回放读取失败：${error.message}`, true);
  } finally { playback.loading = false; }
}

async function seekPlayback(targetMs) {
  if (!playback.active) return;
  const target = clamp(numeric(targetMs), 0, playback.durationMs);
  const generation = ++playback.generation;
  playback.playing = false;
  playback.lastFrame = 0;
  playback.loading = true;
  $("playbackPlayButton").textContent = "播放";
  const warmupMs = Math.max(0, target - windowSeconds * 1000);
  clearHistory(false);
  cameraView.status = { available: true, enabled: false };
  updateCameraStatus({
    message: "历史回放 · 此时间范围尚无 UART1 数据",
    last_received_ms: null,
    rx_bytes: 0, tx_bytes: 0, rx_overflow: 0, tx_overflow: 0,
    bridge_dropped: 0, hardware_errors: 0, dashboard_now_ms: target,
  });
  counts = {}; gaps = {};
  playback.records = [];
  playback.position = 0;
  playback.after = 0;
  try {
    let records = await fetchPlaybackPage({fromMs:warmupMs, after:0, generation});
    if (records === null) return;
    playback.records.push(...records);
    while (playback.hasMore && numeric(playback.records.at(-1)?.elapsed_ms) < target + 5000) {
      records = await fetchPlaybackPage({after:playback.after, generation});
      if (records === null || !records.length) break;
      playback.records.push(...records);
    }
    playback.currentMs = target;
    processPlaybackRecords(target, false);
    updatePlaybackControls();
    dirty = true;
  } catch (error) {
    showToast(`定位回放失败：${error.message}`, true);
  } finally {
    if (generation === playback.generation) playback.loading = false;
  }
}

function updatePlaybackControls() {
  $("playbackProgress").value = String(Math.round(playback.currentMs));
  $("playbackTime").textContent = `${formatDuration(playback.currentMs, true)} / ${formatDuration(playback.durationMs, true)}`;
}

async function openPlayback(sessionId) {
  if (recordingState.recording) { showToast("请先停止当前录制，再进入回放", true); return; }
  const actionState = controlState?.status?.action_state_name || controlState?.status?.action_state?.name;
  if (!controlState || controlState.status_error || controlState.connected === false || !actionState) {
    showToast("无法确认车辆已停止，暂不能进入会禁用急停的回放模式", true);
    return;
  }
  if (["ARMED", "RUNNING", "RAMP_DOWN"].includes(actionState)) { showToast("车辆动作进行中，不能进入只读回放", true); return; }
  if ([1, 2].includes(numeric(stepperView.snapshot?.state))) { showToast("步进电机测试正在运行，不能进入历史回放；请先等待完成或按步进立即停止", true); return; }
  try {
    if (manualDrive.enabled) await disableManualDrive("已进入历史回放");
    if (playback.active) exitPlayback();
    const response = await fetch(`/api/recordings/${encodeURIComponent(sessionId)}/manifest`, {cache:"no-store"});
    const manifest = await response.json();
    if (!response.ok) throw new Error(manifest.error?.message || `HTTP ${response.status}`);
    playback.active = true;
    playback.sessionId = sessionId;
    playback.manifest = manifest;
    playback.durationMs = numeric(manifest.duration_ms);
    playback.currentMs = 0;
    playback.speed = numeric($("playbackSpeed").value, 1);
    playback.hasMore = true;
    playback.after = 0;
    document.body.classList.add("playback-mode");
    $("liveToolbar").hidden = true;
    $("playbackBar").hidden = false;
    $("playbackReadonlyNotice").hidden = false;
    $("playbackName").textContent = manifest.name || sessionId;
    $("playbackProgress").max = String(playback.durationMs);
    if ($("parameterDialog").open) $("parameterDialog").close();
    if ($("recordingLibraryDialog").open) $("recordingLibraryDialog").close();
    applyPlaybackLock(true);
    $("connectionDot").className = "status-dot waiting";
    $("connectionText").textContent = "历史回放 · 实时注入已暂停";
    $("deviceText").textContent = `${manifest.firmware_identity?.firmware_id || "未知固件"} · ${manifest.start_time_utc || ""}`;
    await seekPlayback(0);
    showToast("已进入只读历史回放");
  } catch (error) {
    if (playback.active) exitPlayback();
    showToast(`无法开始回放：${error.message}`, true);
  }
}

function exitPlayback() {
  if (!playback.active) return;
  playback.generation += 1;
  playback.active = false;
  playback.playing = false;
  playback.sessionId = null;
  playback.manifest = null;
  playback.records = [];
  playback.position = 0;
  playback.after = 0;
  playback.hasMore = false;
  playback.currentMs = 0;
  document.body.classList.remove("playback-mode");
  $("playbackBar").hidden = true;
  $("liveToolbar").hidden = false;
  $("playbackReadonlyNotice").hidden = true;
  $("playbackPlayButton").textContent = "播放";
  applyPlaybackLock(false);
  clearHistory(true);
  counts = {}; gaps = {}; lastServerOk = 0;
  poll(); pollControl(); pollStepper();
  showToast("已退出回放并恢复实时数据");
}

function advancePlayback(frameNow) {
  if (!playback.active || !playback.playing || playback.loading) return;
  if (!playback.lastFrame) { playback.lastFrame = frameNow; return; }
  const delta = Math.min(250, frameNow - playback.lastFrame) * playback.speed;
  playback.lastFrame = frameNow;
  playback.currentMs = Math.min(playback.durationMs, playback.currentMs + delta);
  processPlaybackRecords(playback.currentMs, true);
  updatePlaybackControls();
  ensurePlaybackBuffer();
  dirty = true;
  if (playback.currentMs >= playback.durationMs) {
    playback.playing = false;
    $("playbackPlayButton").textContent = "播放";
    showToast("回放已结束");
  }
}

function render(frameNow) {
  advancePlayback(frameNow);
  if (!dirty || (!playback.active && paused)) { requestAnimationFrame(render); return; }
  dirty = false;
  const newest = Math.max(0, ...Object.values(latest).map((event) => event.received_ms));
  const hostNow = playback.active ? playback.currentMs : (lastServerOk ? newest + (performance.now() - lastServerOk) : newest);
  updateCameraStatus({ dashboard_now_ms: hostNow });
  trimHistory(hostNow);
  updateNumbers(Math.round(hostNow));
  drawChart("speedChart", [
    { stream: "core", key: "speed_l_mm_s", color: COLORS.cyan },
    { stream: "core", key: "speed_r_mm_s", color: COLORS.orange },
    { stream: "core", key: "request_l_mm_s", color: COLORS.blue, dash: true },
    { stream: "core", key: "request_r_mm_s", color: COLORS.amber, dash: true },
  ], hostNow);
  drawChart("motorChart", [
    { stream: "core", key: "pwm_l", color: COLORS.cyan },
    { stream: "core", key: "pwm_r", color: COLORS.orange },
    { stream: "wheel", key: "error_l_mm_s", color: COLORS.violet, dash: true, secondary: true },
    { stream: "wheel", key: "error_r_mm_s", color: COLORS.pink, dash: true, secondary: true },
  ], hostNow);
  drawLineVisual("lineChart", hostNow);
  drawGyroVisual("gyroChart", hostNow);
  drawStepperTrend("stepperSpeedChart", "speed", COLORS.amber);
  drawStepperTrend("stepperPositionChart", "position", COLORS.cyan);
  requestAnimationFrame(render);
}

$("pauseButton").addEventListener("click", () => {
  paused = !paused;
  $("pauseButton").textContent = paused ? "继续" : "暂停";
  $("pauseButton").setAttribute("aria-pressed", String(paused));
  if (!paused) dirty = true;
});
$("clearButton").addEventListener("click", () => clearHistory(false));
$("stepperNavLink").addEventListener("click", () => setLayoutMode("scroll"));
$("cameraPauseButton").addEventListener("click", () => {
  cameraView.autoScrollPaused = !cameraView.autoScrollPaused;
  const button = $("cameraPauseButton");
  button.textContent = cameraView.autoScrollPaused ? "恢复滚动" : "暂停滚动";
  button.setAttribute("aria-pressed", String(cameraView.autoScrollPaused));
  if (!cameraView.autoScrollPaused) renderCameraData();
});
$("cameraClearButton").addEventListener("click", clearCameraDisplay);
$("windowSelect").addEventListener("change", (event) => { windowSeconds = numeric(event.target.value, 10); dirty = true; });
$("layoutScrollButton").addEventListener("click", () => setLayoutMode("scroll"));
$("layoutCockpitButton").addEventListener("click", () => setLayoutMode("cockpit"));
$("openParametersButton").addEventListener("click", openParameterDialog);
$("closeParametersButton").addEventListener("click", () => $("parameterDialog").close());
$("openCalibrationButton").addEventListener("click", async () => {
  if (!$("calibrationDialog").open) $("calibrationDialog").showModal();
  await pollCalibration(true);
  $("calibrationPrimaryButton").focus();
});
async function closeCalibrationDialog() {
  if (calibrationState.active && calibrationState.owner_name === "HOST") {
    if (!window.confirm("当前候选标定尚未保存。关闭向导将取消本次标定并保留旧值，是否继续？")) return;
    await runCalibrationAction("cancel", $("cancelCalibrationButton"));
  }
  $("calibrationDialog").close();
}
$("closeCalibrationButton").addEventListener("click", closeCalibrationDialog);
$("calibrationDialog").addEventListener("cancel", (event) => { event.preventDefault(); closeCalibrationDialog(); });
$("calibrationPrimaryButton").addEventListener("click", () => {
  const action = $("calibrationPrimaryButton").dataset.action;
  if (action) runCalibrationAction(action, $("calibrationPrimaryButton"));
});
$("cancelCalibrationButton").addEventListener("click", () => runCalibrationAction("cancel", $("cancelCalibrationButton")));
$("startRecordingButton").addEventListener("click", () => {
  $("autoRecording").checked = autoRecordingEnabled();
  if (!$("recordingDialog").open) $("recordingDialog").showModal();
  $("recordingName").focus();
});
$("closeRecordingDialogButton").addEventListener("click", () => $("recordingDialog").close());
$("autoRecording").addEventListener("change", (event) => {
  try { window.localStorage.setItem(AUTO_RECORDING_STORAGE_KEY, String(event.target.checked)); } catch (_error) { /* storage may be disabled */ }
});
$("recordingForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    const result = await command("/api/recordings/start", {
      name: $("recordingName").value,
      notes: $("recordingNotes").value,
    }, $("beginManualRecordingButton"));
    $("recordingDialog").close();
    updateRecordingStatus(result);
    showToast("遥测录制已开始");
  } catch (error) { showToast(`开始录制失败：${error.message}`, true); }
});
$("stopRecordingButton").addEventListener("click", async () => {
  if (!window.confirm("停止当前遥测录制并完成索引与摘要写入？")) return;
  try {
    const result = await command("/api/recordings/stop", {}, $("stopRecordingButton"));
    updateRecordingStatus(result);
    showToast("录制已停止，会话文件已同步");
  } catch (error) { showToast(`停止录制失败：${error.message}`, true); }
});
$("addMarkerButton").addEventListener("click", () => {
  $("markerLabel").value = "";
  $("markerDialog").showModal();
  $("markerLabel").focus();
});
$("cancelMarkerButton").addEventListener("click", () => $("markerDialog").close());
$("markerForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    await command("/api/recordings/marker", {label:$("markerLabel").value});
    $("markerDialog").close();
    showToast("时间点标记已写入");
  } catch (error) { showToast(`添加标记失败：${error.message}`, true); }
});
$("recordingLibraryButton").addEventListener("click", () => {
  if (!$("recordingLibraryDialog").open) $("recordingLibraryDialog").showModal();
  loadRecordingLibrary();
});
$("refreshRecordingLibraryButton").addEventListener("click", loadRecordingLibrary);
$("closeRecordingLibraryButton").addEventListener("click", () => $("recordingLibraryDialog").close());
$("playbackPlayButton").addEventListener("click", async () => {
  if (!playback.active || playback.loading) return;
  if (playback.currentMs >= playback.durationMs) await seekPlayback(0);
  playback.playing = !playback.playing;
  playback.lastFrame = 0;
  $("playbackPlayButton").textContent = playback.playing ? "暂停" : "播放";
  dirty = true;
});
$("playbackBackButton").addEventListener("click", () => seekPlayback(playback.currentMs - 5000));
$("playbackForwardButton").addEventListener("click", () => seekPlayback(playback.currentMs + 5000));
$("playbackProgress").addEventListener("input", (event) => {
  $("playbackTime").textContent = `${formatDuration(event.target.value, true)} / ${formatDuration(playback.durationMs, true)}`;
});
$("playbackProgress").addEventListener("change", (event) => seekPlayback(event.target.value));
$("playbackSpeed").addEventListener("change", (event) => { playback.speed = numeric(event.target.value, 1); playback.lastFrame = 0; });
$("exitPlaybackButton").addEventListener("click", exitPlayback);
$("actionModeButton").addEventListener("click", () => setManualMode("action"));
$("manualModeButton").addEventListener("click", () => setManualMode("manual"));
$("manualEnableButton").addEventListener("click", async () => {
  if (manualDrive.enabled) {
    await disableManualDrive("已手动关闭");
    return;
  }
  try {
    const result = await command("/api/control/manual/enable", {}, $("manualEnableButton"));
    manualDrive.controllerId = result.controller_id;
    manualDrive.sequence = 0;
    manualDrive.enabled = true;
    manualDrive.keys.clear();
    renderManualKeys();
    $("manualEnableButton").textContent = "停止并关闭遥控";
    $("manualEnableButton").classList.add("enabled");
    $("manualConnection").textContent = "已启用 · 等待 WASD";
    updateArmButton();
    showToast("键盘遥控已启用，WASD 可直接控制车辆");
  } catch (error) { showToast(`无法启用键盘遥控：${error.message}`, true); }
});
[$("manualMaxSpeed"), $("manualSteering")].forEach((input) => input.addEventListener("input", () => {
  updateManualSettingLabels();
  saveManualSettings();
  if (manualDrive.enabled && (manualComponents().throttle || manualComponents().steering)) sendManualCommand();
}));
window.addEventListener("keydown", (event) => {
  if (!manualInputAllowed(event)) return;
  const key = event.key.toLowerCase();
  event.preventDefault();
  if (manualDrive.keys.has(key)) return;
  manualDrive.keys.add(key);
  renderManualKeys();
  sendManualCommand();
});
window.addEventListener("keyup", (event) => {
  const key = String(event.key).toLowerCase();
  if (!manualDrive.enabled || !["w", "a", "s", "d"].includes(key) || !manualDrive.keys.has(key)) return;
  event.preventDefault();
  manualDrive.keys.delete(key);
  renderManualKeys();
  sendManualCommand();
});
window.addEventListener("blur", () => { if (manualDrive.enabled) disableManualDrive("页面失去焦点"); });
document.addEventListener("visibilitychange", () => {
  if (document.hidden && manualDrive.enabled) disableManualDrive("页面已隐藏");
});
window.addEventListener("pagehide", () => {
  const controllerId = manualDrive.controllerId;
  if (!controllerId || !controlToken) return;
  setManualLocalDisabled("页面已关闭");
  fetch("/api/control/manual/disable", {
    method:"POST", keepalive:true,
    headers:{"Content-Type":"application/json", "X-M6-Control-Token":controlToken},
    body:JSON.stringify({controller_id:controllerId}),
  }).catch(() => {});
});
$("actionKind").addEventListener("change", updateActionFields);
$("lineContinuous").addEventListener("change", () => { updateContinuousOption(); updateArmButton(); });
$("safetyAck").addEventListener("change", updateArmButton);
$("actionForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!event.currentTarget.reportValidity()) return;
  const request = actionRequest();
  if (request.continuous && !window.confirm("持续循迹不会按时间自动停止，关闭网页后小车仍会继续运行。确认进入 Arm 阶段吗？")) return;
  try {
    const result = await command("/api/control/arm", request, $("armButton"));
    syncArmedState(result.armed);
    showToast("动作已 Arm，尚未运动，请核对后 Confirm");
  } catch (error) { showToast(error.message, true); }
});
$("confirmActionButton").addEventListener("click", async () => {
  if (currentArmToken === null) return;
  try {
    const result = await command("/api/control/confirm", {
      arm_token: currentArmToken,
      recording: {
        auto: autoRecordingEnabled(),
        name: $("recordingName").value,
        notes: $("recordingNotes").value,
      },
    }, $("confirmActionButton"));
    currentArmToken = null; $("confirmActionBar").hidden = true; $("safetyAck").checked = false;
    if (result.recording_error) showToast(`动作已开始，但自动录制失败：${result.recording_error}`, true);
    else showToast(result.recording?.recording ? "动作已开始，联动录制已包含前 5 秒" : "动作已开始，急停按钮始终可用");
    if (result.recording) updateRecordingStatus(result.recording);
    await pollControl();
  } catch (error) { showToast(error.message, true); }
});
$("cancelActionButton").addEventListener("click", async () => {
  try {
    await command("/api/control/cancel", {}, $("cancelActionButton"));
    currentArmToken = null; $("confirmActionBar").hidden = true; $("safetyAck").checked = false;
    showToast("当前动作已取消"); await pollControl();
  } catch (error) { showToast(error.message, true); }
});
$("estopButton").addEventListener("click", async () => {
  try {
    await command("/api/control/estop", {}, null);
    currentArmToken = null; $("confirmActionBar").hidden = true; $("safetyAck").checked = false;
    showToast("急停指令已进入高优先级队列"); await pollControl();
  } catch (error) { showToast(`急停请求失败：${error.message}`, true); }
});
$("stepperStopButton").addEventListener("click", () =>
  stepperCommand("/api/stepper/stop", {}, $("stepperStopButton"), "步进停止已在帧边界优先发送"));
$("stepperDisableButton").addEventListener("click", () =>
  stepperCommand("/api/stepper/disable", {}, $("stepperDisableButton"), "步进停止并失能批次已发送"));
$("stepperEnableButton").addEventListener("click", () => {
  if (!window.confirm("将使能地址 1 的 ZDT 步进电机。\n车辆急停和车辆 Safety 不会停止此电机；确认测试区域安全并可使用红色“步进立即停止”按钮吗？")) return;
  stepperCommand("/api/stepper/enable", {confirm:"ENABLE_ZDT_STEPPER"}, $("stepperEnableButton"), "步进使能命令已发送");
});
$("stepperVelocityForm").addEventListener("submit", (event) => {
  event.preventDefault();
  if (!event.currentTarget.reportValidity()) return;
  const body = {
    direction:numeric($("stepperVelocityDirection").value),
    speed_rpm:numeric($("stepperVelocitySpeed").value),
    acceleration:numeric($("stepperVelocityAcceleration").value),
    duration_ms:numeric($("stepperVelocityDuration").value),
    confirm:"START_ZDT_VELOCITY_TEST",
  };
  const direction = body.direction ? "CCW 逆时针" : "CW 顺时针";
  if (!window.confirm(`即将执行速度测试：${direction}，${body.speed_rpm} RPM，加速度 ${body.acceleration}，持续 ${body.duration_ms} ms。\n车辆急停不会停止步进电机；确认直接执行吗？`)) return;
  stepperCommand("/api/stepper/velocity", body, $("stepperVelocityButton"), "定时速度测试已启动");
});
$("stepperPositionForm").addEventListener("submit", (event) => {
  event.preventDefault();
  if (!event.currentTarget.reportValidity()) return;
  const body = {
    direction:numeric($("stepperPositionDirection").value),
    speed_rpm:numeric($("stepperPositionSpeed").value),
    acceleration:numeric($("stepperPositionAcceleration").value),
    pulses:numeric($("stepperPositionPulses").value),
    timeout_ms:numeric($("stepperPositionTimeout").value),
    confirm:"START_ZDT_POSITION_TEST",
  };
  const direction = body.direction ? "CCW 逆时针" : "CW 顺时针";
  if (!window.confirm(`即将执行相对当前位置运动：${direction}，${body.pulses} 脉冲，${body.speed_rpm} RPM，加速度 ${body.acceleration}，超时 ${body.timeout_ms} ms。\n车辆急停不会停止步进电机；确认直接执行吗？`)) return;
  stepperCommand("/api/stepper/position", body, $("stepperPositionButton"), "相对位置测试已启动");
});
$("clearFaultButton").addEventListener("click", async () => {
  const faults = numeric(controlState?.status?.fault_bits);
  if (!faults) return;
  const names = (controlState?.fault_details || []).map((fault) => fault.title).join("、") || hex32(faults);
  if (!window.confirm(`将清除当前锁存故障：${names}。\n清除后不会自动恢复运动，下一次运动仍需重新 Arm / Confirm。确认测试区域安全吗？`)) return;
  try {
    await command("/api/control/clear-fault", {
      confirm: "CLEAR_OBSERVED_FAULTS",
      observed_fault_bits: faults,
    }, $("clearFaultButton"));
    showToast("Safety 故障已清除；车辆保持停止，运动需重新 Arm / Confirm");
    await pollControl();
  } catch (error) { showToast(`清故障失败：${error.message}`, true); }
});
$("loadParametersButton").addEventListener("click", () => loadParameters(false));
$("refreshParametersButton").addEventListener("click", () => loadParameters(true));
$("parameterGroup").addEventListener("change", filterParameters);
$("parameterSearch").addEventListener("input", filterParameters);
$("revertParametersButton").addEventListener("click", () => parameterOperation("/api/parameters/revert", {}, $("revertParametersButton"), "已撤销全部暂存值"));
$("loadFlashButton").addEventListener("click", () => parameterOperation("/api/parameters/load", {}, $("loadFlashButton"), "Flash 参数已载入暂存区，尚未应用"));
$("applyParametersButton").addEventListener("click", () => parameterOperation("/api/parameters/apply", {}, $("applyParametersButton"), "暂存参数已应用到运行控制"));
$("defaultsButton").addEventListener("click", () => {
  if (!window.confirm("只把默认值载入暂存区，不会立即应用。继续吗？")) return;
  parameterOperation("/api/parameters/defaults", {confirm:"LOAD_DEFAULTS_TO_STAGE"}, $("defaultsButton"), "默认值已载入暂存区，尚未应用");
});
$("saveParametersButton").addEventListener("click", () => {
  if (!window.confirm("将当前 Active 参数写入 Flash，并跨重启保留。确认保存吗？")) return;
  parameterOperation("/api/parameters/save", {confirm:"SAVE_ACTIVE_PARAMETERS"}, $("saveParametersButton"), "Active 参数已保存到 Flash");
});
window.addEventListener("resize", () => { dirty = true; });

initializeSensors();
renderCameraData();
updateCameraStatus();
setLayoutMode(storedLayoutMode(), false);
loadManualSettings();
setManualMode("action");
$("autoRecording").checked = autoRecordingEnabled();
setInterval(poll, 50);
setInterval(pollControl, 1000);
setInterval(pollStepper, 250);
setInterval(() => {
  if ($("calibrationDialog").open || calibrationState.active) pollCalibration(true);
}, 250);
setInterval(pollRecordingStatus, 500);
setInterval(updateCountdown, 100);
setInterval(() => {
  const input = manualComponents();
  if (manualDrive.enabled && (input.throttle || input.steering)) sendManualCommand();
}, 100);
setInterval(() => { if (!paused) dirty = true; }, 250);
setInterval(() => {
  if (!playback.active && cameraView.status.last_received_ms != null && lastServerOk)
    updateCameraStatus({ dashboard_now_ms: numeric(cameraView.status.dashboard_now_ms) + 500 });
}, 500);
poll();
pollControl();
pollStepper();
pollCalibration(true);
pollRecordingStatus();
requestAnimationFrame(render);
