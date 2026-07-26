"""Thread-safe, Tk-independent controller for the M6 desktop application.

The controller intentionally talks only to the public ``m6link.Session``
surface (or a duck-compatible fake).  It never imports pyserial and never
allows an arm response to trigger confirmation automatically.
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import IntFlag
import importlib
import inspect
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import threading
import time
from typing import Any, Callable, Iterable, Mapping, MutableMapping, Sequence


# Full-coverage live profile with headroom below the frozen 7000 B/s ceiling:
# CORE 50 Hz + WHEEL 25 Hz + LINE 16.7 Hz + HEALTH 1 Hz = 6105 B/s.
# Names are resolved through m6link.TelemetryStream at connection time so the
# GUI never duplicates wire IDs.
DEFAULT_TELEMETRY_PERIODS = {
    "CORE": 20,
    "WHEEL": 40,
    "LINE": 60,
    "HEALTH": 1000,
}


class GuiInputError(ValueError):
    """A user-editable value is outside the frozen r14 safety contract."""


@dataclass(frozen=True)
class GuiActionSpec:
    """Duck-compatible fallback for :class:`m6link.ActionSpec`."""

    kind: Any
    wheel_mask: int = 0
    target_l_mm_s: int = 0
    target_r_mm_s: int = 0
    angle_mdeg: int = 0
    yaw_rate_mdeg_s: int = 0
    duration_ms: int = 0
    expected_active_revision: int = 0


@dataclass(frozen=True)
class ActionDraft:
    label: str
    kind: str
    wheel_mask: int
    target_l_mm_s: int
    target_r_mm_s: int
    angle_mdeg: int
    yaw_rate_mdeg_s: int
    duration_ms: int
    required_capabilities: frozenset[str]


@dataclass
class ArmedLease:
    token: int
    expires_at: float
    label: str
    draft: ActionDraft


@dataclass(frozen=True)
class ParameterRow:
    key: str
    active: Any
    staged: Any
    persisted: Any
    minimum: Any = None
    maximum: Any = None
    unit: str = ""
    writable: bool = True


@dataclass(frozen=True)
class TelemetryPoint:
    time_s: float
    left_mm_s: float = math.nan
    right_mm_s: float = math.nan
    yaw_deg: float = math.nan
    yaw_rate_dps: float = math.nan
    grayscale: tuple[float, ...] = ()
    centroid: float = math.nan


@dataclass
class UiState:
    phase: str = "disconnected"
    port: str = ""
    legacy: bool = False
    protocol_version: int = 0
    status_text: str = "未连接"
    last_error: str = ""
    identity: Any = None
    capabilities: frozenset[str] = frozenset()
    diagnostics: dict[str, Any] = field(default_factory=dict)
    parameters: dict[str, ParameterRow] = field(default_factory=dict)
    telemetry: deque[TelemetryPoint] = field(default_factory=lambda: deque(maxlen=750))
    armed: ArmedLease | None = None
    action_status: Any = None
    recording: bool = False
    recording_path: str = ""
    replay_result: Any = None
    score_result: Any = None
    generation: int = 0

    @property
    def connected(self) -> bool:
        return self.phase == "connected"


def object_mapping(value: Any) -> dict[str, Any]:
    """Return public fields from a model, mapping, namedtuple, or object."""

    if value is None:
        return {}
    if isinstance(value, Mapping):
        return {str(key): item for key, item in value.items()}
    if is_dataclass(value):
        return asdict(value)
    as_dict = getattr(value, "_asdict", None)
    if callable(as_dict):
        return dict(as_dict())
    if hasattr(value, "__dict__"):
        return {
            str(key): item
            for key, item in vars(value).items()
            if not str(key).startswith("_")
        }
    slots = getattr(type(value), "__slots__", ())
    if isinstance(slots, str):
        slots = (slots,)
    return {
        str(key): getattr(value, key)
        for key in slots
        if not str(key).startswith("_") and hasattr(value, key)
    }


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name) if name is not None else str(value)


def normalize_capabilities(value: Any) -> frozenset[str]:
    """Normalize an IntFlag, bit mask, sequence, or mapping to registry names."""

    if value is None:
        return frozenset()
    if isinstance(value, Mapping):
        return frozenset(
            str(name).upper() for name, enabled in value.items() if bool(enabled)
        )
    if isinstance(value, str):
        pieces = value.replace("|", ",").replace(" ", ",").split(",")
        return frozenset(piece.strip().upper() for piece in pieces if piece.strip())
    if isinstance(value, IntFlag):
        names = {
            member.name.upper()
            for member in type(value)
            if member.value and (value & member) == member
        }
        if names:
            return frozenset(names)
        value = int(value)
    if isinstance(value, int):
        # Registry numbers are owned by m6link.  A bare integer is decoded only
        # through its generated Capability enum so this GUI cannot become a
        # third handwritten protocol registry.
        enum_type: Any = None
        try:
            module = importlib.import_module("m6link")
            enum_type = getattr(module, "Capability", None)
            if enum_type is None:
                registry = importlib.import_module("m6link.registry")
                enum_type = getattr(registry, "Capability", None)
        except ImportError:
            enum_type = None
        if enum_type is None:
            return frozenset()
        decoded = enum_type(value)
        return frozenset(
            member.name.upper()
            for member in enum_type
            if member.value and (decoded & member) == member
        )
    try:
        return frozenset(_enum_name(item).upper() for item in value)
    except TypeError:
        return frozenset({_enum_name(value).upper()})


def protocol_version(identity: Any, session: Any = None) -> int:
    fields = object_mapping(identity)
    for name in ("protocol_version", "wire_version", "wire_revision", "wire"):
        raw = fields.get(name)
        if raw is not None:
            try:
                return int(raw)
            except (TypeError, ValueError):
                pass
    mode = fields.get("mode", getattr(session, "mode", ""))
    text = _enum_name(mode).upper()
    return 2 if "V2" in text or "ESTABLISHED" in text else 1 if "V1" in text else 0


def capability_source(identity: Any, session: Any) -> Any:
    fields = object_mapping(identity)
    for name in ("enabled_capabilities", "capabilities", "supported_capabilities"):
        if name in fields:
            return fields[name]
    return getattr(session, "capabilities", None)


def is_emergency_key(keysym: str, char: str = "") -> bool:
    """Pure predicate shared by the Tk binding and headless tests."""

    return keysym in {"space", "S", "s"} or char in {" ", "S", "s"}


def feature_visibility(
    capabilities: Iterable[str], *, connected: bool, protocol: int
) -> dict[str, bool]:
    """Capability-driven visibility policy, kept pure for headless tests."""

    caps = {str(name).upper() for name in capabilities}
    v2 = connected and protocol == 2
    wheel = v2 and {"ACTION_TWO_PHASE", "WHEEL_DIAG_ACTION"}.issubset(caps)
    return {
        "parameter_tab": v2 and "PARAM_READ" in caps,
        "parameter_stage": v2 and "PARAM_STAGE" in caps,
        "parameter_apply": v2 and "PARAM_BOUNDARY_APPLY" in caps,
        "parameter_persist": v2 and "PARAM_PERSIST" in caps,
        "wheel_actions": wheel,
        "action_tab": wheel or (v2 and "HEADING_CALIBRATION" in caps),
        "wizard_tab": wheel,
        "heading": wheel and "HEADING_ACTION" in caps,
        "configurable_turn": wheel
        and {"HEADING_ACTION", "CONFIGURABLE_TURN"}.issubset(caps),
        "heading_calibration": v2 and "HEADING_CALIBRATION" in caps,
        "recording": v2 and "TELEMETRY_SUBSCRIBE" in caps,
    }


def _int_speed(value: Any, *, allow_zero: bool = False) -> int:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise GuiInputError("轮速必须是数字") from exc
    if not math.isfinite(numeric) or abs(numeric) > 400:
        raise GuiInputError("轮速必须位于 -400..400 mm/s")
    rounded = int(round(numeric))
    if rounded == 0 and allow_zero:
        return 0
    if abs(rounded) < 50:
        raise GuiInputError("非零动作轮速绝对值必须为 50..400 mm/s")
    return rounded


def _duration(value: Any, *, low: int = 100, high: int = 10_000) -> int:
    try:
        duration = int(round(float(value) * 1000.0))
    except (TypeError, ValueError) as exc:
        raise GuiInputError("持续时间必须是秒数") from exc
    if duration < low or duration > high:
        raise GuiInputError(f"持续时间必须位于 {low / 1000:g}..{high / 1000:g} 秒")
    return duration


def wheel_action(left: Any, right: Any, duration_s: Any, *, label: str = "独立轮速") -> ActionDraft:
    left_i = _int_speed(left, allow_zero=True)
    right_i = _int_speed(right, allow_zero=True)
    mask = (1 if left_i else 0) | (2 if right_i else 0)
    if mask == 0:
        raise GuiInputError("至少选择一个非零轮速；正常停止请使用 Cancel")
    return ActionDraft(
        label=label,
        kind="WHEEL_SPEED_TEST",
        wheel_mask=mask,
        target_l_mm_s=left_i,
        target_r_mm_s=right_i,
        angle_mdeg=0,
        yaw_rate_mdeg_s=0,
        duration_ms=_duration(duration_s),
        required_capabilities=frozenset({"ACTION_TWO_PHASE", "WHEEL_DIAG_ACTION"}),
    )


def straight_action(speed: Any, duration_s: Any, *, reverse: bool = False) -> ActionDraft:
    speed_i = abs(_int_speed(speed)) * (-1 if reverse else 1)
    return wheel_action(
        speed_i,
        speed_i,
        duration_s,
        label="后退" if reverse else "前进",
    )


def arc_action(
    speed: Any,
    duration_s: Any = 1.5,
    *,
    ratio: Any = 0.7,
    left: bool,
) -> ActionDraft:
    outer = _int_speed(speed)
    try:
        ratio_f = float(ratio)
    except (TypeError, ValueError) as exc:
        raise GuiInputError("弧线内外轮比例必须是数字") from exc
    if not 0.2 <= ratio_f <= 1.0:
        raise GuiInputError("弧线内外轮比例必须位于 0.2..1.0")
    inner = _int_speed(outer * ratio_f)
    left_speed, right_speed = (inner, outer) if left else (outer, inner)
    return wheel_action(
        left_speed,
        right_speed,
        duration_s,
        label="左平滑弧线" if left else "右平滑弧线",
    )


def heading_hold_action(speed: Any, duration_s: Any) -> ActionDraft:
    speed_i = _int_speed(speed)
    return ActionDraft(
        label="航向保持直行",
        kind="HEADING_HOLD",
        wheel_mask=3,
        target_l_mm_s=speed_i,
        target_r_mm_s=speed_i,
        angle_mdeg=0,
        yaw_rate_mdeg_s=0,
        duration_ms=_duration(duration_s, high=30_000),
        required_capabilities=frozenset({"ACTION_TWO_PHASE", "HEADING_ACTION"}),
    )


def turn_action(angle_deg: Any, duration_s: Any) -> ActionDraft:
    try:
        angle = float(angle_deg)
    except (TypeError, ValueError) as exc:
        raise GuiInputError("相对转角必须是数字") from exc
    if not math.isfinite(angle) or abs(angle) < 1.0 or abs(angle) > 360.0:
        raise GuiInputError("相对转角绝对值必须位于 1..360°")
    return ActionDraft(
        label=f"相对转向 {angle:g}°",
        kind="HEADING_TURN",
        wheel_mask=3,
        target_l_mm_s=0,
        target_r_mm_s=0,
        angle_mdeg=int(round(angle * 1000.0)),
        yaw_rate_mdeg_s=0,
        duration_ms=_duration(duration_s, low=500, high=30_000),
        required_capabilities=frozenset(
            {"ACTION_TWO_PHASE", "HEADING_ACTION", "CONFIGURABLE_TURN"}
        ),
    )


def wizard_drafts(speed: Any = 160, duration_s: Any = 1.5, ratio: Any = 0.7) -> tuple[ActionDraft, ...]:
    """Build the bounded acceptance sequence; every item still needs a new arm."""

    speed_i = abs(_int_speed(speed))
    return (
        wheel_action(speed_i, 0, duration_s, label="左轮单轮"),
        wheel_action(0, speed_i, duration_s, label="右轮单轮"),
        straight_action(speed_i, duration_s),
        arc_action(speed_i, duration_s, ratio=ratio, left=True),
        arc_action(speed_i, duration_s, ratio=ratio, left=False),
        heading_hold_action(speed_i, duration_s),
        turn_action(45, max(float(duration_s), 0.5)),
        turn_action(-45, max(float(duration_s), 0.5)),
    )


class Controller:
    """Own asynchronous GUI operations without owning the serial device itself."""

    def __init__(
        self,
        session_factory: Callable[..., Any],
        *,
        clock: Callable[[], float] = time.monotonic,
        telemetry_limit: int = 750,
    ) -> None:
        self.session_factory = session_factory
        self.clock = clock
        self.state = UiState()
        self.state.telemetry = deque(maxlen=max(50, int(telemetry_limit)))
        self._session: Any = None
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="m6gui-rpc")
        self._estop_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="m6gui-estop")
        self._completions: SimpleQueue[tuple[str, Future[Any]]] = SimpleQueue()
        self._pending: set[str] = set()
        self._pending_lock = threading.Lock()
        self._last_diag_refresh = -math.inf
        self._closed = False

    @property
    def session(self) -> Any:
        return self._session

    def has_capability(self, *names: str) -> bool:
        available = self.state.capabilities
        return all(str(name).upper() in available for name in names)

    def discover_ports(self) -> list[str]:
        """Discover ports through m6link/factory only; never touch pyserial here."""

        target = self.session_factory
        method = getattr(target, "discover_ports", None)
        if not callable(method):
            try:
                module = importlib.import_module(target.__module__.split(".")[0])
                method = getattr(module, "discover_ports", None)
            except (AttributeError, ImportError):
                method = None
        if not callable(method):
            return []
        ports = method()
        result: list[str] = []
        for port in ports or ():
            if isinstance(port, str):
                result.append(port)
            else:
                result.append(str(getattr(port, "device", port)))
        return result

    def _submit(
        self,
        operation: str,
        fn: Callable[[], Any],
        *,
        priority: bool = False,
        replace: bool = False,
    ) -> bool:
        if self._closed:
            return False
        with self._pending_lock:
            if operation in self._pending and not replace:
                return False
            self._pending.add(operation)
        pool = self._estop_worker if priority else self._worker
        future = pool.submit(fn)
        future.add_done_callback(lambda done: self._completions.put((operation, done)))
        return True

    def connect(self, port: str, *, legacy: bool = False) -> bool:
        if self.state.phase in {"connecting", "connected"}:
            return False
        self.state.phase = "connecting"
        self.state.port = str(port)
        self.state.legacy = bool(legacy)
        self.state.status_text = "正在连接（显式 V1）" if legacy else "正在连接并要求 V2"
        self.state.last_error = ""

        def work() -> tuple[Any, Any, dict[str, Any], dict[str, ParameterRow]]:
            try:
                session = self.session_factory(port=port, legacy=legacy)
            except TypeError:
                try:
                    session = self.session_factory(port, legacy=legacy)
                except TypeError:
                    session = self.session_factory()
            connect = getattr(session, "connect")
            try:
                identity = connect(port, legacy=legacy)
            except TypeError:
                try:
                    identity = connect(port)
                except TypeError:
                    identity = connect()
            if identity is None:
                identity = getattr(session, "identity", None)
            version = protocol_version(identity, session)
            if not legacy and version != 2:
                getattr(session, "close", lambda: None)()
                raise RuntimeError("GUI 默认要求 V2；设备未建立 V2，请仅在需要时显式选择 V1 legacy")
            if version == 2:
                diagnostics = self._query_diagnostics(session)
                parameters = self._query_parameters(session)
                self._subscribe(session)
            else:
                diagnostics = {}
                parameters = {}
            return session, identity, diagnostics, parameters

        return self._submit("connect", work)

    def disconnect(self) -> bool:
        session = self._session
        if session is None:
            self._set_disconnected()
            return True
        self.state.status_text = "正在断开"
        return self._submit("disconnect", lambda: getattr(session, "close")())

    def tick(self) -> None:
        """Schedule non-blocking event/diagnostic work from a Tk ``after`` loop."""

        if self.state.armed is not None and self.clock() >= self.state.armed.expires_at:
            self.state.status_text = f"Arm 已过期：{self.state.armed.label}，必须重新 Arm"
            self.state.armed = None
            self.state.generation += 1
        if (
            not self.state.connected
            or self.state.protocol_version != 2
            or self._session is None
        ):
            return
        include_diagnostics = self.clock() - self._last_diag_refresh >= 1.0
        session = self._session
        self._submit(
            "refresh",
            lambda: self._refresh_worker(session, include_diagnostics),
        )

    def poll(self) -> bool:
        changed = False
        while True:
            try:
                operation, future = self._completions.get_nowait()
            except Empty:
                break
            with self._pending_lock:
                self._pending.discard(operation)
            changed = True
            try:
                result = future.result()
            except Exception as exc:  # surface errors to the operator, never Tk stderr
                self._apply_error(operation, exc)
            else:
                self._apply_result(operation, result)
        return changed

    def _apply_result(self, operation: str, result: Any) -> None:
        if operation == "connect":
            session, identity, diagnostics, parameters = result
            self._session = session
            self.state.identity = identity
            self.state.protocol_version = protocol_version(identity, session)
            self.state.capabilities = normalize_capabilities(capability_source(identity, session))
            self.state.diagnostics = diagnostics
            self.state.parameters = parameters
            self.state.phase = "connected"
            self.state.status_text = (
                "已连接：V1 legacy" if self.state.legacy else "已连接：V2 established"
            )
        elif operation == "disconnect":
            self._set_disconnected()
        elif operation == "refresh":
            diagnostics, samples, events = result
            if diagnostics:
                self.state.diagnostics.update(diagnostics)
                self._last_diag_refresh = self.clock()
            for sample in samples:
                point = telemetry_point(sample)
                if point is not None:
                    self.state.telemetry.append(point)
            if events:
                self.state.diagnostics["events"] = events[-20:]
        elif operation == "parameters":
            self.state.parameters = result
            self.state.status_text = "参数已刷新"
        elif operation in {"stage", "apply", "revert", "load", "defaults", "save"}:
            self.state.status_text = {
                "stage": "参数已暂存",
                "apply": "参数已应用",
                "revert": "staged 已回退",
                "load": "持久参数已加载",
                "defaults": "默认参数已载入 staged",
                "save": "参数已安全保存",
            }[operation]
            self.refresh_parameters()
        elif operation == "arm":
            draft, armed = result
            token, expires_at = self._normalize_armed(armed)
            self.state.armed = ArmedLease(token, expires_at, draft.label, draft)
            self.state.status_text = f"已 Arm：{draft.label}；必须人工 Confirm"
        elif operation == "confirm":
            self.state.action_status = result
            self.state.armed = None
            self.state.status_text = "Confirm 已人工发送；动作状态以设备回复为准"
        elif operation == "cancel":
            self.state.action_status = result
            self.state.armed = None
            self.state.status_text = "已请求正常停止（Cancel）"
        elif operation == "heading_calibrate":
            self.state.diagnostics["heading_calibration"] = result
            self.state.status_text = "航向校准完成；请核对设备状态后再 Arm 动作"
        elif operation == "estop":
            self.state.armed = None
            self.state.action_status = result
            self.state.status_text = "急停已发送；故障保持锁存"
        elif operation == "record_start":
            self.state.recording = True
            self.state.status_text = f"正在录制：{self.state.recording_path}"
        elif operation == "record_stop":
            self.state.recording = False
            self.state.status_text = "录制已停止"
        elif operation == "replay":
            summary, samples = normalize_replay(result)
            self.state.replay_result = summary
            if samples:
                self.state.telemetry.clear()
                limit = self.state.telemetry.maxlen or 750
                for sample in samples[-limit:]:
                    point = telemetry_point(sample)
                    if point is not None:
                        self.state.telemetry.append(point)
            self.state.status_text = "记录已加载回放"
        elif operation == "score":
            self.state.score_result = result
            self.state.status_text = "评分完成"
        self.state.last_error = ""
        self.state.generation += 1

    def _apply_error(self, operation: str, exc: Exception) -> None:
        if operation == "connect":
            self._session = None
            self.state.phase = "error"
        elif operation == "record_start":
            self.state.recording = False
        self.state.last_error = f"{operation}: {exc}"
        self.state.status_text = f"操作失败：{exc}"
        self.state.generation += 1

    def _set_disconnected(self) -> None:
        self._session = None
        self.state.phase = "disconnected"
        self.state.protocol_version = 0
        self.state.capabilities = frozenset()
        self.state.armed = None
        self.state.recording = False
        self.state.status_text = "未连接"
        self.state.generation += 1

    @staticmethod
    def _call(session: Any, names: Sequence[str], *args: Any, **kwargs: Any) -> Any:
        for name in names:
            method = getattr(session, name, None)
            if callable(method):
                try:
                    return method(*args, **kwargs)
                except TypeError:
                    if kwargs:
                        return method(*args)
                    raise
        raise AttributeError(f"Session 缺少接口：{'/'.join(names)}")

    @classmethod
    def _optional_call(cls, session: Any, names: Sequence[str]) -> Any:
        try:
            return cls._call(session, names)
        except AttributeError:
            return None

    @classmethod
    def _query_diagnostics(cls, session: Any) -> dict[str, Any]:
        queries = {
            "status": ("get_status", "status_get"),
            "reset": ("get_reset_info", "reset_get"),
            "tasks": ("get_task_stats", "task_stats_get"),
            "uart": ("get_uart_stats", "uart_stats_get"),
            "link": ("get_link_stats", "link_stats_get"),
        }
        return {
            label: value
            for label, names in queries.items()
            if (value := cls._optional_call(session, names)) is not None
        }

    @classmethod
    def _query_parameters(cls, session: Any) -> dict[str, ParameterRow]:
        result = cls._optional_call(session, ("get_parameters", "get"))
        if result is None:
            return {}
        if not isinstance(result, Mapping):
            result = object_mapping(result)
        snapshot_values = result.get("values")
        if isinstance(snapshot_values, Mapping):
            result = snapshot_values
        rows: dict[str, ParameterRow] = {}
        for raw_key, raw_value in result.items():
            fields = object_mapping(raw_value)
            key = str(fields.get("key", raw_key))
            descriptor = object_mapping(fields.get("descriptor"))
            active = fields.get("active", fields.get("active_value", raw_value if not fields else None))
            staged = fields.get("staged", fields.get("staged_value", active))
            persisted = fields.get("persisted", fields.get("persisted_value"))
            rows[key] = ParameterRow(
                key=key,
                active=active,
                staged=staged,
                persisted=persisted,
                minimum=fields.get("minimum", descriptor.get("minimum")),
                maximum=fields.get("maximum", descriptor.get("maximum")),
                unit=str(fields.get("unit", descriptor.get("unit", ""))),
                writable=bool(fields.get("writable", descriptor.get("writable", True))),
            )
        return rows

    @classmethod
    def _subscribe(cls, session: Any) -> None:
        method = getattr(session, "subscribe", None)
        if callable(method):
            try:
                module = importlib.import_module(session.__class__.__module__.split(".")[0])
                stream_type = getattr(module, "TelemetryStream")
                method({
                    getattr(stream_type, name): period
                    for name, period in DEFAULT_TELEMETRY_PERIODS.items()
                })
            except (AttributeError, ImportError):
                # Duck-compatible fake implementations may accept symbolic
                # names; no numeric stream IDs are duplicated here.
                method(dict(DEFAULT_TELEMETRY_PERIODS))
            return
        method = getattr(session, "subscribe_telemetry", None)
        if callable(method):
            method(rate_hz=50)

    @classmethod
    def _read_events(cls, session: Any) -> tuple[list[Any], list[Any]]:
        samples: list[Any] = []
        events: list[Any] = []
        read = getattr(session, "read_event", None)
        if callable(read):
            for _ in range(64):
                try:
                    item = read(timeout=0.0)
                except Empty:
                    break
                if item is None:
                    break
                if is_telemetry(item):
                    samples.append(unwrap_telemetry(item))
                else:
                    events.append(item)
        else:
            read_many = getattr(session, "read_events", None)
            if callable(read_many):
                for item in read_many(timeout=0.0, limit=64) or ():
                    (samples if is_telemetry(item) else events).append(
                        unwrap_telemetry(item) if is_telemetry(item) else item
                    )
        return samples, events

    @classmethod
    def _refresh_worker(cls, session: Any, include_diagnostics: bool) -> tuple[dict[str, Any], list[Any], list[Any]]:
        diagnostics = cls._query_diagnostics(session) if include_diagnostics else {}
        samples, events = cls._read_events(session)
        return diagnostics, samples, events

    def refresh_parameters(self) -> bool:
        if self._session is None:
            return False
        session = self._session
        return self._submit("parameters", lambda: self._query_parameters(session))

    def stage_parameter(self, key: str, value: Any) -> bool:
        if self._session is None:
            return False
        session = self._session
        return self._submit(
            "stage",
            lambda: self._call(session, ("stage_parameter", "stage"), key, value),
        )

    def _parameter_operation(self, operation: str, names: Sequence[str]) -> bool:
        if self._session is None:
            return False
        session = self._session
        return self._submit(operation, lambda: self._call(session, names))

    def apply_parameters(self) -> bool:
        return self._parameter_operation("apply", ("apply_parameters", "apply"))

    def revert_parameters(self) -> bool:
        return self._parameter_operation("revert", ("revert_parameters", "revert"))

    def load_parameters(self) -> bool:
        return self._parameter_operation("load", ("load_parameters", "load"))

    def default_parameters(self) -> bool:
        return self._parameter_operation("defaults", ("default_parameters", "defaults"))

    def save_parameters(self) -> bool:
        return self._parameter_operation("save", ("save_parameters", "save"))

    def arm_action(self, draft: ActionDraft) -> bool:
        if self._session is None:
            return False
        missing = draft.required_capabilities - self.state.capabilities
        if missing:
            raise GuiInputError("设备未发布能力：" + ", ".join(sorted(missing)))
        if self.state.armed is not None and self.clock() < self.state.armed.expires_at:
            raise GuiInputError("已有待 Confirm 的 Arm；请先 Confirm、Cancel 或等待过期")
        session = self._session

        def work() -> tuple[ActionDraft, Any]:
            spec = self._make_action_spec(session, draft)
            armed = self._call(session, ("arm_action",), spec)
            return draft, armed

        return self._submit("arm", work)

    @staticmethod
    def _make_action_spec(session: Any, draft: ActionDraft) -> Any:
        spec_type: Any = GuiActionSpec
        kind: Any = draft.kind
        try:
            module = importlib.import_module(session.__class__.__module__.split(".")[0])
            spec_type = getattr(module, "ActionSpec", GuiActionSpec)
            kind_type = getattr(module, "ActionKind", None)
            if kind_type is not None:
                kind = getattr(kind_type, draft.kind, draft.kind)
        except ImportError:
            pass
        fields = {
            "kind": kind,
            "wheel_mask": draft.wheel_mask,
            "target_l_mm_s": draft.target_l_mm_s,
            "target_r_mm_s": draft.target_r_mm_s,
            "angle_mdeg": draft.angle_mdeg,
            "yaw_rate_mdeg_s": draft.yaw_rate_mdeg_s,
            "duration_ms": draft.duration_ms,
        }
        identity = object_mapping(getattr(session, "identity", None))
        status = object_mapping(getattr(session, "status", None))
        fields["expected_active_revision"] = int(
            status.get("active_revision", identity.get("active_revision", 0)) or 0
        )
        try:
            signature = inspect.signature(spec_type)
            accepted = {name for name in signature.parameters if name != "self"}
            fields = {name: value for name, value in fields.items() if name in accepted}
        except (TypeError, ValueError):
            pass
        return spec_type(**fields)

    def _normalize_armed(self, armed: Any) -> tuple[int, float]:
        fields = object_mapping(armed)
        token = int(fields.get("token", armed if isinstance(armed, int) else 0))
        if token == 0:
            raise RuntimeError("设备返回了无效的零 Arm token")
        now = self.clock()
        expires_at = fields.get("expires_at")
        if isinstance(expires_at, (int, float)) and now < float(expires_at) <= now + 30.0:
            return token, float(expires_at)
        expiry_ms = fields.get("expiry_ms", fields.get("expires_in_ms", 10_000))
        try:
            seconds = float(expiry_ms) / 1000.0
        except (TypeError, ValueError):
            seconds = 10.0
        if not 0.0 < seconds <= 30.0:
            seconds = 10.0
        return token, now + seconds

    def confirm_action(self) -> bool:
        lease = self.state.armed
        if self._session is None or lease is None:
            return False
        if self.clock() >= lease.expires_at:
            self.state.armed = None
            raise GuiInputError("Arm 已过期；必须重新 Arm")
        session = self._session
        token = lease.token
        return self._submit(
            "confirm", lambda: self._call(session, ("confirm_action",), token)
        )

    def cancel_action(self) -> bool:
        if self._session is None:
            return False
        session = self._session
        return self._submit(
            "cancel", lambda: self._call(session, ("cancel_action",), 0)
        )

    def calibrate_heading(self) -> bool:
        """Run the independent, non-motor heading calibration RPC."""

        if self._session is None:
            return False
        if not self.has_capability("HEADING_CALIBRATION"):
            raise GuiInputError("设备未发布能力：HEADING_CALIBRATION")
        if self.state.armed is not None and self.clock() < self.state.armed.expires_at:
            raise GuiInputError("存在待 Confirm 的 Arm；请先 Cancel 或等待过期再校准")
        session = self._session
        return self._submit(
            "heading_calibrate",
            lambda: self._call(
                session,
                ("calibrate_heading", "heading_calibrate", "heading_calibration"),
            ),
        )

    def emergency_stop(self) -> bool:
        if self._session is None:
            return False
        self.state.status_text = "正在发送高优先级急停"
        session = self._session
        return self._submit(
            "estop",
            lambda: self._call(session, ("emergency_stop",)),
            priority=True,
        )

    def start_recording(self, path: str | Path) -> bool:
        if self._session is None:
            return False
        self.state.recording_path = str(path)
        session = self._session
        return self._submit(
            "record_start",
            lambda: self._call(session, ("start_recording",), str(path)),
        )

    def stop_recording(self) -> bool:
        if self._session is None:
            return False
        session = self._session
        return self._submit(
            "record_stop", lambda: self._call(session, ("stop_recording",))
        )

    def replay(self, path: str | Path) -> bool:
        path_text = str(path)

        def work() -> Any:
            method = getattr(self._session, "replay", None)
            if callable(method):
                return method(path_text)
            module = self._m6link_module()
            replay_type = getattr(module, "Replay", None)
            if replay_type is not None:
                return replay_type(Path(path_text))
            return getattr(module, "load_recording")(Path(path_text))

        return self._submit("replay", work)

    def score(self, path: str | Path) -> bool:
        path_text = str(path)

        def work() -> Any:
            method = getattr(self._session, "score", None)
            if callable(method):
                return method(path_text)
            module = self._m6link_module()
            replay_type = getattr(module, "Replay", None)
            if replay_type is not None:
                return replay_type(Path(path_text)).score()
            loaded = getattr(module, "load_recording")(Path(path_text))
            samples = loaded[0] if isinstance(loaded, tuple) else loaded
            return getattr(module, "score_samples")(samples)

        return self._submit("score", work)

    def _m6link_module(self) -> Any:
        if self._session is not None:
            name = self._session.__class__.__module__.split(".")[0]
            return importlib.import_module(name)
        return importlib.import_module("m6link")

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        session, self._session = self._session, None
        if session is not None:
            try:
                getattr(session, "close", lambda: None)()
            except Exception:
                pass
        self._worker.shutdown(wait=False, cancel_futures=True)
        self._estop_worker.shutdown(wait=False, cancel_futures=True)


def is_telemetry(item: Any) -> bool:
    fields = object_mapping(item)
    event_type = fields.get("type", fields.get("event_type", fields.get("kind", "")))
    if "TELEMETRY" in _enum_name(event_type).upper():
        return True
    name = type(item).__name__.upper()
    if "TELEMETRY" in name:
        return True
    body = fields.get("sample", fields.get("data"))
    if body is not None and "TELEMETRY" in _enum_name(event_type).upper():
        return True
    telemetry_keys = {
        "speed_l_mm_s",
        "filtered_l_mm_s",
        "yaw_cdeg",
        "adc",
        "grayscale",
        "gray_values",
    }
    return bool(telemetry_keys.intersection(fields))


def unwrap_telemetry(item: Any) -> Any:
    fields = object_mapping(item)
    return fields.get("sample", fields.get("data", item))


def _first(fields: Mapping[str, Any], *names: str, default: Any = math.nan) -> Any:
    for name in names:
        if name in fields and fields[name] is not None:
            return fields[name]
    return default


def _finite_float(value: Any, scale: float = 1.0) -> float:
    try:
        number = float(value) / scale
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def telemetry_point(sample: Any) -> TelemetryPoint | None:
    fields = object_mapping(sample)
    if not fields:
        return None
    nested_values = fields.get("values")
    if isinstance(nested_values, Mapping):
        fields = {**fields, **nested_values}
    raw_time = _first(fields, "time_s", "timestamp_s", "time_ms", default=time.monotonic())
    time_scale = 1000.0 if "time_ms" in fields and "time_s" not in fields else 1.0
    left = _first(
        fields,
        "left_mm_s",
        "speed_l_mm_s",
        "filtered_l_mm_s",
        "filter_l",
        "wheel_left_mm_s",
    )
    right = _first(
        fields,
        "right_mm_s",
        "speed_r_mm_s",
        "filtered_r_mm_s",
        "filter_r",
        "wheel_right_mm_s",
    )
    if "yaw_deg" in fields:
        yaw = _finite_float(fields["yaw_deg"])
    else:
        yaw = _finite_float(_first(fields, "yaw_cdeg", "yaw_x100"), 100.0)
    if "yaw_rate_dps" in fields:
        rate = _finite_float(fields["yaw_rate_dps"])
    else:
        rate = _finite_float(_first(fields, "yaw_rate_cdeg_s", "rate_x100"), 100.0)
    gray = _first(fields, "grayscale", "gray_values", "adc", default=())
    if isinstance(gray, (str, bytes)):
        gray = ()
    try:
        gray_tuple = tuple(_finite_float(value) for value in gray)[:8]
    except TypeError:
        gray_tuple = tuple(
            _finite_float(fields.get(f"gray_{index}", math.nan)) for index in range(8)
        )
    if "centroid" in fields:
        centroid = _finite_float(fields["centroid"])
    elif "position_x1000" in fields:
        centroid = _finite_float(fields["position_x1000"], 1000.0)
    else:
        centroid = _finite_float(_first(fields, "pos_filter_x1000"), 1000.0)
    return TelemetryPoint(
        time_s=_finite_float(raw_time, time_scale),
        left_mm_s=_finite_float(left),
        right_mm_s=_finite_float(right),
        yaw_deg=yaw,
        yaw_rate_dps=rate,
        grayscale=gray_tuple,
        centroid=centroid,
    )


def model_text(value: Any, *, max_fields: int = 18) -> str:
    """Compact diagnostic rendering used by the view and replay panel."""

    fields = object_mapping(value)
    if not fields:
        return "—" if value is None else str(value)
    pairs: list[str] = []
    for index, (name, item) in enumerate(fields.items()):
        if index >= max_fields:
            pairs.append("…")
            break
        if isinstance(item, (Mapping, list, tuple)):
            item = str(item)
        elif hasattr(item, "name"):
            item = item.name
        pairs.append(f"{name}={item}")
    return "  ".join(pairs)


def normalize_replay(result: Any) -> tuple[dict[str, Any], list[Any]]:
    """Normalize fake, modern Replay, and legacy load_recording results."""

    if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], list):
        samples, metadata = result
        summary = {
            "sample_count": len(samples),
            "metadata": metadata,
        }
        return summary, samples
    fields = object_mapping(result)
    raw_samples = fields.pop("samples", None)
    if raw_samples is None and isinstance(result, Mapping):
        raw_samples = result.get("samples")
    samples = list(raw_samples) if isinstance(raw_samples, (list, tuple)) else []
    if samples:
        fields["sample_count"] = len(samples)
    if not fields:
        fields = {"result": str(result), "sample_count": len(samples)}
    return fields, samples
