"""Safety-oriented command facade for the local M6 dashboard."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import secrets
import threading
import time
from typing import Any, Callable, Mapping

from .errors import DeviceRejected, Disconnected, M6Error
from .faults import decode_safety_faults
from .models import ActionSpec, ArmedAction, HMissionConfig
from .registry import (
    ActionKind,
    Capability,
    GrayscaleCalibrationOwner,
    GrayscaleCalibrationState,
    REGISTRY,
    RpcResult,
)
from .stepper_registry import CAPABILITY_MASK as STEPPER_CAPABILITY_MASK
from .stepper_registry import StepperNotReady, StepperRefresh
from .h_mission_registry import CAPABILITY_MASK as H_MISSION_CAPABILITY_MASK


_DASHBOARD_ACTIONS = {
    ActionKind.WHEEL_SPEED_TEST,
    ActionKind.HEADING_HOLD,
    ActionKind.HEADING_TURN,
    ActionKind.HEADING_RATE,
    ActionKind.LINE_FOLLOW,
}
_MAX_DASHBOARD_ACTION_MS = 10_000
_STEPPER_MOTION_PREFLIGHT_S = 0.050


def _stepper_not_ready_message(detail: int) -> str:
    labels = (
        (StepperNotReady.NO_MOTOR_STATUS, "尚未收到电机状态"),
        (StepperNotReady.MOTOR_STATUS_STALE, "电机状态超过 500 ms 未更新"),
        (StepperNotReady.MOTOR_DISABLED, "电机未使能"),
        (StepperNotReady.STALLED, "检测到堵转"),
        (StepperNotReady.STALL_PROTECTION, "堵转保护已触发"),
        (StepperNotReady.LEFT_LIMIT, "左限位输入有效"),
        (StepperNotReady.RIGHT_LIMIT, "右限位输入有效"),
        (StepperNotReady.POWER_LOSS, "掉电标志已置位"),
        (StepperNotReady.INTERNAL_REJECT, "驱动参数被内部校验拒绝"),
    )
    flags = StepperNotReady(detail)
    reasons = [label for bit, label in labels if flags & bit]
    if not reasons:
        reasons.append("固件未提供就绪原因，请确认已烧录 r16 或更新版本")
    return f"步进运动条件不满足：{'；'.join(reasons)}（detail=0x{detail:08X}）"


def _enum_name(value: Any) -> str:
    return str(getattr(value, "name", value))


def _object_payload(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _object_payload(item) for key, item in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(key): _object_payload(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_object_payload(item) for item in value]
    if hasattr(value, "name") and hasattr(value, "value"):
        return {"value": int(value), "name": value.name}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class DashboardController:
    """Serialize RPC mutations and enforce an additional dashboard safety policy."""

    def __init__(
        self,
        session: Any,
        identity: Any,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.session = session
        self.identity = identity
        self.token = secrets.token_urlsafe(24)
        self._clock = clock
        self._operation_lock = threading.Lock()
        self._link_lock = threading.Lock()
        self._link_available = True
        self._armed_lock = threading.Lock()
        self._armed: ArmedAction | None = None
        self._status: Any = None
        self._status_at = float("-inf")
        self._parameters: Any = None
        self._calibration: Any = None
        self._stepper: Any = None
        self._stepper_at = float("-inf")
        self._h_mission: Any = None
        self._h_mission_at = float("-inf")
        self._manual_lock = threading.Lock()
        self._manual_condition = threading.Condition(self._manual_lock)
        self._manual_controller_id: str | None = None
        self._manual_last_sequence = 0
        self._manual_last_input: tuple[int, int, int, int] | None = None
        self._manual_generation = 0
        self._manual_pending: tuple[int, int, int, int, bool, bool] | None = None
        self._manual_target = (0, 0)
        self._manual_driving = False
        self._manual_last_submit_at = float("-inf")
        self._manual_last_ack_at = float("-inf")
        self._manual_error: str | None = None
        self._manual_device: Any = None
        self._manual_stop_worker = False
        self._manual_thread = threading.Thread(
            target=self._manual_worker,
            name="m6-dashboard-manual-drive",
            daemon=True,
        )
        self._manual_thread.start()

    @property
    def capabilities(self) -> Capability:
        session_capabilities = getattr(self.session, "capabilities", None)
        if session_capabilities is not None:
            return Capability(int(session_capabilities))
        return Capability(int(getattr(self.identity, "enabled_capabilities", 0)))

    @property
    def link_available(self) -> bool:
        with self._link_lock:
            return self._link_available

    def _require_link_available(self) -> None:
        if not self.link_available:
            raise Disconnected("vehicle link is reconnecting")

    def link_lost(self) -> None:
        """Revoke every host-side motion lease before reconnecting."""

        with self._link_lock:
            self._link_available = False
        with self._armed_lock:
            self._armed = None
        with self._manual_condition:
            self._manual_generation += 1
            self._manual_controller_id = None
            self._manual_last_input = None
            self._manual_pending = None
            self._manual_driving = False
            self._manual_target = (0, 0)
            self._manual_error = "vehicle link is reconnecting"
            self._manual_device = None
            self._manual_condition.notify_all()

    def _link_restored(self, identity: Any, parameters: Any = None) -> None:
        self.identity = identity
        self._status = None
        self._parameters = parameters
        self._calibration = None
        self._stepper = None
        self._stepper_at = float("-inf")
        self._h_mission = None
        self._h_mission_at = float("-inf")
        self._status_at = float("-inf")
        with self._link_lock:
            self._link_available = True
        with self._manual_lock:
            self._manual_error = None

    def control_state(self, *, force: bool = False) -> dict[str, Any]:
        status_error: str | None = None
        now = self._clock()
        if self.link_available:
            try:
                with self._operation_lock:
                    self._require_link_available()
                    if force or self._status is None or now - self._status_at >= 0.5:
                        self._status = self.session.get_status()
                        if self.capabilities & Capability.MANUAL_DRIVE:
                            self._manual_device = self.session.get_manual_drive()
                        self._status_at = now
            except M6Error as exc:
                status_error = str(exc)
        else:
            status_error = "vehicle link is reconnecting"
        status = self._status
        with self._armed_lock:
            armed = self._armed
            if armed is not None and now >= armed.expires_at:
                self._armed = None
                armed = None
        status_payload = _object_payload(status) if status is not None else {}
        fault_details: tuple[dict[str, Any], ...] = ()
        if status is not None:
            for field in ("action_state", "action_kind", "source"):
                if hasattr(status, field):
                    status_payload[f"{field}_name"] = _enum_name(getattr(status, field))
            fault_details = decode_safety_faults(int(status.fault_bits))
        return {
            "ok": True,
            "control_token": self.token,
            "connected": self.link_available and bool(
                getattr(self.session, "connected", True)
            ),
            "capabilities": int(self.capabilities),
            "actions": self._action_catalog(),
            "status": status_payload,
            "fault_details": list(fault_details),
            "status_error": status_error,
            "armed": self._armed_payload(armed, now),
            "manual_drive": self._manual_payload(now),
        }

    @staticmethod
    def _mix_manual_targets(
        throttle: int, steering: int, max_speed_mm_s: int,
        steering_percent: int,
    ) -> tuple[int, int]:
        if throttle not in (-1, 0, 1) or steering not in (-1, 0, 1):
            raise ValueError("WASD 前后与转向分量必须为 -1、0 或 1")
        if not 50 <= max_speed_mm_s <= 400:
            raise ValueError("WASD 最高速度必须为 50–400 mm/s")
        if not 10 <= steering_percent <= 100:
            raise ValueError("WASD 转向力度必须为 10–100%")
        strength = steering_percent / 100.0
        raw_left = throttle - steering * strength
        raw_right = throttle + steering * strength
        scale = max(1.0, abs(raw_left), abs(raw_right))
        return (
            round(raw_left / scale * max_speed_mm_s),
            round(raw_right / scale * max_speed_mm_s),
        )

    def manual_enable(self) -> dict[str, Any]:
        self._require_link_available()
        if self.capabilities & Capability.MANUAL_DRIVE != Capability.MANUAL_DRIVE:
            raise ValueError("当前固件未协商网页手动驾驶能力")
        controller_id = secrets.token_urlsafe(18)
        with self._manual_condition:
            self._manual_generation += 1
            if self._manual_driving:
                self._manual_pending = (
                    self._manual_generation, self._manual_last_sequence + 1,
                    0, 0, False, True,
                )
            self._manual_controller_id = controller_id
            self._manual_last_sequence = 0
            self._manual_last_input = None
            self._manual_driving = False
            self._manual_target = (0, 0)
            self._manual_error = None
            self._manual_condition.notify()
        return {"ok": True, "controller_id": controller_id, "manual_drive": self._manual_payload(self._clock())}

    def manual_command(self, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        controller_id = str(request.get("controller_id", ""))
        try:
            sequence = int(request["sequence"])
            throttle = int(request["throttle"])
            steering = int(request["steering"])
            max_speed = int(request.get("max_speed_mm_s", 160))
            steering_percent = int(request.get("steering_percent", 65))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("WASD 指令字段无效") from exc
        left, right = self._mix_manual_targets(
            throttle, steering, max_speed, steering_percent
        )
        now = self._clock()
        with self._manual_condition:
            if not controller_id or controller_id != self._manual_controller_id:
                raise ValueError("网页键盘控制权已失效，请重新启用")
            if sequence <= self._manual_last_sequence:
                raise ValueError("WASD 输入序号过期")
            changed = self._manual_last_input != (
                throttle, steering, max_speed, steering_percent
            )
            takeover = not self._manual_driving and (left != 0 or right != 0)
            self._manual_last_sequence = sequence
            self._manual_last_input = (throttle, steering, max_speed, steering_percent)
            self._manual_last_submit_at = now
            self._manual_target = (left, right)
            self._manual_driving = left != 0 or right != 0
            self._manual_pending = (
                self._manual_generation, sequence, left, right, takeover,
                not self._manual_driving,
            )
            self._manual_error = None
            self._manual_condition.notify()
        return {
            "ok": True,
            "accepted": True,
            "changed": changed,
            "sequence": sequence,
            "target_mm_s": [left, right],
        }

    def manual_disable(self, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        controller_id = str(request.get("controller_id", ""))
        with self._manual_condition:
            if controller_id and controller_id != self._manual_controller_id:
                raise ValueError("网页键盘控制权已失效")
            sequence = self._manual_last_sequence + 1
            self._manual_generation += 1
            self._manual_controller_id = None
            self._manual_last_sequence = sequence
            self._manual_last_input = None
            self._manual_target = (0, 0)
            self._manual_driving = False
            self._manual_pending = (
                self._manual_generation, sequence, 0, 0, False, True,
            )
            self._manual_condition.notify()
        return {"ok": True, "accepted": True}

    def _manual_payload(self, now: float) -> dict[str, Any]:
        with self._manual_lock:
            ack_age = None if self._manual_last_ack_at == float("-inf") else max(
                0, round((now - self._manual_last_ack_at) * 1000)
            )
            return {
                "available": self.capabilities & Capability.MANUAL_DRIVE == Capability.MANUAL_DRIVE,
                "enabled": self._manual_controller_id is not None,
                "driving": self._manual_driving,
                "target_mm_s": list(self._manual_target),
                "last_sequence": self._manual_last_sequence,
                "ack_age_ms": ack_age,
                "error": self._manual_error,
                "device": _object_payload(self._manual_device),
            }

    def _manual_worker(self) -> None:
        while True:
            with self._manual_condition:
                while self._manual_pending is None and not self._manual_stop_worker:
                    self._manual_condition.wait()
                if self._manual_stop_worker:
                    return
                command = self._manual_pending
                self._manual_pending = None
            assert command is not None
            generation, sequence, left, right, takeover, stop = command
            try:
                with self._operation_lock:
                    self._require_link_available()
                    with self._manual_lock:
                        if generation != self._manual_generation:
                            continue
                    if stop:
                        result = self.session.stop_manual_drive(command_id=max(1, sequence))
                    else:
                        result = self.session.set_manual_drive(
                            left, right, lease_ms=300, takeover=takeover,
                            command_id=max(1, sequence),
                        )
                    self._status = None
                with self._manual_lock:
                    self._manual_last_ack_at = self._clock()
                    self._manual_error = None
                    self._manual_device = result
                    if hasattr(result, "target_mm_s"):
                        self._manual_target = tuple(result.target_mm_s)
            except Exception as exc:  # transport errors are surfaced in control state
                with self._manual_lock:
                    self._manual_error = str(exc)

    def close(self) -> None:
        try:
            state = self.calibration_state(force=True)
            if state.get("owner_name") == "HOST" and state.get("active"):
                self.session.cancel_grayscale_calibration(
                    expected_generation=state["generation"]
                )
        except Exception:
            pass
        with self._manual_condition:
            self._manual_stop_worker = True
            self._manual_condition.notify_all()
        self._manual_thread.join(timeout=1.0)

    def calibration_state(self, *, force: bool = False) -> dict[str, Any]:
        self._require_link_available()
        available = (
            self.capabilities & Capability.GRAYSCALE_CALIBRATION
            == Capability.GRAYSCALE_CALIBRATION
        )
        if not available:
            return {"ok": True, "available": False, "active": False}
        with self._operation_lock:
            self._require_link_available()
            if force or self._calibration is None:
                self._calibration = self.session.get_grayscale_calibration()
            status = self._calibration
        payload = _object_payload(status)
        payload["state_name"] = _enum_name(status.state)
        payload["owner_name"] = _enum_name(status.owner)
        payload["active_source_name"] = _enum_name(status.active_source)
        payload["result_name"] = _enum_name(status.result)
        payload["active"] = status.owner != GrayscaleCalibrationOwner.NONE
        failures = []
        masks = (
            ("polarity", status.polarity_fail_mask, "白底均值必须高于黑线均值"),
            ("span", status.span_fail_mask, "白黑跨度不足 512 ADC"),
            ("noise", status.noise_fail_mask, "跨度不足峰峰噪声的 8 倍"),
            ("rail", status.rail_fail_mask, "采样触碰 ADC 轨（0 或 4095）"),
        )
        for channel in range(8):
            reasons = [text for _, mask, text in masks if mask & (1 << channel)]
            if reasons:
                failures.append({"channel": channel + 1, "reasons": reasons})
        payload["failures"] = failures
        return {"ok": True, "available": True, **payload}

    def calibration_command(
        self, command: str, request: Mapping[str, Any]
    ) -> dict[str, Any]:
        self._require_link_available()
        if (
            self.capabilities & Capability.GRAYSCALE_CALIBRATION
            != Capability.GRAYSCALE_CALIBRATION
        ):
            raise ValueError("当前固件未协商灰度自动标定能力")
        if command == "confirm" and request.get("confirm") != "WRITE_GRAYSCALE_CALIBRATION_TO_FLASH":
            raise ValueError("写入 Flash 前需要明确确认")
        expected = request.get("expected_generation")
        expected_generation = None if expected is None else int(expected)
        methods = {
            "start": self.session.start_grayscale_calibration,
            "capture-white": self.session.capture_grayscale_white,
            "capture-black": self.session.capture_grayscale_black,
            "confirm": self.session.confirm_grayscale_calibration,
            "cancel": self.session.cancel_grayscale_calibration,
        }
        method = methods.get(command)
        if method is None:
            raise ValueError("未知灰度标定操作")
        with self._operation_lock:
            self._require_link_available()
            if command == "start":
                result = method()
            else:
                result = method(expected_generation=expected_generation)
            self._calibration = result
            self._status = None
            if command == "confirm":
                self.identity = self.session.identity
        return self.calibration_state()

    def parameter_state(self, *, force: bool = False) -> dict[str, Any]:
        self._require_link_available()
        with self._operation_lock:
            self._require_link_available()
            if force or self._parameters is None:
                self._parameters = self.session.get_parameters()
            snapshot = self._parameters
        values = snapshot.values
        parameters = []
        for spec in REGISTRY["parameters"]:
            key = str(spec["key"])
            value = values[key]
            parameters.append(
                {
                    "id": int(spec["id"]),
                    "key": key,
                    "group": key.split(".", 1)[0],
                    "type": str(spec["type"]),
                    "unit": str(spec["unit"]),
                    "minimum": spec["minimum"],
                    "maximum": spec["maximum"],
                    "default": spec["default"],
                    "flags": list(spec["flags"]),
                    "relations": list(spec.get("relations", [])),
                    "active": value.active,
                    "staged": value.staged,
                    "active_source": _enum_name(value.active_source),
                    "staged_source": _enum_name(value.staged_source),
                }
            )
        return {
            "ok": True,
            "summary": {
                "parameter_schema": snapshot.parameter_schema,
                "active_revision": snapshot.active_revision,
                "stage_revision": snapshot.stage_revision,
                "generation": snapshot.generation,
                "active_source": _enum_name(snapshot.active_source),
                "flags": snapshot.flags,
                "slot": snapshot.slot,
                "storage_health": snapshot.storage_health,
                "save_failures": snapshot.save_failures,
            },
            "parameters": parameters,
        }

    def arm(self, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        if request.get("safety_ack") is not True:
            raise ValueError("必须先确认测试区域安全且急停可用")
        try:
            raw_kind = request["kind"]
            kind = ActionKind[raw_kind] if isinstance(raw_kind, str) else ActionKind(int(raw_kind))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("未知运动类型") from exc
        if kind not in _DASHBOARD_ACTIONS:
            raise ValueError("该运动类型未在网页控制器中开放")
        continuous = request.get("continuous") is True
        if continuous and kind != ActionKind.LINE_FOLLOW:
            raise ValueError("只有自动循迹可以持续运行")
        duration_ms = 0 if continuous else int(request.get("duration_ms", 1500))
        if duration_ms > _MAX_DASHBOARD_ACTION_MS:
            raise ValueError("网页控制单次运动最长为 10000 ms")
        spec = ActionSpec(
            kind=kind,
            wheel_mask=int(request.get("wheel_mask", 3)),
            target_l_mm_s=int(request.get("target_l_mm_s", 0)),
            target_r_mm_s=int(request.get("target_r_mm_s", 0)),
            angle_mdeg=round(float(request.get("angle_deg", 0)) * 1000),
            yaw_rate_mdeg_s=round(float(request.get("yaw_rate_deg_s", 0)) * 1000),
            duration_ms=duration_ms,
        )
        with self._operation_lock:
            self._require_link_available()
            status = self.session.get_status()
            self._status, self._status_at = status, self._clock()
            if int(status.fault_bits):
                raise ValueError("小车存在 Safety 故障，不能 Arm")
            armed = self.session.arm_action(spec)
        with self._armed_lock:
            self._armed = armed
        return {"ok": True, "armed": self._armed_payload(armed, self._clock())}

    def confirm(self, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        try:
            arm_token = int(request["arm_token"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("缺少有效的 Arm token") from exc
        with self._armed_lock:
            armed = self._armed
            if armed is None or armed.token != arm_token:
                raise ValueError("Arm 状态已失效，请重新 Arm")
            if self._clock() >= armed.expires_at:
                self._armed = None
                raise ValueError("Arm 已超过 10 秒，请重新 Arm")
            # A token gets one host-side confirm attempt, including across tabs.
            self._armed = None
        with self._operation_lock:
            self._require_link_available()
            status = self.session.confirm_action(armed)
            self._status = None
        action = _object_payload(status)
        action["spec"] = _object_payload(armed.spec)
        return {"ok": True, "action": action}

    def cancel(self) -> dict[str, Any]:
        self._require_link_available()
        with self._operation_lock:
            self._require_link_available()
            status = self.session.cancel_action(0)
            self._status = None
        with self._armed_lock:
            self._armed = None
        return {"ok": True, "action": _object_payload(status)}

    def emergency_stop(self) -> dict[str, Any]:
        """Queue the vehicle-only raw stop without touching the stepper."""

        self._require_link_available()
        self.session.emergency_stop()
        with self._armed_lock:
            self._armed = None
        self._status = None
        with self._manual_lock:
            self._manual_controller_id = None
            self._manual_driving = False
            self._manual_target = (0, 0)
        return {"ok": True, "accepted": True}

    def _stepper_available(self) -> bool:
        return bool(int(self.capabilities) & STEPPER_CAPABILITY_MASK)

    def _h_mission_available(self) -> bool:
        return bool(int(self.capabilities) & H_MISSION_CAPABILITY_MASK)

    @staticmethod
    def _h_mission_payload(snapshot: Any) -> dict[str, Any]:
        payload = _object_payload(snapshot)
        mode_names = {2: "H2", 4: "H4", 5: "H5"}
        state_names = (
            "idle", "running", "completion_latched", "stopping",
            "complete", "aborted",
        )
        stop_names = (
            "none", "h2_finish", "h4_distance", "h5_finish_line",
            "development_distance", "user_cancel", "fault", "gyro",
            "line_white", "outer_sensor", "negative_target", "saturation",
            "stop_timeout", "telemetry_gap", "stop_distance",
        )
        payload["selected_mode_name"] = mode_names.get(snapshot.selected_mode, "unknown")
        payload["running_mode_name"] = mode_names.get(snapshot.running_mode, "unknown")
        payload["state_name"] = state_names[snapshot.state] if 0 <= snapshot.state < len(state_names) else "unknown"
        payload["stop_reason_name"] = stop_names[snapshot.stop_reason] if 0 <= snapshot.stop_reason < len(stop_names) else "unknown"
        payload["ram_only"] = True
        payload["save_supported"] = False
        return payload

    def h_mission_state(self, *, force: bool = False) -> dict[str, Any]:
        self._require_link_available()
        if not self._h_mission_available():
            return {"ok": True, "available": False, "ram_only": True,
                    "save_supported": False, "snapshot": None}
        with self._operation_lock:
            self._require_link_available()
            now = self._clock()
            if force or self._h_mission is None or now - self._h_mission_at >= 0.2:
                self._h_mission = self.session.get_h_mission()
                self._h_mission_at = now
            snapshot = self._h_mission
        return {"ok": True, "available": True, "ram_only": True,
                "save_supported": False,
                "snapshot": self._h_mission_payload(snapshot)}

    def h_mission_command(self, method: str, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        if not self._h_mission_available():
            raise ValueError("当前固件未协商 HMission 能力")
        with self._operation_lock:
            self._require_link_available()
            current = self.session.get_h_mission()
            if method == "stage":
                config = HMissionConfig(
                    h4_straight_speed_mm_s=int(request["h4_straight_speed_mm_s"]),
                    h4_b_margin_mm=int(request["h4_b_margin_mm"]),
                    h5_straight_speed_mm_s=int(request["h5_straight_speed_mm_s"]),
                    h5_curve_speed_mm_s=int(request["h5_curve_speed_mm_s"]),
                    development_end_distance_mm=int(request.get("development_end_distance_mm", 0)),
                )
                snapshot = self.session.stage_h_mission(config, snapshot=current)
            elif method == "apply":
                snapshot = self.session.apply_h_mission(snapshot=current)
            elif method == "revert":
                snapshot = self.session.revert_h_mission(snapshot=current)
            elif method == "select":
                snapshot = self.session.select_h_mission(int(request["mode"]), snapshot=current)
            elif method == "abort":
                snapshot = self.session.abort_h_mission(
                    telemetry_gap=bool(request.get("telemetry_gap", False))
                )
            elif method == "start":
                snapshot = self.session.start_h_mission(snapshot=current)
            else:
                raise ValueError("未知 HMission 命令")
            self._h_mission = snapshot
            self._h_mission_at = self._clock()
        return {"ok": True, "available": True, "ram_only": True,
                "save_supported": False,
                "snapshot": self._h_mission_payload(snapshot)}

    @staticmethod
    def _stepper_payload(snapshot: Any) -> dict[str, Any]:
        payload = _object_payload(snapshot)
        age = int(getattr(snapshot, "response_age_ms", 0xFFFFFFFF))
        payload["position_revolutions"] = float(snapshot.position_revolutions)
        payload["position_degrees"] = float(snapshot.position_degrees)
        payload["link_state"] = (
            "offline" if age == 0xFFFFFFFF or age > 3000 else
            "stale" if age > 1000 else "online"
        )
        state_names = ("idle", "waiting_ack", "running", "completed", "error")
        kind_names = ("none", "velocity", "position")
        stop_names = (
            "none", "user", "disabled", "duration", "position_reached",
            "timeout", "communication", "device_rejected", "startup",
        )
        payload["state_name"] = state_names[snapshot.state] if 0 <= snapshot.state < len(state_names) else "unknown"
        payload["kind_name"] = kind_names[snapshot.kind] if 0 <= snapshot.kind < len(kind_names) else "unknown"
        payload["stop_reason_name"] = stop_names[snapshot.stop_reason] if 0 <= snapshot.stop_reason < len(stop_names) else "unknown"
        motor_labels = (
            (0x01, "已使能"), (0x02, "已到位"), (0x04, "堵转"),
            (0x08, "堵转保护"), (0x10, "左限位"), (0x20, "右限位"),
            (0x80, "掉电"),
        )
        home_labels = (
            (0x01, "编码器就绪"), (0x02, "校准就绪"), (0x04, "回零中"),
            (0x08, "回零失败"), (0x10, "过温"), (0x20, "过流"),
        )
        payload["motor_flags"] = [label for bit, label in motor_labels if snapshot.motor_status & bit]
        payload["homing_flags"] = [label for bit, label in home_labels if snapshot.homing_status & bit]
        return payload

    def stepper_state(self, *, force: bool = False, refresh: bool = False) -> dict[str, Any]:
        self._require_link_available()
        if not self._stepper_available():
            return {"ok": True, "available": False, "safety_independent": True,
                    "snapshot": None, "error": "当前固件未协商步进电机能力"}
        error: str | None = None
        with self._operation_lock:
            self._require_link_available()
            now = self._clock()
            if force or refresh or self._stepper is None or now - self._stepper_at >= 0.2:
                try:
                    mask = StepperRefresh(0x0F) if refresh else StepperRefresh(0)
                    self._stepper = self.session.get_stepper(mask)
                    self._stepper_at = now
                except M6Error as exc:
                    error = str(exc)
            snapshot = self._stepper
        return {
            "ok": True,
            "available": True,
            "safety_independent": True,
            "snapshot": self._stepper_payload(snapshot) if snapshot is not None else None,
            "error": error,
        }

    def _stepper_call(self, method: str, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        if not self._stepper_available():
            raise ValueError("当前固件未协商步进电机能力")
        with self._operation_lock:
            self._require_link_available()
            if method == "enable":
                if request.get("confirm") != "ENABLE_ZDT_STEPPER":
                    raise ValueError("使能步进电机需要明确确认")
                snapshot = self.session.enable_stepper()
            elif method == "stop":
                snapshot = self.session.stop_stepper()
            elif method == "disable":
                snapshot = self.session.disable_stepper()
            elif method in ("velocity", "position"):
                expected = "START_ZDT_VELOCITY_TEST" if method == "velocity" else "START_ZDT_POSITION_TEST"
                if request.get("confirm") != expected:
                    raise ValueError("启动步进测试需要明确确认")
                try:
                    direction = int(request["direction"])
                    speed = int(request["speed_rpm"])
                    acceleration = int(request["acceleration"])
                    # Browser timers are throttled in background tabs. Refresh
                    # the safety-critical 0x3A status inside this serialized
                    # POST so a newly focused page cannot race the 500 ms
                    # firmware freshness check.
                    self.session.get_stepper(StepperRefresh.MOTOR_STATUS)
                    time.sleep(_STEPPER_MOTION_PREFLIGHT_S)
                    self._stepper = self.session.get_stepper(StepperRefresh(0))
                    self._stepper_at = self._clock()
                    if method == "velocity":
                        snapshot = self.session.start_stepper_velocity_test(
                            direction=direction, speed_rpm=speed,
                            acceleration=acceleration,
                            duration_ms=int(request["duration_ms"]),
                        )
                    else:
                        snapshot = self.session.start_stepper_position_test(
                            direction=direction, speed_rpm=speed,
                            acceleration=acceleration, pulses=int(request["pulses"]),
                            timeout_ms=int(request["timeout_ms"]),
                        )
                except DeviceRejected as exc:
                    if exc.result == int(RpcResult.WRONG_STATE):
                        raise M6Error(_stepper_not_ready_message(exc.detail)) from exc
                    raise
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError("步进测试参数无效或超出允许范围") from exc
            else:
                raise ValueError("未知步进电机命令")
            self._stepper = snapshot
            self._stepper_at = self._clock()
        return {"ok": True, "available": True,
                "snapshot": self._stepper_payload(snapshot)}

    def stepper_enable(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return self._stepper_call("enable", request)

    def stepper_stop(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return self._stepper_call("stop", request)

    def stepper_disable(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return self._stepper_call("disable", request)

    def stepper_velocity(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return self._stepper_call("velocity", request)

    def stepper_position(self, request: Mapping[str, Any]) -> dict[str, Any]:
        return self._stepper_call("position", request)

    def clear_fault(self, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        if request.get("confirm") != "CLEAR_OBSERVED_FAULTS":
            raise ValueError("清除 Safety 故障需要明确确认")
        if self.capabilities & Capability.FAULT_CLEAR != Capability.FAULT_CLEAR:
            raise ValueError("当前固件未协商清故障能力")
        try:
            observed = int(request["observed_fault_bits"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("缺少有效的已观察故障位") from exc
        with self._operation_lock:
            self._require_link_available()
            status = self.session.get_status()
            current = int(status.fault_bits)
            if current == 0:
                raise ValueError("当前没有可清除的 Safety 故障")
            if observed != current:
                self._status, self._status_at = status, self._clock()
                raise ValueError("故障位已变化，请刷新并重新确认")
            result = self.session.clear_fault(current)
            self._status = None
        with self._armed_lock:
            self._armed = None
        return {
            "ok": True,
            "cleared_fault_bits": current,
            "result": _object_payload(result),
        }

    def reconnect(
        self,
        subscriptions: Mapping[Any, int],
        *,
        stopping: threading.Event | None = None,
    ) -> Any:
        """Rebuild the transport/session without replaying any motion command."""

        with self._operation_lock:
            identity = self.session.reconnect()
            if stopping is not None and stopping.is_set():
                try:
                    self.session.close(clean=False)
                except TypeError:
                    self.session.close()
                raise Disconnected("dashboard stopped during reconnect")
            parameter_reader = getattr(self.session, "get_parameters", None)
            parameters = parameter_reader() if callable(parameter_reader) else None
            if stopping is not None and stopping.is_set():
                try:
                    self.session.close(clean=False)
                except TypeError:
                    self.session.close()
                raise Disconnected("dashboard stopped during reconnect")
            self.session.subscribe(subscriptions)
            if stopping is not None and stopping.is_set():
                try:
                    self.session.close(clean=False)
                except TypeError:
                    self.session.close()
                raise Disconnected("dashboard stopped during reconnect")
            self._link_restored(identity, parameters)
        return identity

    def parameter_command(self, command: str, request: Mapping[str, Any]) -> dict[str, Any]:
        self._require_link_available()
        with self._operation_lock:
            self._require_link_available()
            if command == "stage":
                key = str(request.get("key", ""))
                if not key:
                    raise ValueError("缺少参数 key")
                result = self.session.stage_parameter(key, request.get("value"))
            elif command == "apply":
                result = self.session.apply_parameters()
            elif command == "revert":
                result = self.session.revert_parameters()
            elif command == "load":
                result = self.session.load_parameters()
            elif command == "defaults":
                if request.get("confirm") != "LOAD_DEFAULTS_TO_STAGE":
                    raise ValueError("恢复默认值需要明确确认")
                result = self.session.default_parameters()
            elif command == "save":
                if request.get("confirm") != "SAVE_ACTIVE_PARAMETERS":
                    raise ValueError("持久化保存需要明确确认")
                result = self.session.save_parameters()
            else:
                raise ValueError("未知参数操作")
            self._parameters = None
            self._status = None
        return {"ok": True, "result": _object_payload(result)}

    def _action_catalog(self) -> list[dict[str, Any]]:
        result = []
        for rule in REGISTRY["semantics"]["actions"]["kinds"]:
            kind = ActionKind[str(rule["name"])]
            required = [Capability[name] for name in rule["required_capabilities"]]
            available = kind in _DASHBOARD_ACTIONS and all(
                self.capabilities & capability == capability for capability in required
            )
            item = dict(rule)
            item["value"] = int(kind)
            item["available"] = available
            if kind == ActionKind.MOTOR_SELF_TEST:
                item["unavailable_reason"] = "网页控制器暂不开放电机自检"
            elif not available:
                item["unavailable_reason"] = "当前固件未协商所需能力"
            result.append(item)
        return result

    @staticmethod
    def _armed_payload(armed: ArmedAction | None, now: float) -> dict[str, Any] | None:
        if armed is None:
            return None
        return {
            "arm_token": armed.token,
            "expires_in_ms": max(0, round((armed.expires_at - now) * 1000)),
            "spec": _object_payload(armed.spec),
            "kind_name": _enum_name(armed.spec.kind),
        }


__all__ = ["DashboardController"]
