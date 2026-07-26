#!/usr/bin/env python3
"""One-shot human-confirmed M6 ground-action acceptance window."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk


ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = ROOT / "python" / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from m6link import ActionKind, ActionSpec, Session  # noqa: E402


@dataclass(frozen=True, slots=True)
class Step:
    label: str
    left_mm_s: int
    right_mm_s: int
    duration_ms: int


def build_steps(speed: int, ratio: float, duration_s: float) -> tuple[Step, ...]:
    speed = int(speed)
    duration_ms = round(float(duration_s) * 1000)
    if not 50 <= speed <= 400:
        raise ValueError("speed must be 50..400 mm/s")
    if not 0.1 <= ratio <= 1.0:
        raise ValueError("arc ratio must be 0.1..1.0")
    if not 100 <= duration_ms <= 10_000:
        raise ValueError("duration must be 0.1..10.0 s")
    inner = round(speed * ratio)
    if inner < 50:
        raise ValueError("arc inner wheel must remain at least 50 mm/s")
    return (
        Step(f"前进 {speed}/{speed}", speed, speed, duration_ms),
        Step(f"左弧 {inner}/{speed}", inner, speed, duration_ms),
        Step(f"右弧 {speed}/{inner}", speed, inner, duration_ms),
    )


def _snapshot(session: Session) -> dict[str, object]:
    status = session.get_status()
    reset = session.get_reset_info()
    tasks = session.get_task_stats()
    uart = session.get_uart_stats()
    link = session.get_link_stats()
    return {
        "status": asdict(status),
        "reset": asdict(reset),
        "tasks": asdict(tasks),
        "uart": asdict(uart),
        "link": asdict(link),
    }


def _assert_safe(snapshot: dict[str, object], *, baseline: dict[str, object] | None = None) -> None:
    status = snapshot["status"]
    tasks = snapshot["tasks"]
    uart = snapshot["uart"]
    link = snapshot["link"]
    reset = snapshot["reset"]
    assert isinstance(status, dict) and isinstance(tasks, dict)
    assert isinstance(uart, dict) and isinstance(link, dict) and isinstance(reset, dict)
    if int(status["system_state"]) != 2 or int(status["fault_bits"]):
        raise RuntimeError("Safety is not IDLE/fault-free")
    if any(int(value) for value in status["target_mm_s"]):
        raise RuntimeError("wheel target did not return to zero")
    if any(int(value) for value in status["applied_pwm"]):
        raise RuntimeError("applied PWM did not return to zero")
    if any(int(value) for value in tasks["miss_count"]):
        raise RuntimeError("task miss counter is nonzero")
    if int(uart["rx_drop"]) or int(uart["tx_drop"]) or int(uart["format_count"]):
        raise RuntimeError("UART drop/format counter is nonzero")
    if any(int(value) for value in link["drops"][:2] + link["drops"][3:]):
        raise RuntimeError("P0/P1/P3 drop counter is nonzero")
    if int(link["control_error_drop"]):
        raise RuntimeError("control response drop counter is nonzero")
    for key in ("crc_errors", "frame_errors", "timeout_errors", "session_mismatch", "sequence_errors"):
        if int(link[key]):
            raise RuntimeError(f"link {key} is nonzero")
    if baseline is None:
        return
    base_reset = baseline["reset"]
    base_tasks = baseline["tasks"]
    base_uart = baseline["uart"]
    base_link = baseline["link"]
    assert isinstance(base_reset, dict) and isinstance(base_tasks, dict)
    assert isinstance(base_uart, dict) and isinstance(base_link, dict)
    if int(reset["boot_id"]) != int(base_reset["boot_id"]):
        raise RuntimeError("boot ID changed during acceptance")
    if list(tasks["miss_count"]) != list(base_tasks["miss_count"]):
        raise RuntimeError("task miss increased during action")
    for key in ("rx_drop", "tx_drop", "format_count"):
        if int(uart[key]) != int(base_uart[key]):
            raise RuntimeError(f"UART {key} increased during action")
    if list(link["drops"]) != list(base_link["drops"]):
        raise RuntimeError("P0-P3 drop increased during action")
    if int(link["control_error_drop"]) != int(base_link["control_error_drop"]):
        raise RuntimeError("control response drop increased during action")


class AcceptanceWindow:
    def __init__(self, root: tk.Tk, *, port: str, steps: tuple[Step, ...], output: Path) -> None:
        self.root = root
        self.port = port
        self.steps = steps
        self.output = output
        self.session = Session(timeout=1.0)
        self.identity = None
        self.baseline: dict[str, object] | None = None
        self.snapshots: list[dict[str, object]] = []
        self.index = 0
        self.armed = None
        self.busy = True
        self.action_running = False
        self.closed = False

        root.title("M6 一次性落地动作验收")
        root.geometry("720x400")
        root.protocol("WM_DELETE_WINDOW", self.close)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        self.step_var = tk.StringVar(value="正在建立 V2 与录制基线…")
        self.status_var = tk.StringVar(value=f"端口 {port} | 输出 {output}")
        self.lease_var = tk.StringVar(value="尚未 Arm")
        ttk.Label(frame, textvariable=self.step_var, font=("Microsoft YaHei UI", 18, "bold")).pack(anchor="w")
        ttk.Label(frame, textvariable=self.status_var, wraplength=670).pack(anchor="w", pady=(10, 18))
        ttk.Label(frame, textvariable=self.lease_var, font=("Consolas", 11)).pack(anchor="w", pady=(0, 18))

        row = ttk.Frame(frame)
        row.pack(fill="x")
        self.arm_button = ttk.Button(row, text="Arm 当前步骤", command=self.arm_current, state="disabled")
        self.arm_button.pack(side="left", expand=True, fill="x", padx=(0, 5))
        self.confirm_button = ttk.Button(row, text="Confirm（仅人工点击）", command=self.confirm_current, state="disabled")
        self.confirm_button.pack(side="left", expand=True, fill="x", padx=5)
        self.next_button = ttk.Button(row, text="重新摆位后进入下一步", command=self.next_step, state="disabled")
        self.next_button.pack(side="left", expand=True, fill="x", padx=5)
        self.finish_button = ttk.Button(row, text="完成并保存", command=self.finish, state="disabled")
        self.finish_button.pack(side="left", expand=True, fill="x", padx=(5, 0))

        safety = ttk.Frame(frame)
        safety.pack(fill="x", pady=(22, 0))
        ttk.Button(safety, text="Cancel 正常停止", command=self.cancel).pack(side="left")
        tk.Button(
            safety,
            text="急停 S",
            command=self.emergency_stop,
            bg="#b91c1c",
            fg="white",
            activebackground="#991b1b",
            font=("Microsoft YaHei UI", 12, "bold"),
            padx=24,
            pady=8,
        ).pack(side="right")
        ttk.Label(
            frame,
            text="每一步必须取得新的 10 秒 token，再由操作者点击 Confirm。窗口不会自动 Confirm。动作完成并核对零输出后才允许下一步。",
            wraplength=670,
        ).pack(anchor="w", pady=(22, 0))
        root.bind_all("<KeyPress-s>", lambda _event: self.emergency_stop())
        root.bind_all("<KeyPress-S>", lambda _event: self.emergency_stop())
        root.bind_all("<KeyPress-space>", lambda _event: self.emergency_stop())
        threading.Thread(target=self._initialize, name="m6-accept-init", daemon=True).start()

    def _ui(self, fn) -> None:
        if not self.closed:
            self.root.after(0, fn)

    def _fail(self, exc: BaseException) -> None:
        def show() -> None:
            self.busy = False
            self.action_running = False
            self.arm_button.configure(state="disabled")
            self.confirm_button.configure(state="disabled")
            self.next_button.configure(state="disabled")
            self.finish_button.configure(state="disabled")
            self.status_var.set(f"停止：{type(exc).__name__}: {exc}")
            messagebox.showerror("M6 验收停止", str(exc))
        self._ui(show)

    def _initialize(self) -> None:
        try:
            self.identity = self.session.connect(self.port)
            before = _snapshot(self.session)
            _assert_safe(before)
            self.session.start_recording(self.output, duration_s=None, period_ms=20, test_mode="wheel")
            self.baseline = _snapshot(self.session)
            _assert_safe(self.baseline)
            self.snapshots.append({"phase": "ready", **self.baseline})
        except BaseException as exc:
            self._fail(exc)
            return
        def ready() -> None:
            self.busy = False
            self.step_var.set(f"步骤 1/{len(self.steps)}：{self.steps[0].label}")
            self.status_var.set("V2、录制和安全基线已就绪；确认场地后点击 Arm。")
            self.arm_button.configure(state="normal")
        self._ui(ready)

    def arm_current(self) -> None:
        if self.busy or self.armed is not None:
            return
        self.busy = True
        self.arm_button.configure(state="disabled")
        step = self.steps[self.index]
        self.status_var.set("正在请求 Arm；电机不会启动…")
        threading.Thread(target=self._do_arm, args=(step,), name="m6-accept-arm", daemon=True).start()

    def _do_arm(self, step: Step) -> None:
        try:
            identity = self.identity
            if identity is None:
                raise RuntimeError("identity is unavailable")
            spec = ActionSpec(
                kind=ActionKind.WHEEL_SPEED_TEST,
                wheel_mask=3,
                target_l_mm_s=step.left_mm_s,
                target_r_mm_s=step.right_mm_s,
                duration_ms=step.duration_ms,
                expected_active_revision=identity.active_revision,
            )
            armed = self.session.arm_action(spec)
        except BaseException as exc:
            self._fail(exc)
            return
        def armed_ui() -> None:
            self.armed = armed
            self.busy = False
            self.confirm_button.configure(state="normal")
            self.status_var.set("Arm 成功；请在倒计时内由操作者点击 Confirm。")
            self._countdown()
        self._ui(armed_ui)

    def _countdown(self) -> None:
        if self.armed is None or self.closed:
            return
        remaining = self.armed.expires_at - time.monotonic()
        if remaining <= 0:
            self.armed = None
            self.lease_var.set("token 已过期；必须重新 Arm")
            self.confirm_button.configure(state="disabled")
            self.arm_button.configure(state="normal")
            return
        self.lease_var.set(f"token={self.armed.token} | 剩余 {remaining:0.1f}s")
        self.root.after(100, self._countdown)

    def confirm_current(self) -> None:
        if self.busy or self.armed is None:
            return
        armed = self.armed
        self.busy = True
        self.action_running = True
        self.confirm_button.configure(state="disabled")
        self.status_var.set("正在发送人工 Confirm…")
        threading.Thread(target=self._do_confirm, args=(armed,), name="m6-accept-confirm", daemon=True).start()

    def _do_confirm(self, armed) -> None:
        try:
            self.session.confirm_action(armed)
        except BaseException as exc:
            self._fail(exc)
            return
        step = self.steps[self.index]
        def confirmed() -> None:
            self.armed = None
            self.lease_var.set("Confirm 已由操作者发送；等待动作自动停止…")
            self.status_var.set("动作运行中；异常立即按 S/空格/急停。")
            self.root.after(step.duration_ms + 700, self._verify_current)
        self._ui(confirmed)

    def _verify_current(self) -> None:
        threading.Thread(target=self._do_verify, name="m6-accept-verify", daemon=True).start()

    def _do_verify(self) -> None:
        try:
            current = _snapshot(self.session)
            _assert_safe(current, baseline=self.baseline)
            self.baseline = current
            self.snapshots.append({"phase": f"step_{self.index + 1}", **current})
        except BaseException as exc:
            self._fail(exc)
            return
        def verified() -> None:
            self.busy = False
            self.action_running = False
            self.status_var.set("动作已自动停止且诊断无增量；观察无异常后再继续。")
            if self.index + 1 == len(self.steps):
                self.finish_button.configure(state="normal")
            else:
                self.next_button.configure(state="normal")
        self._ui(verified)

    def next_step(self) -> None:
        if self.busy or self.action_running or self.index + 1 >= len(self.steps):
            return
        self.index += 1
        self.next_button.configure(state="disabled")
        self.step_var.set(f"步骤 {self.index + 1}/{len(self.steps)}：{self.steps[self.index].label}")
        self.status_var.set("请先人工重新摆位并确认场地安全，再点击 Arm。")
        self.arm_button.configure(state="normal")

    def cancel(self) -> None:
        threading.Thread(target=self._do_cancel, name="m6-accept-cancel", daemon=True).start()

    def _do_cancel(self) -> None:
        try:
            self.session.cancel_action(0)
        except BaseException as exc:
            self._fail(exc)
            return
        def done() -> None:
            self.armed = None
            self.action_running = False
            self.busy = False
            self.confirm_button.configure(state="disabled")
            self.arm_button.configure(state="normal")
            self.status_var.set("Cancel 已发送；必须重新 Arm。")
        self._ui(done)

    def emergency_stop(self) -> str:
        threading.Thread(target=self._do_estop, name="m6-accept-estop", daemon=True).start()
        return "break"

    def _do_estop(self) -> None:
        try:
            self.session.emergency_stop()
        except BaseException as exc:
            self._fail(exc)
            return
        def stopped() -> None:
            self.armed = None
            self.action_running = False
            self.busy = True
            self.status_var.set("急停已发送；故障保持锁存，本轮验收终止。")
            self.confirm_button.configure(state="disabled")
            self.arm_button.configure(state="disabled")
            self.next_button.configure(state="disabled")
            self.finish_button.configure(state="disabled")
        self._ui(stopped)

    def finish(self) -> None:
        if self.busy or self.action_running:
            return
        self.busy = True
        self.finish_button.configure(state="disabled")
        self.status_var.set("正在停止录制并保存诊断…")
        threading.Thread(target=self._do_finish, name="m6-accept-finish", daemon=True).start()

    def _write_acceptance(self) -> None:
        path = self.output.with_suffix(".acceptance.json")
        payload = {
            "firmware": getattr(self.identity, "firmware_id", None),
            "port": self.port,
            "steps": [asdict(step) for step in self.steps],
            "snapshots": self.snapshots,
        }
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")

    def _do_finish(self) -> None:
        try:
            self.session.stop_recording()
            self._write_acceptance()
            self.session.close()
        except BaseException as exc:
            self._fail(exc)
            return
        self._ui(self.root.destroy)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        def shutdown() -> None:
            try:
                if self.action_running:
                    self.session.emergency_stop()
                self.session.close()
            finally:
                self.root.after(0, self.root.destroy)
        threading.Thread(target=shutdown, name="m6-accept-close", daemon=True).start()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--port", default="COM10")
    result.add_argument("--speed", type=int, required=True)
    result.add_argument("--ratio", type=float, default=0.7)
    result.add_argument("--duration", type=float, default=1.5)
    result.add_argument("--output", type=Path, required=True)
    return result


def main() -> int:
    args = parser().parse_args()
    steps = build_steps(args.speed, args.ratio, args.duration)
    root = tk.Tk()
    AcceptanceWindow(root, port=args.port, steps=steps, output=args.output.resolve())
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
