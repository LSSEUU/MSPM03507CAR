"""Human-readable Safety fault decoding derived from the frozen registry."""

from __future__ import annotations

from typing import Any

from .registry import REGISTRY


_FAULT_TEXT = {
    "EMERGENCY_STOP": (
        "急停已触发",
        "主机或遥控端发送了立即停机命令。",
        "确认车辆已停稳且测试区域安全，然后手动清除故障。",
    ),
    "GYRO_TIMEOUT": (
        "陀螺仪数据超时",
        "需要陀螺仪的动作期间，固件未及时收到有效陀螺仪数据。",
        "检查陀螺仪供电、接线和数据刷新，再清除故障。",
    ),
    "GRAYSCALE_TIMEOUT": (
        "灰度传感器数据超时",
        "循迹期间灰度数据停止更新或超过看门狗时限。",
        "检查灰度板供电、ADC 与采样任务，再清除故障。",
    ),
    "ACTION_TIMEOUT": (
        "动作执行超时",
        "限时动作未在固件规定的截止时间内正常结束。",
        "检查动作参数和控制环状态，再清除故障。",
    ),
    "SCHEDULER_DEADLINE": (
        "调度任务超期",
        "实时任务错过了允许的调度截止时间。",
        "检查任务阻塞、串口负载和中断耗时，再清除故障。",
    ),
    "WATCHDOG_RESET": (
        "看门狗复位记录",
        "上一次运行发生看门狗复位，固件保留了故障记录。",
        "先查看复位与任务统计，确认原因后再清除故障。",
    ),
    "ENCODER_INVALID": (
        "编码器反馈无效",
        "轮速控制检测到编码器方向、跳变或反馈范围异常。",
        "检查编码器接线、方向和机械传动，再清除故障。",
    ),
    "WHEEL_STALL": (
        "车轮堵转",
        "电机已有明显输出，但编码器速度长期不足。",
        "断电检查卡滞、轮胎和电机驱动，排除后再清除故障。",
    ),
    "LINE_LOST": (
        "循迹丢线",
        "有效灰度通道或循迹置信度持续不足，固件停止了循迹。",
        "把车辆放回赛道并检查阈值、环境光和传感器高度，再清除故障。",
    ),
}


def decode_safety_faults(fault_bits: int) -> tuple[dict[str, Any], ...]:
    """Return every known set bit plus one entry for any unknown bits."""

    bits = int(fault_bits) & 0xFFFFFFFF
    remaining = bits
    result: list[dict[str, Any]] = []
    for entry in REGISTRY["registries"]["safety_fault_bits"]:
        value = int(entry.get("value", 1 << int(entry["bit"])))
        if not bits & value:
            continue
        name = str(entry["name"])
        title, cause, recovery = _FAULT_TEXT.get(
            name,
            (name, "固件报告了该 Safety 故障。", "确认安全并排除原因后再清除故障。"),
        )
        result.append(
            {
                "bit": value,
                "hex": f"0x{value:08X}",
                "name": name,
                "title": title,
                "cause": cause,
                "recovery": recovery,
            }
        )
        remaining &= ~value
    if remaining:
        result.append(
            {
                "bit": remaining,
                "hex": f"0x{remaining:08X}",
                "name": "UNKNOWN",
                "title": "未知 Safety 故障位",
                "cause": "当前上位机版本不认识固件返回的部分故障位。",
                "recovery": "不要继续运行；核对固件与上位机版本后再处理。",
            }
        )
    return tuple(result)


__all__ = ["decode_safety_faults"]
