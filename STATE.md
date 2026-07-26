# MSPM03507CAR 当前状态

更新时间：2026-07-27

本文只保留当前实现、不可破坏的安全边界、有效验证和下一步。历史过程见 `docs/`。

## 1. 当前基线

- 平台：MSPM0G3507-64P 两轮差速小车，32 MHz，Keil ARM Compiler 6.22；入口为 `Source/main.c`，冻结板级配置修订号 `20260725`。
- 车上运行正式 `M6-runtime-20260726-r14`。已确认 schema/registry/active revision 均为 1，参数来源 `defaults`、generation 0，V2 mask 为 `0x000000000053EFFF`。
- 主机保留 `m6link`、薄 CLI `scripts/m6_tune.py`、冻结 registry/golden vectors、记录/回放/评分；GUI 和 GUI 验收脚本已删除。
- 唯一活动仓库/worktree 为当前目录。本地 `main` 尚未推送；仅另保留历史分支 `codex/hc04-debug`。默认 `git status` 干净。

## 2. 已实现能力

- r14 包含 256 B UART RX 环、每轮 32 B 预算、后台格式化，以及冻结 V2 codec、HELLO/session、window=1 重放/去重、V1 resync、P0–P3 优先队列和背压诊断。
- V2 已发布身份/状态/reset/task/UART/link、参数事务、telemetry、fault clear、两阶段独立轮速/航向动作、航向校准和 ±360° 相对转向。
- V2 未发布 line action、trace/bulk、灰度批采、motor self-test 或 heading-rate action。
- `m6link.Session` 是唯一串口所有者；默认探测 V2，V1 必须显式 legacy。RPC 不盲重试。
- `arm_action()` 只返回 token 和 10 秒有效期；Confirm 必须由人工独立调用，库和 CLI 不得自动 Confirm。
- 参数共 35 个；A/B 持久化扇区为 `0x1F800`/`0x1FC00`，无有效记录时使用 defaults 并报告 `DEGRADED`。

## 3. 安全边界

- `System/safety.c` 是唯一电机输出所有者；无动作时两路制动，应用、诊断和遥控不得直接写 PWM。
- 裸 `S` 或空格在 UART RX ISR 的任何 parser 状态下立即制动并锁存 `FAULT`；`x` 只清故障到 `IDLE`，不恢复动作。
- WWDT0 约 31 ms；只有 5/10/20 ms 关键任务均按期完成时才喂狗，不得延长、禁用或旁路。
- 参数 `apply` 只允许无故障 `IDLE`；`save` 还要求无动作、轮速环关闭且电机制动。
- 轮目标硬限幅 ±700 mm/s；动作/传感器/遥控超时、编码器异常和堵转均受故障门控。
- 无线链路不能替代可立即触达的物理断电手段。

## 4. 关键硬件

| 功能 | 当前配置 |
|---|---|
| UART / DAP | PA10/PA11，115200 8N1，当前 COM10；PA19/PA20/NRST，Horco CMSIS-DAP v2，SWD 1 MHz |
| JY61P | PA22 UART2_RX，9600，只接模块 TX；模块平放，PA21/VREF− 保持高阻 |
| 电机 / 编码器 | A：PB3/PA14/PA13、PA8/PA9；B：PB2/PA16/PA17、PB9/PB8；PWM 4 kHz |
| 灰度 | PB1/PA24/PA26 地址，PA25 ADC；OUT 必须不高于 3.3 V，模块或安装变化后重新标定 |
| 红外 | PB11 TIMG0_CCP1；不可与旋转编码器按压开关共用 |

完整接线见 `docs/接线说明.md`。

## 5. 有效验证

- 2026-07-27：全量 125 项主机测试和冻结 V2 checker 通过。
- r14 Keil Rebuild：0 error、0 warning；`Code=60836, RO-data=9720, RW-data=12, ZI-data=28972`；最高普通镜像字节 `0x113AF`，低于参数区；256 B `RW_NOINIT` 位于 `0x20207F00`。
- r14 已通过 V1 fallback、V2 HELLO、身份/Safety/reset/诊断基线，以及日志关闭和 10/25/50 Hz 各约 60 秒压力测试；无复位、任务漏期、高优先级丢包或 link error。
- V1 半帧和 V2 背压下的裸字节急停通过。
- 架空双轮 100/300 mm/s、左右单轮各 160 mm/s通过，均正常停止。
- 落地 160 mm/s 前进、左弧、右弧已执行并安全停止，但缺有效记录，性能不计通过。
- PB11 架空路径及落地 100 mm/s 前进/航向保持/松键制动通过；落地后退/左右转未正式验收。
- M5 模拟灰度接线和静态标定通过；组合镜像落地巡线尚未验收。M7 尚未开始。

## 6. 下一步

1. 建立可靠的非 GUI 验收入口，保留人工独立 Confirm，活动 Recorder 中不得交错诊断 RPC。
2. 重新记录 160 mm/s 直线和左右弧；通过后按 `180→200→220→250 mm/s` 分级验证。
3. 验证 r14 航向保持、±45°、±90° 和可配置角度动作。
4. 完成长时间 50 Hz 链路、参数保存/复位/断电保持、PB11 剩余动作、M5 巡线和 M7 安全验收。

出现故障、boot ID 变化、任务漏期、控制/RPC 丢包、遥测缺口、连续 PWM 饱和达到 300 ms 或明显振荡时立即停止。相同问题两次修复仍失败时停止试变体。

## 7. 常用入口

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/flash.ps1 -PreflightOnly
python scripts/m6_tune.py probe --json
```

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
- 命令行下载失败只允许原命令重试一次，第二次失败后改由用户手动下载。
- 文档索引：`docs/README.md`。
- M6 当前证据：`docs/2026-07-26-M6-V2通信与复位可观测性验证记录.md`。
- V2 合同：`docs/2026-07-26-M6-V2-wire-registry-v1.md` 和 `Tests/vectors/m6_v2_wire_v1.json`。
- 最终归档：仓库外 `Archive/2026-07-27-final-clean/MANIFEST.md`。
