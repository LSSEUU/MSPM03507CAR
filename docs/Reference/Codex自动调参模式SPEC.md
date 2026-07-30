# Codex 自动调参模式 SPEC（CAR-TUNE v1）

> 状态：2026-07-30 实车流程已验收；当前采用赛前紧急有界批次模式  
> 默认模式：监督式硬件在环（Supervised HIL）  
> 适用仓库：`MSPM03507CAR`  
> 依据：`AI自动调参方案见解.md`、`H题_MSPM03507CAR_完赛实施与调试SPEC.md`与当前仓库实现  
> 目标：把 Codex、M6 V2、固件 Safety、实车操作员组成可重复、可回滚、有证据的调参闭环

本文中“必须”是安全或证据硬约束；“建议”可根据现场情况调整。

---

# 0. 执行结论

当前最适合的不是无人值守式“AI 自己跑车”，而是监督式自动调参：

- Codex 负责连接、参数快照、候选计算、Stage/Apply、Arm、录制、硬失败判定、评分、接受/回滚和报告；
- 固件 Safety 仍是所有车轮运动的最终门控；
- 操作员负责摆放实物、确认测试区域和急停，并在每次运动前完成真实的 Confirm；
- 只有终验通过且操作员明确同意，才可写入 Flash。

模式分三级：

| 模式 | 当前状态 | 运动权限 | 用途 |
|---|---|---|---|
| `CAR-TUNE/REPLAY` | 可用 | 无 | 离线回放、重算指标、生成候选，不接触实车 |
| `CAR-TUNE/SUPERVISED` | **默认；实车录制、评分、候选接受/回滚已通过验证** | 每次动作独立 Arm/Confirm；操作员可预授权有界批次 | 轮速、航向、循迹调参 |
| `CAR-TUNE/AUTO` | **禁用** | 无人连续确认 | 需第 12 节能力齐备后才能启用 |

竞赛截止日前只实施 `REPLAY` 和 `SUPERVISED`，不为追求“全自动”修改已冻结的 M6 V2 Schema。

---

# 1. 当前能力与边界

## 1.1 已经具备

- M6 V2 设备发现、固件身份和参数 Schema 核对；
- 参数 Stage、Apply、Revert、Load、Save 和 Flash 双槽；
- 两阶段 Arm/Confirm，Arm token 约 10 s 过期；
- `WHEEL_SPEED_TEST`、`HEADING_HOLD`、`HEADING_TURN`、`HEADING_RATE`、`LINE_FOLLOW` 动作；
- 20 ms 周期遥测、动作联动录制、离线回放和完整性统计；
- 轮速/循迹硬失败检测、多指标评分、坐标搜索、拒绝候选回滚；
- 当前 OUT.8IR.B 的物理按键标定、八路资格验证、运行时白底/黑线采样及显式 Flash 确认。

## 1.2 必须承认的限制

- M6 V2 合同明确 `host_auto_confirm=false`；Session 不会也不应自动 Confirm。
- 当前主 CLI 不允许在 V2 上盲目执行无人 `tune-wheel`；保留的兼容实现也要求架空确认和逐次 `CONFIRM`。
- dashboard 原生录制为 `m6-dashboard-session-v1` 多流 `timeline.jsonl`。A0 适配器现已支持直接评分或稳定转换为合并 CSV/JSON，并保留动作元数据及 gap/reset/abort/incomplete 等硬失败；2026-07-30 已完成新底盘左轮、右轮和双轮真实会话交叉验证，详见 `docs/2026-07-30-CAR-TUNE-A0录制适配器验证记录.md`。历史 `summary.json` 或旧底盘分数仍不得作为新 baseline。
- 当前遥测没有电池电压，电池状态只能由人工记录，不能做自动低压补偿。
- 车辆 Safety 尚未统一停止 Emm42；因此 v1 禁止对球杆/步进执行器做无人调参。
- 当前轮速实际 PWM 限制为 `500`，虽然冻结参数表仍允许更大的前馈值，调参器必须使用更严格的实际输出边界。
- 2026-07-25 至 07-26 的 `docs/m6-data` 属于旧底盘/旧方向证据。可用于检查工具，**不得作为 2026-07-30 新底盘的 golden baseline 或性能结论**。

---

# 2. 角色合同

## 2.1 Codex 必须做

1. 进入模式时只保留一个串口所有者，查明旧 M6 进程的 PID 和命令行，不盲目结束无关进程。
2. 记录 Git SHA/工作区状态、固件 ID、Schema、参数 revision/generation/source 和测试条件。
3. 先保存 golden profile，再生成任何候选。
4. 只使用 Stage/Apply 试验候选；拒绝后立即回滚到当前已接受值。
5. 无批次授权时，在候选运动前向操作员报告动作、速度、持续时间、修改的唯一参数及新旧值；有有效批次时写入动作记录并持续执行，不逐次打断操作员。
6. 运动结束后先判硬失败，再计算分数；硬失败不能被“平均分更好”覆盖。
7. 为每个候选留下参数差异、原始数据、指标、决策和回滚结果。

## 2.2 操作员必须做

1. 确认架空/落地状态、测试空间、机械固定和可立即断电。
2. 确认电池条件可比，并告诉 Codex 满电/使用中/低电等状态。
3. 选择逐动作确认，或明确授予一个有界批次；批次必须写明 scope、架空/落地、动作类型、最大速度、单次最长时间、最大次数和退出条件。
4. 观察异响、抖动、结构移动和轨迹危险；任何时候可急停或断电。

## 2.3 禁止的 Codex 行为

- 不直接输出 PWM，不绕过 `Safety_RequestMotor()`；
- 不重用旧 Arm token；没有有效批次授权时不得代操作员 Confirm。有有效批次授权时，每个动作仍必须重新 Arm、使用新 token Confirm，并在动作前自动核对批次边界；
- 不自动清除 Safety fault 后继续运动；
- 不在调参搜索中写 Flash，不使用 `defaults` 代替 golden profile；
- 不同时修改两个参数家族，不把机械/传感器故障当成 PID 问题；
- 不在用户说“停止”、“不对”、“异响”或发来其他紧急信息后继续候选。

---

# 3. 进入、暂停和退出协议

## 3.1 建议启动语句

```text
进入 CAR-TUNE：scope=wheel_lifted，
车轮已架空，急停可用，允许速度=100/160/220/300 mm/s，
只 Apply 不 Save。
```

`scope` 只能是：

- `replay`：纯离线；
- `wheel_lifted`：架空轮速与前馈；
- `ground_heading`：落地直行、轮径和航向；
- `line_track`：低速循迹到整圈；
- `ball_bench`：当前只允许规划/回放，禁止自动运动。

未指定时默认 `scope=wheel_lifted`、`Apply 不 Save`、单次动作不超过 7 s。

## 3.2 有界批次授权

为压缩赛前调参等待时间，`CAR-TUNE/SUPERVISED` 允许操作员一次授权多个动作，但不接受“无限速度、无限次数、无限时长”的授权。批次至少绑定：

- scope 和架空/落地状态；
- 允许的 action kind、wheel mask/方向和最大目标；
- 单次最长持续时间、最多动作数和有效期；
- 允许修改的参数家族，以及 Apply/Save 权限；
- 故障、反向、失效、持续饱和、遥测异常、断连/重启和人工撤销等退出条件。

批次内每个物理动作仍执行独立的预检、Arm、新 token Confirm、联动录制、在线急停监控和离线硬失败判定。任一硬失败会撤销剩余次数，不自动清故障或继续下一项。用户的新消息若包含“停止”“暂停”“不对”“异响”等危险或撤销语义，优先于批次并立即停止。

有界批次只是监督式人工预授权，不把当前系统宣称为 `CAR-TUNE/AUTO`，也不改变固件 `host_auto_confirm=false` 能力声明。

赛前紧急模式默认使用有界批次，不再逐动作请求文字确认；批次边界、自动硬门、回滚和急停规则不降级。Codex 应连续完成同一参数族的基线、候选、重复验证和回滚，只在批次完成、硬失败、需要改变物理测试条件或需要 Save 时暂停并汇报。

2026-07-30 批次 `T2-WHEEL-LIFTED`：仅架空 `WHEEL_SPEED_TEST`，目标绝对值不超过 `300 mm/s`，单次不超过 `7 s`，最多 `24` 次；只允许轮速参数家族 Apply，不 Save；不覆盖落地、循迹、航向、步进电机或滚球动作。该批次在第 18 次动作发现 `ACTION_TIMEOUT` 硬失败后已自动终止；固件 r17 已修复正常结束与 Safety 超时同刻竞争，烧录/重启后必须用新的有界批次授权继续。

后续批次 `T3-R17` 已完成 6/6 次架空 200/300 mm/s 终验，r17 无超时复现，`wheel.filter_alpha=0.10` 在 300 mm/s 配对对照中优于 `0.25`。该批次已正常结束；转入落地测试前必须重新确认物理环境。

## 3.3 停止语义

- “拒绝这次”：取消当前 Arm 或拒绝当前候选，回滚后可继续会话；
- “暂停调参”：停止动作、回滚到最后接受值、保留会话证据；
- “停止调参”或任何危险描述：立即急停，不自动清故障，尽可能回滚，然后结束模式；
- 串口断开、主机异常退出或固件重启：等同“停止调参”，重连后先完成回滚审计，不得直接继续运动。

---

# 4. 调参会话状态机

```text
IDLE
 -> DISCOVER
 -> PREFLIGHT
 -> SNAPSHOT_GOLDEN
 -> BASELINE
 -> PREPARE_CANDIDATE
 -> STAGE_APPLY
 -> ARM
 -> WAIT_HUMAN_CONFIRM
 -> RUN_RECORD
 -> STOP
 -> HARD_GATE
 -> SCORE
 -> ACCEPT | REVERT
 -> SEARCH_NEXT | REPEAT_VALIDATE
 -> FINAL_REVIEW
 -> OPTIONAL_SAVE
 -> REPORT
 -> IDLE

任意运动态 -> ABORT -> EMERGENCY_STOP -> REVERT_AUDIT -> IDLE
```

状态约束：

- `SNAPSHOT_GOLDEN` 完成前不得 Stage 参数；
- `WAIT_HUMAN_CONFIRM` 中 Codex 只能等待人工确认、取消或超时；
- `HARD_GATE` 不通过时不得进入普通评分；
- 每个候选与已接受参数集最多差一个参数；
- `OPTIONAL_SAVE` 必须是最后独立步骤，并再次获得用户明确同意。

---

# 5. 统一预检与 golden profile

## 5.1 Codex 自动预检

1. 只启动一个 M6 dashboard/Session，不让 dashboard、monitor 和串口脚本争抢同一端口。
2. `probe` 通过，固件 ID 和 Schema 与预期一致；重连或重启后必须重做。
3. 参数没有未知 staged 变更；如有，先向用户报告差异，再决定 Apply 或 Revert。
4. SystemState/ActionState 为 IDLE，手动驾驶未占用，Safety fault 为 0。
5. 轮速目标、实测轮速和 PWM 在静止容差内；编码器和必需传感器有效。
6. 20 ms 遥测连续至少 2 s，无 malformed frame、gap、drop、missed task 增量。
7. 当次参数与当前硬件实际限制一致，特别是 PWM 限制 `500`。

## 5.2 操作员物理预检

- `wheel_lifted`：两个动力轮完全离地，车架不会落下/移动，手、线材、摆杆远离转动件；Emm42 支路建议断能。
- `ground_heading`：平整防滑地面，前方空间和两侧逃逸区充足，车体前向为动力轮一侧。
- `line_track`：灰度阵列居中且纵向位置固定，已完成当日白底/黑线检查，赛道无破损、强反光和杂物。
- 所有模式：人员可立即断电，测试区无旁观者和易损物。

## 5.3 golden profile 规则

golden profile 是会话开始时“已在设备上激活且操作员认可”的完整参数快照，不是编译默认值，也不是旧底盘历史记录。

必须记录：

```text
timestamp, scope, operator
git_sha, dirty_worktree
firmware_id, schema, boot/session identity
active_revision, stage_revision, generation, source, storage_health
all active parameters
hardware: wheel diameter, left/right CPR and signs, PWM limit
test condition: lifted/ground, surface/track, battery state, direction
```

会话中每接受一个候选，更新 `accepted profile`，但不覆盖最初 `golden profile`。

---

# 6. 候选生成与搜索算法

## 6.1 总原则

- LLM 只做编排、异常解释和报告；数值候选必须由固定公式、拟合或确定性搜索产生。
- 调参顺序是：机械/传感器 -> 前馈 -> 轮速 PI -> 滤波/斜坡 -> 航向 -> 循迹 -> 任务速度。
- 每次只调一个参数；拟合产生的 `kS+kV` 可作为同一前馈模型原子候选，但必须整组回滚。
- 一个家族最多 3 轮；一整轮没有达到接受门槛就提前停止。

## 6.2 轮速前馈

架空分别采集 `100/160/220/300 mm/s`，每点至少包含最后 1 s 稳态样本。左右轮分别拟合：

```text
PWM = kS + kV * abs(speed_mm_s)
```

约束：

```text
0 <= kS
0 <= kV <= 5
kS + kV * 400 <= 450
```

`450` 是当前 `PWM_LIMIT=500` 的 10% 闭环修正余量，比冻结参数表的旧 `680` 关系更严。若数据已在某一速度持续饱和，不允许继续拟合，先降低速度或查机械/供电。

仓库现有 `fit_feedforward()` 仍按旧 `680` 关系截断；CAR-TUNE v1 必须在应用它的输出前再执行上述 `450` 后置约束，不得直接 Apply 未后处理的拟合值。

前馈候选必须在全部 4 个速度复测，不用单速度评分代替多点证据。

## 6.3 PI、滤波和斜坡

默认坐标步长：

| 参数 | 候选步长 |
|---|---:|
| `wheel.left/right.kp` | `max(abs(value)*10%, 0.05)` |
| `wheel.left/right.ki` | `max(abs(value)*10%, 0.25)` |
| `wheel.filter_alpha` | `max(abs(value)*10%, 0.05)` |
| `wheel.accel/decel` | `max(abs(value)*10%, 50)` |
| `wheel.jerk` | `max(abs(value)*10%, 500)` |

搜索使用有方向的单坐标爬山：

1. 在已接受值上先测试 `+step`；
2. 达到接受门槛则保留，该坐标当轮不再测反方向；
3. 未达标则回滚，再测 `-step`；
4. 两个方向都未达标就保留原值；
5. 下一轮从新的已接受值重算步长，最多 3 轮。

先用单轮动作隔离左右 PI，再用双轮动作终验。滤波和斜坡在 PI 后调，不用过度慢的加速斜坡掩盖振荡。

## 6.4 航向与循迹

- 航向先调直行 `straight.kp/kd`，再调 `turn.kp/kd`、`rate_alpha`、限幅和 smooth zone。
- 直行默认从 `100 mm/s` 开始，通过后再到 `200/300 mm/s`；正负 90° 转向必须对称复测。
- 循迹进入前必须完成灰度标定和通道方向检查。当前左/右位置符号已正确，**自动调参不得交换灰度左右或反转位置符号**。
- 循迹参数顺序为：位置/微分滤波 -> `line.kp` -> `line.kd` -> correction limit/slew -> curve gain -> speed min/max。
- 循迹先在 `100 mm/s` 跑短段、直线和单弯，再跑整圈；不在整圈中同时调航向和循迹。

---

# 7. 录制、硬失败与评分

## 7.1 录制合同

每个动作必须在 Confirm 时联动录制，默认周期 `20 ms`。记录必须包含动作前预滚、运动段、减速段、事件和参数快照。

下列任一条触发硬失败：

- 动作未实际开始或遥测缺失/超时/格式错误；
- 任何 `fault_bits != 0`；
- missed task、UART RX/TX drop、telemetry drop 出现增量；
- 相邻遥测间隔大于 `max(100 ms, 3*period)`；
- 车轮 PWM 连续饱和达 `300 ms`；
- 循迹中不可用线状态持续约 `1.6 s`，或固件更早报告丢线/超时；
- 人员急停、断电、报告异响或结构移动。

硬失败的处理顺序固定为：急停 -> 标记本次失败 -> 回滚 -> 保存证据 -> 结束当前搜索家族。不自动清故障并重试。

## 7.2 软指标

使用仓库现有指标和权重：

```text
score = rise_time_s
      + 4 * overshoot_ratio
      + 6 * steady_error_ratio
      + 0.05 * error_zero_crossings
      + 5 * saturation_ratio
      + yaw_rms_deg / 10
      + 2 * line_position_rms
      + correction_jerk_rms / 1000
      + 10 * line_loss_events
      + 20 * transport_or_scheduler_errors
```

低分更好。不同固件、Schema、测试模式、目标速度或地面条件的记录不直接比分。

## 7.3 候选接受规则

- 普通单参数候选：无硬失败且 score 相对当前 accepted profile 改善至少 `5%`。
- 前馈候选：必须完成 `100/160/220/300 mm/s` 全点证据，聚合 score 改善至少 `10%`，且最大超调和饱和比不得比基线差。
- 初选通过不等于终验通过。最终候选与 golden 必须在相同条件下各重复 3 次：3/3 无硬失败，中位 score 仍达到上述改善门槛，最差的超调/饱和/稳态误差不退化。
- 若 3 次的 score 极差超过中位数的 `15%`，结论标为“证据噪声过大”，不保存候选，先查电池、支架、地面和传感器。

---

# 8. 适配当前新底盘的执行顺序

当前硬件基线：轮径 `65.0 mm`；物理左轮为原始 B，`736.4 count/rev`、前进计数为正；物理右轮为原始 A，`732.6 count/rev`、前进计数为负；动力轮一侧为车头；PWM 限制 `500`。循迹为当前 OUT.8IR.B 数字掩码链路（PA24/SDA、PA26/SCL），位置左/右符号已确认正确。

| 门 | 条件/动作 | 主要产出 | 出口条件 |
|---|---|---|---|
| A0 录制适配器（已通过） | 把 dashboard `core/wheel/line/health` 多流按设备时间合并为评分行 | 可归档的 CSV/JSON、动作元数据和硬失败 | 已完成：软件回归与三个 T1 实车 session 交叉验证通过，gap/reset/abort 不丢失 |
| T0 工具演练 | `REPLAY`，使用历史文件 | 评分器和报告链路可用 | 不连车也能生成一份完整决策报告 |
| T1 新底盘预检（已通过） | 架空，100 mm/s，左/右/双轮 | 映射、符号、计数、Safety 证据 | 已完成：无反向加速、编码器无效、饱和或堵转 |
| T2 前馈 | 架空，100/160/220/300 | 左右 `kS/kV` 拟合和复测 | 多点接受规则通过 |
| T3 轮速环 | 架空，先单轮后双轮 | PI、filter、accel/decel/jerk | 100/200/300 mm/s 均无硬失败，3 次终验通过 |
| T4 落地标定 | 直线 1 m，100 mm/s 开始 | 负载滚动轮周、自然偏航 | 1 m 误差不超 1%；否则只修正轮径，不改 CPR |
| T5 航向 | 100 -> 200 -> 300 mm/s，正负转向 | 直行与转向参数 | 直行、+90°、-90° 各 3/3 通过 |
| T6 OUT.8IR.B/循迹 | 物理按键标定和八路资格验证，100 mm/s 短段到整圈 | 循迹 PD、限幅和速度包络 | 整圈无丢线/安全故障，然后才提速 |
| T7 任务调参 | 按完赛 SPEC 的 R0-R6 | H2-H6 任务指标 | 使用完赛 SPEC 的重复通过率，不用单次最好成绩 |

A0/T1 已于 2026-07-30 通过，下一门为 T2。T2-T5 不等待循迹附加资料，可继续人工验证。T6 可使用当前已接入的 OUT.8IR.B 进行，但必须先按 `docs/灰度传感器自动标定操作手册.md` 完成实物标定、八路资格验证和方向 sanity check；只验证方向，不自动交换左右。

---

# 9. 高效运行策略

1. **单一串口会话**：Codex 维持同一个 dashboard/Session，用其 HTTP API 完成参数和 Arm，操作员只需在网页看状态并 Confirm，不在多个终端复制数据。
2. **先拟合后搜索**：`kS/kV` 用 4 点线性拟合，不用网格穷举；PI 用有方向的坐标搜索和提前停止。
3. **基线复用有边界**：同一会话、同一速度、同一支撑/地面下可复用最近 accepted baseline；更换电池、地面、轮胎、固件或发生重启后必须重录基线。
4. **不重复已证明的测试**：新底盘的原始 A/B 映射已通过时，每会话只做低速 sanity check，不重跑完整原始电机测试。
5. **内外循环分离**：轮速不稳定不调航向，航向不稳定不把偏航归因于循迹，循迹未通过不做车球联合高速测试。
6. **时间箱**：每个参数家族默认最多 3 轮；无改善就输出“参数无证据需修改”，不在实车上无限尝试。

---

# 10. 证据目录与报告

每次会话使用独立目录：

```text
docs/m6-data/YYYY-MM-DD/<scope>-<session-id>/
  session.json              # 身份、硬件、环境、授权范围
  golden.json               # 初始完整参数快照
  accepted.json             # 当前已接受参数
  candidates/
    <sequence>-<key>-<value>/
      manifest.json
      telemetry.*
      events.*
      metrics.json
      decision.json
  validation/
  final.json
  report.md
```

如使用 dashboard 原生 session 目录，保留其 `manifest/schema/events/summary` 结构，Codex 只在上层增加索引和决策文件，不改写原始遥测。

A0 必须使用 `device_time_ms`、stream sample id 和 gap 标志合并 `core/wheel/line/health`，复用 V2 `Recorder` 的字段映射；不得仅按主机到达顺序硬拼。任何 device reset、incomplete session、action abort、sample gap 或录制不完整必须进入硬失败。

A0 命令入口如下；两种入口都只读原始 session，不连接串口、不启动动作：

```powershell
# 直接读取 dashboard session 目录并评分
m6-tune score <dashboard-session-dir> --mode wheel --speed 100

# 需要归档或与旧工具互操作时，导出稳定的合并 CSV/JSON
m6-tune adapt-dashboard <dashboard-session-dir> <output-base> --mode wheel --speed 100

# 候选与同条件 baseline 比较
m6-tune score <candidate-session-or-json> --baseline <baseline-session-or-json>
```

动作 manifest 已携带实际 Arm spec 时，`--mode` 和 `--speed` 可省略；显式参数仅用于补救旧记录或做审计复算。`adapt-dashboard` 返回码 `0` 表示无硬失败，`3` 表示记录可解析但存在硬失败，`2` 表示格式或输入错误；生成的派生文件不得覆盖原始 dashboard session。

`report.md` 必须回答：

- 本次调了什么，哪些参数明确未动；
- golden -> final 的完整差异；
- 每个候选的接受/拒绝原因；
- 硬失败、人工干预和异常回滚记录；
- 3 次终验的中位/最差指标；
- 参数是仅 RAM Apply、已写 Flash，还是因回滚失败而处于 pending 状态；
- 下一个可安全进入的门。

---

# 11. 保存和封版

满足以下全部条件时，Codex 才可提议 Save：

- 没有 Safety fault、pending rollback、未解释的 drop/gap 或 staged 差异；
- 同条件 3 次终验全部通过；
- 轮速参数已完成关键速度包络，循迹参数已完成短段和整圈；
- final 参数和证据文件已写入工作区；
- 用户在看到参数 diff 后明确回复同意保存。

Save 后立即重启/重连核对 active source、generation 和全量参数，再做一次低风险 sanity run。若封版，还需将 final 参数、固件 ID 和完赛 SPEC 的验收 run 绑定。

---

# 12. `CAR-TUNE/AUTO` 的启用门槛

以下条件未全部满足前，不得将本模式宣称为无人全自动：

1. 新增固件可验证的、有有效期的主机运动授权，绑定 action kind、最大速度、最大时长、最大次数和随时撤销；
2. M6 Registry 显式协商出对应能力，不用隐藏指令绕过 `host_auto_confirm=false`；
3. 完成自动授权、过期、断线、重启、乱序和故障注入测试；
4. 有可采信的电池电压/供电状态遥测，可把供电差异从参数差异中分离；
5. 对滚球调参，车辆与 Emm42 已有统一的急停、超时、限位和故障锁存；
6. 有不依赖 LLM 主观判断的确定性运动调度器、评分器和回滚审计。

这些属于赛后或 H1-H6 已通过后的增强项，不是 2026-08-01 前的必做项。

---

# 13. 第一次落地验证脚本

第一次使用本模式时，不立即搜索参数，先验证流程：

1. 运行 `REPLAY`，读取一组正常轮速记录和一组动作未启动记录，验证 score 和硬失败可重现；不将其分数当新底盘基线。
2. A0 软件与 CLI 回归已完成；用首个真实 dashboard session 核对合并行、动作 spec、metrics、停止段和 hard failures，原始 session 保持只读。
3. 连接新底盘，只做 2 s 无运动遥测预检并导出 golden profile。
4. 架空车轮，用 `100 mm/s`、约 7 s 做左轮、右轮、双轮各 1 次；三次都由操作员单独 Confirm。
5. 人工观察结果与遥测符号一致，检查 A0 产出的有效样本、动作事件、停止段和报告。
6. 主动拒绝一个无改善的测试候选，确认参数回到 accepted profile。
7. 不写 Flash，结束会话并重连，确认设备仍保持原有持久化参数。

2026-07-30 状态：上述 7 步已全部通过，证据见 `docs/2026-07-30-CAR-TUNE-A0录制适配器验证记录.md`。`CAR-TUNE/SUPERVISED` 已可用于本车，下一阶段进入 T2 前馈调参。
