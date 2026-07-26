# M6 V2 wire registry v1

状态：2026-07-26 冻结基线，名称 `m6-v2-wire-v1`。机器可读唯一事实源是 `Tests/vectors/m6_v2_wire_v1.json`，本文件是人类可读解释；两者冲突时必须先修正 JSON、golden vectors、本文、`STATE.md` 和日期证据，再推进 registry revision，禁止让 C 与 Python 各自猜测。

当前 MCU 仍运行正式 r12。待上板 r13 只完成 R2 热路径清理，不宣告 V2 capability；R3 的离线 codec/队列也不得在 R4 双协议、安全协商和共享业务服务完成前设置 `WIRE_R1`。

## 1. 基本编码与 parser

- 多字节整数均为 little-endian；有符号整数使用二进制补码。
- `f32` 为 IEEE-754 binary32 little-endian；请求只接受 finite 值。
- `bool8` 只接受 `0/1`。ASCII 字符串由显式长度字段界定，不接受 NUL 或非 ASCII。
- reserved 字段发送时必须为 0，接收非 0 视为 `MALFORMED`；不得直接发送 C struct。

| 常量 | 值 |
|---|---:|
| `WIRE_VERSION` / `REGISTRY_REVISION` | `2 / 1` |
| `FRAME_DELIMITER` / `FRAME_ESCAPE` / XOR | `0x7E / 0x7D / 0x20` |
| `MUST_ESCAPE` | `0x20, 0x53, 0x7D, 0x7E` |
| `HEADER_SIZE` / `CRC_SIZE` | `13 / 2 B` |
| `MAX_PAYLOAD` / `MAX_RPC_REQUEST_PAYLOAD` | `96 / 32 B` |
| `MAX_DECODED_FRAME` / `MAX_WIRE_FRAME` | `111 / 224 B` |
| RX 环物理/有效大小 | `256 / 255 B` |
| 每主循环 RX 预算 | `32 wire bytes` |
| interbyte / total timeout | `50 / 100 ms`，`elapsed >= limit` |
| RX 高水位上板目标 | `<192 B` |

未转义帧为 `version:u8, type:u8, flags:u8, session:u32, sequence:u32, payload_length:u16, payload, crc16:u16`。CRC 是 CRC-16/CCITT-FALSE（poly `0x1021`、init `0xFFFF`、refin/refout false、xorout 0），覆盖 version 至 payload，CRC 本身按 little-endian 附加；`"123456789" -> 0x29B1 -> B1 29`。

头、payload 和 CRC 中四个保留字节全部转义，所以合法帧内部没有裸空格或裸 `S`。UART RX ISR 仍在入环前消费裸 `S`/空格、请求 Safety 急停并递增 emergency epoch；codec 不拥有急停语义。

parser 状态为 `HUNT/COLLECT/ESCAPED/DISCARD`，冻结以下边界：

1. delimiter 结束当前候选并同时成为下一个候选的 opening delimiter；空 `COLLECT` 不启动超时，重复 delimiter 保持空 `COLLECT` 且不计错。编码器仍发每帧独立首尾 delimiter，不依赖共享 delimiter。
2. 第一枚内部字节启动 total/interbyte 计时。每次处理新字节前先按无符号 32 位差值检查超时；同时到期只计 total，随后才把当前字节按新状态处理。
3. escape 后只允许 `00/73/5D/5E`；其他后继或 ESCAPED 中 delimiter 记一次 escape error，delimiter 同时开启新的空候选。
4. wire 计数含两端 delimiter；encoded body 最多 222 B。若新内部字节已使 224 B 内无法再容纳 closing delimiter，立即进入 DISCARD。decoded 最多 111 B。
5. DISCARD 中只等 delimiter；delimiter 恢复为空 `COLLECT`。DISCARD 仍受 total/interbyte timeout：每个被丢弃的非 delimiter 字节先做同一 timeout 检查，再只刷新 interbyte 的 last time，total 起点不变；到期或 epoch 变化直接回 HUNT。
6. 物理检查顺序固定为容量、最小 15 B/头中长度、CRC，再依次进入 version、flags、session、type 语义。CRC 正确但 `version!=2` 时，`V2_ARMED` 清候选并进入 `V1_RESYNC`；`V2_ESTABLISHED` 静默丢弃、递增 `frame_error` 并保持 session，绝不返回 `PROTOCOL_ERROR`。物理解码长度不符静默丢弃；可信 session 内的 type-specific payload 长度错才可限频返回 `BAD_LENGTH`。
7. CRC、escape、截断、噪声、overflow、timeout 和 session mismatch 都只计数丢弃；残余字节永不回放给 V1。RX overflow epoch 变化只清 parser 候选并计数，不取消已经 admit 的 apply，也不改变业务结果。emergency epoch 变化立即清候选和 P2/P3；急停或 Safety fault 才按第 4 节取消 pending apply 并缓存确定结果。

## 2. frame、flags、capability

| Type | 值 | 方向 | 说明 |
|---|---:|---|---|
| `HELLO` | `0x01` | H→D | 仅 `V2_ARMED` |
| `READY` | `0x02` | D→H | HELLO 结果 |
| `RPC_REQUEST` | `0x10` | H→D | established 控制请求 |
| `RPC_RESPONSE` | `0x11` | D→H | 回显主机 sequence |
| `PROTOCOL_ERROR` | `0x12` | D→H | 可信语义错误，限频 |
| `EVENT` | `0x20` | D→H | P0/P1 事件 |
| `TELEMETRY` | `0x30` | D→H | P2，stream 前缀 |
| `BULK` | `0x31` | D→H | P3，显式 chunk |

`REPLAY=0x01` 只允许缓存的 `RPC_RESPONSE`；`GAP_BEFORE=0x02` 只允许 `EVENT/TELEMETRY/BULK`。`KNOWN_FLAGS_MASK=0x03`，还必须逐 type 拒绝非法组合。

capability mask 是 u64，JSON/V1 使用固定 16 位小写十六进制字符串；单个低位 capability 注册值可用精确整数：

| Bit | 名称 | Bit | 名称 |
|---:|---|---:|---|
| 0 | `WIRE_R1` | 12 | `LINE_TRACE_READ` |
| 1 | `RAW_ESTOP_S_SPACE` | 13 | `LEGACY_V1` |
| 2 | `SESSION_DEDUP` | 14 | `HANDSHAKE_RESYNC` |
| 3 | `PRIORITY_TX` | 15 | `FAULT_CLEAR` |
| 4 | `RESET_DIAG` | 16 | `WHEEL_DIAG_ACTION` |
| 5 | `HEALTH_STATS` | 17 | `HEADING_ACTION` |
| 6 | `PARAM_READ` | 18 | `LINE_ACTION` |
| 7 | `PARAM_STAGE` | 19 | `MOTOR_SELF_TEST` |
| 8 | `PARAM_BOUNDARY_APPLY` | 20 | `HEADING_CALIBRATION` |
| 9 | `PARAM_PERSIST` | 21 | `GRAYSCALE_CAPTURE` |
| 10 | `TELEMETRY_SUBSCRIBE` | 22 | `CONFIGURABLE_TURN` |
| 11 | `ACTION_TWO_PHASE` | 23 | `BULK_TRANSFER` |
| 24 | `HEADING_RATE_ACTION` |  |  |

进入 V2 的最低 required mask 为 bits 0–3、13、14，即 `0x000000000000600F`；`LEGACY_V1/HANDSHAKE_RESYNC` 是 rev1 握手失败和干净关闭的必需恢复语义，不得一边省略 capability 一边依赖该路径。功能未实现、未测试或未接入 Safety 时不得提前置位；一个粗粒度 `ACTION_TWO_PHASE` 位也不代表所有 action kind 都可用，具体 kind 还受对应 capability 约束。

## 3. V1 协商、HELLO 与 session

设备模式：`V1 -> V2_DRAIN -> V2_ARMED -> V2_ESTABLISHED`；握手失败或干净关闭后的隔离态为 `V1_RESYNC`。主机先用现有事务核对 firmware/schema/revision/source/generation，再查：

```text
:41,get,meta.capabilities\r\n
ok,41,meta.capabilities,<16位小写十六进制u64>\r\n
```

只有 IDLE、无故障、无动作、无 pending apply/Flash 且具备 required mask 才写一次：

```text
:42,mode,v2,0123456789abcdef\r\n
ok,42,v2-ready,0123456789abcdef,2,1,96,32,1\r\n
```

challenge 为非零随机 u64、固定 16 位小写 hex。设备接受 mode 后立刻隔离旧非急停命令并冻结所有 V1 producer：已开始发送的记录和已受理的事务回复继续物理排完，尚未开始的 telemetry/诊断/G6/help 整条取消并记 legacy drop；随后上面的 ready 作为最后一个 V1 frame。最后五项依次为 wire、registry、max payload、max RPC request、window，ready 物理完成才进入 `V2_ARMED`。排空判据必须同时覆盖上层待提交记录、旧 FIFO、UART FIFO 和 `DL_UART_isBusy()`，不能只看 `debug_tx_idle()`。

`HELLO` payload 固定 40 B：

```text
0  hello_magic:u32 = 0x3256364D；线上字节 4D 36 56 32（ASCII "M6V2"）
4  registry_min:u16
6  registry_max:u16
8  challenge:u64
16 required_capabilities:u64
24 optional_capabilities:u64
32 host_max_payload:u16
34 host_max_rpc_request:u16
36 host_rx_window:u8 (=1)
37 reserved[3]
```

`READY` payload 固定 64 B：

```text
0  ready_magic:u32 = 0x59443252；线上字节 52 32 44 59（ASCII "R2DY"）
4  selected_registry:u16
6  result:u16
8  detail:u32
12 challenge_echo:u64
20 assigned_session:u32
24 boot_id:u32
28 raw_reset_cause:u32
32 session_generation:u32
36 parameter_schema:u16
38 reserved[2]
40 device_capabilities:u64
48 enabled_capabilities:u64
56 selected_max_payload:u16
58 selected_max_rpc_request:u16
60 device_rx_window:u8 (=1)
61 reserved[3]
```

两者 header 都是 `session=0, sequence=0`。主机 `required_capabilities` 必须至少包含固定 `HANDSHAKE_REQUIRED=0x000000000000600F`；required 与 optional 必须互斥，重叠返回 `INVALID_ARGUMENT/detail=最低重叠 bit`。固定 required 缺位返回 `UNSUPPORTED_CAPABILITY/detail=最低缺失固定 bit`；required 中设备不支持或未知的 bit 也返回 `UNSUPPORTED_CAPABILITY/detail=最低缺失/未知 bit`，optional 未知 bit 只在交集中忽略。成功 READY 的 session 必须非零，enabled 为 device 与 `(required|optional)` 的交集且必须包含全部 required。session 使用 CRC-32/ISO-HDLC 覆盖 `ASCII("M6V2SESS") || challenge_le64 || boot_id_le32 || raw_cause_le32 || session_generation_le32 || parameter_schema_le16`；结果 0 映射为 1。它只隔离误连，不是认证。一次 mode challenge 只允许一次 HELLO、只分配一次 generation/session；READY 丢失不在 established 中重放，主机报告握手结果未知并要求物理复位。

registry revision 1 只在 `registry_min<=1<=registry_max` 时选择 revision 1，且不做可变长度降级：HELLO 的 `host_max_payload` 必须至少为 96，`host_max_rpc_request` 必须至少为 32，设备自身也必须支持这两个固定上限；否则不建立 session。成功 READY 的 selected 值固定为 `1/96/32/window=1`。这样 64 B READY、84 B RESET/LINK response 与全部冻结业务消息都不会超过主机声明的接收能力。

失败 READY 仍回显 challenge，并可报告 boot/raw/generation/schema/device capability；`selected_registry/assigned_session/enabled_capabilities/selected_max_payload/selected_max_rpc_request/device_rx_window` 必须全为 0，物理发送完成后进入 `V1_RESYNC`。检查优先级固定为 magic、challenge、registry 范围、reserved、window、registry 交集、固定 required mask、required/optional 重叠、设备缺失/未知 required、payload 上限、RPC request 上限；registry 不相交返回 `INCOMPATIBLE_REGISTRY/detail=device registry revision`，capability 和重叠错误使用上一段的最低 bit index，host 上限不足返回 `BAD_LENGTH/detail=(required<<16)|actual`。其他 HELLO 字段错误返回相应冻结 result/detail，绝不部分建立 session。唯一例外是 `host_max_payload<64`：按主机自报能力无法合法发送固定 64 B READY，因此静默进入 resync并计 handshake error。

READY 必须先物理发送完成，设备才建立 session、把积压 P0 事件开放给仲裁。`V2_ARMED` 中 HELLO CRC/escape/长度错误、RX overflow 或 500 ms timeout 都清候选并进入 V1_RESYNC；裸急停先由 ISR/Safety 制动和锁存 fault，再走同一隔离路径，绝不清 fault。`V2_ESTABLISHED` 中同类 frame 错误只丢帧计数并保持 established；裸急停也保持 session，供主机读取 fault/RESET 并执行显式受门控操作。

| 协商常量 | ms/次数 |
|---|---:|
| V1 transaction / TX drain / V1 ready / HELLO | `1000 / 250 / 1000 / 500 ms` |
| resync quiet / probe timeout | `100 / 500 ms` |
| mode / HELLO / resync probe attempts / RPC automatic retries | `1 / 1 / 1 / 0` |
| `V2_AUTO_FALLBACK_MS` | `0` |

只有尚未 established 的 `V2_DRAIN/V2_ARMED` 排空、ready 或 HELLO 失败/超时，或 `SESSION_CLOSE` 最后一帧物理完成后，才能清候选并进 `V1_RESYNC`。drain 超时后先停止继续装填，但仍等待所有已开始字节物理排空；RX/TX 未同时为空时不得识别 probe。连续静默 100 ms 后只识别固定字节串 `:1,get,meta.capabilities\r\n`，任何不匹配的非急停字节都丢弃并重启 quiet，不调用通用旧命令 parser；固定回复 `ok,1,meta.capabilities,<16位小写十六进制u64>\r\n` 物理排空后才恢复通用 V1/旧单字节命令。`V2_ESTABLISHED` 中 CRC/timeout/overflow 永不触发 ASCII fallback。主机崩溃留下 established session 时，新进程要求物理复位；不得盲发二进制或自动发送 `x`。

`V2_DRAIN/V2_ARMED/V1_RESYNC` 的 250/500/100 ms 等待全部是后台状态机的 wrap-safe 计时；5/10/20/100 ms 任务、Safety 和原 WWDT 健康门控始终运行，不得忙等、额外喂狗或暂停调度。若 `SESSION_CLOSE` 的 OK response 已开始发送时发生裸急停，仍把该帧物理发完后进入 `V1_RESYNC`，但 fault、制动和 breadcrumb 完整保留，绝不因 close 清故障。

## 4. sequence、去重与副作用

- HELLO/READY sequence 固定 0。主机 RPC 从 1 严格递增，`0xFFFFFFFF` 保留，host 最后可用值为 `0xFFFFFFFE`，到达前必须重新协商。
- response 回显请求 sequence。EVENT/TELEMETRY/BULK 共用独立 async sequence，从 1 开始，在消息被仲裁选为下一物理帧时才分配，保证异步帧在线上单调；采样覆盖由 per-stream sample ID、`GAP_BEFORE` 和 drop 计数表达。普通异步帧最后可用值为 `0xFFFFFFFD`，`0xFFFFFFFE` 专用于终止 `LINK_STATE(sequence_exhausted)`。当 next 等于 `0xFFFFFFFE` 时，在普通仲裁前进入终止态：RPC result 和不消费 async sequence 的 control-error 仍可先发送，但 P0/P1/P2/P3 均不再 eligible，terminal LINK_STATE 是唯一异步候选。P0 latch 保留，终止帧后可由 RPC 查询最终 reset/Safety/emergency 真值；终止帧直接在唯一 in-flight buffer 中合成，不占 P1、不能因队列拥塞丢失。该 EVENT 固定 flags=0、severity WARNING、occurrence=1，body 为 established/current session/current expected host seq/next async `0xFFFFFFFF`。其物理发送完成后停止全部异步帧但保留 RPC，主机必须干净 close/重建 session，绝不回绕。
- 设备强制 host window=1。检查顺序固定为：先判断是否已有 pending/queued/in-flight 请求；若有，第二个不同请求由 control-error latch 返回 non-consuming `BUSY`，expected 不变；只有无未完成请求时才进入 sequence 比较。
- 四个合并的 replay/RPC-result entry 各保存完整 request payload（≤32 B）和完整 response payload（≤96 B），不用可碰撞 fingerprint。metadata 包含 sequence、两种长度、状态、淘汰顺序、queued/in-flight/replay 标志，`sizeof(entry)<=152`；entry 本身就是最终回复槽，不再另占一份 result buffer。
- 新请求必须先预留一个上述合并 entry；失败由独立 control-error/BUSY latch 回 `BUSY`，不执行也不推进 sequence。主机收到该 non-consuming `BUSY` 后也不得递增 sequence；等待建议时间后可用同一 sequence 发起下一次调用。
- 无未完成请求且 `sequence==expected` 时，预留成功后才推进并执行；确定性业务错误也缓存。较小序号且完整 request 相同：pending/queued/in-flight 时只关联原结果、不另回临时 PENDING；completed 时用同 payload 重编码并置 `REPLAY`。同 seq 不同 bytes 为 `SEQUENCE_CONFLICT`；已淘汰为 `REPLAY_EXPIRED`；较大 seq 为 `SEQUENCE_GAP`。
- 只淘汰最旧 completed，pending 永不淘汰。急停或 Safety fault 在 5 ms apply pending 期间出现时调用既有 cancel，原请求完成为 `SAFETY_FAULT`/`WRONG_STATE` 并缓存，重复请求绝不恢复提交。
- 写入前失败是 `TimeoutBeforeSend`；开始写后无法在同 session 用原 bytes/seq 找回结果是主机 `OutcomeUnknown`。默认不自动重试；显式恢复只能逐字节重发原帧。

`BUSY` 是 RPC response，不是 protocol error；专用 latch 也承载 stale/conflict/可信 protocol error，永不借用已为副作用预留的合并 entry。若该 latch 已占用则只增 `control_error_drop`。

## 5. RPC 与 result registry

request 为 `opcode:u16 + body`，总 payload≤32 B。response 为 `opcode:u16, result:u16, detail:u32 + success body`，总 payload≤96 B；result 非 OK 时没有 success body。

| Opcode | 名称 | Opcode | 名称 |
|---:|---|---:|---|
| `0001` | `PING` | `8000` | `SESSION_CLOSE` |
| `0002` | `IDENTITY_GET` | `8001` | `FAULT_CLEAR` |
| `0003` | `STATUS_GET` | `8100` | `PARAM_STAGE_SET` |
| `0004` | `RESET_GET` | `8101` | `PARAM_APPLY` |
| `0005` | `TASK_STATS_GET` | `8102` | `PARAM_REVERT` |
| `0006` | `UART_STATS_GET` | `8103` | `PARAM_LOAD` |
| `0007` | `LINK_STATS_GET` | `8104` | `PARAM_DEFAULTS` |
| `0008` | `ACTION_GET` | `8105` | `PARAM_SAVE` |
| `0009` | `SUBSCRIPTION_GET` | `8200` | `SUBSCRIPTION_SET` |
| `000A` | `SNAPSHOT_GET` | `8300` | `ACTION_ARM` |
| `0100` | `PARAM_SUMMARY_GET` | `8301` | `ACTION_CONFIRM` |
| `0101` | `PARAM_DESCRIPTOR_GET` | `8302` | `ACTION_CANCEL` |
| `0102` | `PARAM_VALUE_GET` | `8303` | `HEADING_CALIBRATE` |
| `0200` | `TRACE_INFO_GET` | `8304` | `GRAYSCALE_CAPTURE_START` |
| `0201` | `TRACE_CHUNK_GET` |  |  |

高位为 1 的 opcode 有副作用。绝不分配 RPC 急停 opcode。

关键 payload：

- `PING(cookie:u32)` → `cookie, uptime_ms, boot_id`。
- `IDENTITY_GET` → wire/registry/schema、active/stage revision、generation、boot ID、supported capability u64、source、image flags、`firmware_len:u8 + ASCII`；firmware 最长 31 B。
- `STATUS_GET` → time、system/action/kind/wheel enabled、完整 Safety fault/时间/计数/required sensors/deadline、requested/applied PWM、target/measured speed、active/stage revision、source/dirty。required sensor mask 固定 bit0 gyro、bit1 grayscale。
- `RESET_GET` → 第 8 节固定 76 B；加 response prefix 共 84 B。
- `TASK_STATS_GET` → task count/health/pending/missed/window mask 与 4 组 release/completion/miss u32；所有 task mask 固定 bit0/1/2/3 对应 5/10/20/100 ms。
- `UART_STATS_GET` →现有五个 DebugStats u32、emergency epoch、RX/TX queued/high-water u16。
- `LINK_STATS_GET` → mode/window/table、session/next host/next async、frames、CRC/frame/timeout/session/sequence/replay/busy、P0–P3/control-error drop 与四级 high-water。
- `SNAPSHOT_GET(stream_id,reserved[3])` → `stream_id,schema,length,data`，不改变订阅 sample ID。
- `PARAM_SUMMARY/ DESCRIPTOR/ VALUE` 见第 6 节；key 最长 31 B。
- `TRACE_INFO_GET` → 24 B trace info；`TRACE_CHUNK_GET(capture_id:u32, offset:u16, count:u8(1..2), reserved)` 先预留一条 P3，再回 `transfer_id,capture_id,offset,count,record_size`。同 seq replay 只重放 RPC ACK；若 BULK 丢失，主机用新 seq 重取同 offset。
- `SESSION_CLOSE` 仅在 IDLE、无故障/动作/pending/Flash 时成功；response 物理完成才进入 V1_RESYNC。
- `FAULT_CLEAR(boot_id, observed_fault_bits)` 要求当前 FAULT、全制动且观测值精确匹配，仍调用 `Safety_ClearFaults()`，不恢复动作。
- `PARAM_STAGE_SET(expected_stage_revision,param_id,type,reserved,value_bits)`；apply/revert/load/defaults/save 都携带相应 expected active/stage/generation。apply success 与 `PARAM_COMMITTED` 的 change mask 为 u64，bit `param_id-1` 覆盖全部 35 项。fault-free IDLE、无动作的普通 apply 沿用现状立即提交并立即形成最终回复；只有已武装轮速诊断中的 ID 1–4、13 才进入原 5 ms 开头的 pending/commit，最终回复只在该边界真正提交后产生。save 继续要求 IDLE、无故障/动作、wheel disabled、requested/applied/final motor PWM 全 0，并保留 A/B commit-last、读回验证和 WWDT 门槛。
- `SUBSCRIPTION_SET(replace=1,count=0..4,reserved, entries{stream_id,flags=0,period_ms})`；未知或重复 stream 返回 `INVALID_ARGUMENT`，成功回复按 stream ID 升序返回规范化 entries 和 estimated wire B/s；超出 P2 预算整笔拒绝。
- `ACTION_ARM` 固定 24 B：`kind:u8,wheel_mask:u8,flags:u16(=0),target_l:i16,target_r:i16,angle_mdeg:i32,yaw_rate_mdeg_s:i32,duration_ms:u32,expected_active_revision:u32`；response 回显规范化参数并给 nonzero token/10 s expiry。`ACTION_CONFIRM(boot_id,token)` 是独立 RPC，`m6link` 不得自动 confirm。token 绑定 session、arm seq、boot、active revision 与完整动作参数。confirm 时必须重新检查 session/boot/token/expiry、Safety/system/action/sensor、active revision 和全部限值，不能只验证 token。`ACTION_CANCEL(action_id)` 是正常停止，不清 fault。
- `HEADING_CALIBRATE(boot_id)` 和 `GRAYSCALE_CAPTURE_START(boot_id)` 都受 capability/state/sensor 门控。

Action kind：`0 NONE, 1 WHEEL_SPEED_TEST, 2 HEADING_HOLD, 3 HEADING_TURN, 4 HEADING_RATE, 5 LINE_FOLLOW, 6 MOTOR_SELF_TEST`。state：`0 IDLE,1 ARMED,2 RUNNING,3 RAMP_DOWN,4 COMPLETE,5 REJECTED,6 FAULT`。reason：`0 NONE,1 HOST_CANCEL,2 COMPLETE,3 TIMEOUT,4 SAFETY_FAULT,5 EMERGENCY,6 SENSOR,7 REJECTED`。这些是新 wire 枚举，R4 必须通过 action manager 映射现有诊断状态，禁止强转当前 C enum。所有动作 `duration_ms` 必须非零、最大 30000 ms，并用 wrap-safe deadline；所有速度目标仍受 ±700 mm/s 最终硬限，任何未实现 kind 返回 `UNSUPPORTED_CAPABILITY`。

逐 kind 的 rev1 规范化/门控为：WHEEL_SPEED_TEST 需要 capability 16，`wheel_mask` 只能为 1–3，未选轮目标必须为 0、已选轮目标必须落在 `[-400,-50]∪[50,400] mm/s`，angle/rate 为 0，duration 100–10000 ms；HEADING_HOLD 需要 capability 17，mask=3、左右目标相等且落在同一非零速度域，angle/rate 为 0，duration 100–30000 ms；HEADING_TURN 需要 capability 17，mask=3、左右目标/rate 为 0、angle 的绝对值为 1000–360000 mdeg、duration 500–30000 ms，没有 capability 22 时只接受 ±90000 mdeg；HEADING_RATE 还必须有 capability 24，mask=3、左右目标/angle 为 0、rate 的绝对值为 1000–360000 mdeg/s，duration 100–30000 ms；LINE_FOLLOW 需要 capability 18，mask=3、target/angle/rate 全 0，duration 100–30000 ms；MOTOR_SELF_TEST 需要 capability 19，mask=3、target/angle/rate 全 0、duration 2500–5000 ms，只运行编译期固定且经 Safety 的电机自检序列，不开放 raw PWM。NONE、未知位或不适用字段非零均拒绝。

result u16 冻结为：

```text
0000 OK                 0001 MALFORMED          0002 BAD_FLAGS
0003 BAD_LENGTH         0004 UNSUPPORTED_TYPE   0005 UNSUPPORTED_OPCODE
0006 INCOMPATIBLE_REGISTRY 0007 UNSUPPORTED_CAPABILITY
0008 SESSION_REQUIRED   0009 SESSION_MISMATCH   000A SEQUENCE_ZERO
000B SEQUENCE_GAP       000C SEQUENCE_CONFLICT  000D REPLAY_EXPIRED
000E SEQUENCE_EXHAUSTED 000F BUSY               0010 SESSION_CLOSING
0100 INVALID_ARGUMENT   0101 UNKNOWN_PARAMETER  0102 READ_ONLY
0103 INVALID_VALUE      0104 OUT_OF_RANGE        0105 RELATION
0106 STALE_ACTIVE_REVISION 0107 STALE_STAGE_REVISION 0108 STALE_GENERATION
0109 WRONG_STATE        010A SAFETY_FAULT        010B ACTION_ACTIVE
010C NOT_ARMED          010D ARM_EXPIRED         010E SENSOR_NOT_READY
010F UNSUPPORTED_RATE   0110 BANDWIDTH_EXCEEDED  0111 NOT_FOUND
0200 STORAGE_IO         0201 STORAGE_VERIFY      0202 STORAGE_SCHEMA
7FFF INTERNAL
```

`detail`：BAD_LENGTH 为高 16 expected/低 16 actual；unsupported 为相应 type/opcode；sequence gap 为 expected；conflict/replay 为请求 seq；参数错误低 8 位 param ID；stale 为设备当前 revision/generation；WRONG_STATE 低 8 system/次低 8 action；SAFETY_FAULT 为完整 mask；BUSY 高 8 resource ID、低 24 建议等待 ms。resource ID：`1 rpc_entry,2 control_error,3 bulk,4 param_boundary,5 flash,6 action,7 tx_drain,8 async_event`。

`PROTOCOL_ERROR` payload 固定 8 B：`result:u16, offending_type:u8, offending_flags:u8, detail:u32`；header 使用当前 session并回显 offending sequence，不消费 RPC sequence。该 frame 只允许 `MALFORMED/BAD_FLAGS/BAD_LENGTH/UNSUPPORTED_TYPE` 四种 result；BUSY、sequence、stale 和业务结果都走 RPC response。session mismatch、CRC 或无法信任 header 时静默。方向错误按 unsupported type；未知 type detail 为其数值。每 100 ms 最多一条。

## 6. Parameter registry 与事务

wire value type：`0 INVALID,1 F32,2 BOOL,3 U32,4 ENUM_U8`；source：`0 DEFAULTS,1 PERSISTED,2 RUNTIME`；storage slot：`0 NONE,1 A,2 B`；health：`0 OK,1 DEGRADED`。展示单位固定为 `0 NONE,1 TUNING_GAIN,2 RATIO,3 MM_PER_S,4 MM_PER_S2,5 MM_PER_S3,6 PWM,7 PWM_PER_MM_PER_S,8 DEGREE,9 BOOL`；`TUNING_GAIN` 刻意不伪造不同控制器增益的物理量纲。relation group 固定为 `0 NONE,1 LEFT_FF,2 RIGHT_FF,3 LINE_SPEED,4 REMOTE_SPEED`。

参数 ID 固定为当前 descriptor 顺序 `1..35`，永不复用，新增只追加。完整 key/min/max/default/unit 位型在 JSON。分组为：

- 1–13：`wheel.left.kp/ki`, `wheel.right.kp/ki`, filter、accel/decel/jerk、左右 ks/kv、`wheel.diag_target`；
- 14–21：heading straight/turn kp/kd、rate alpha、correction limit、turn speed、smooth zone；
- 22–31：line speed min/max、position/derivative alpha、kp/kd、correction limit/slew、curve gain、lost search speed；
- 32–35：remote low/medium/high、heading hold。

1–34 为 F32，35 为 BOOL。全部 writable/persistent；只有 1–4、13 可在轮速诊断 5 ms 边界提交。descriptor flags：bit0 writable、bit1 diagnostic-boundary、bit2 persistent、bit3 cross-field relation。

关系组：9/10 满足 `ks+kv*400<=680`，11/12 同理，22/23 满足 min≤max，32/33/34 单调。ID13 的域不是连续 min/max，而是 `[-400,-50] ∪ {0} ∪ [50,400]`，domain rule 固定为 `DIAG_SPEED_GAP`。最终轮目标仍硬限幅 ±700 mm/s。

R4 新增 RAM-only `stage_revision`，启动为 1；成功 set/revert/load/defaults 和 apply 后递增，不进入 Flash schema。`PARAM_DESCRIPTOR_GET` 返回 id/type/flags/key length、min/max/default bits、domain rule、relation group、unit 和 key；F32/BOOL/U32 的 active/staged/value 均按 type 解释原始 u32 LE bits。`PARAM_VALUE_GET` success 的 byte 24 是固定为 0 的 `reserved_flags`，byte 25–27 也保留为 0；rev1 不从该字节推导 dirty/pending 状态。

现有边界不变：staging 不改 active；pending apply 时其他 staging 返回 BUSY；普通 apply 仅 fault-free IDLE 且所有动作 idle；诊断 apply 只允许 1–4、13 并在 5 ms 开头提交；急停/fault 取消 pending；save 只能走共享业务服务，不得直接调用底层 Flash writer 绕过全制动和 WWDT 门槛。

## 7. Telemetry、EVENT 与 BULK

TELEMETRY payload 前缀固定 8 B：`stream_id:u8, schema:u8(=1), reserved:u16, sample_id:u32`。每流 sample ID 在整个 session 内从 1 单调递增，不因 disable→enable 重置；只有订阅实际采样才递增，disabled 不采样、不制造假 gap。替换订阅时必须在 success response 前原子清除受影响流的旧 P2 slot，但保留其 next sample ID。frame async sequence 在 TX 选中时分配。stream：`1 CORE,2 WHEEL,3 LINE,4 HEALTH`。

数值从 float 转整数时先按字段 scale、half-away-from-zero，再饱和到目标整数。固定 body（不含 8 B 前缀）：

- CORE 32 B：time/revision u32；requested L/R、filtered speed L/R、PWM L/R i16；yaw cdeg i16；yaw rate cdeg/s i32；saturation bits u8、rate source u8、fault bits u32。rate source `0 none,1 angle_difference,2 gyro_frame`；saturation bit0/1 左右 PI，bit2 line correction limited/slewed，bit3 wheel target hard-clamped。
- WHEEL 52 B：time u32；raw/filtered/error/ramp 为 mm/s i16，P/I/FF 项为 PWM i16，最终 PWM L/R i16；sat_ms L/R、dt_us、flags u16；本次 encoder delta count L/R i32。u32 source 缩到 u16 时饱和。flags bit0 enabled、1 sample valid、2/3 左右 saturated、4 encoder-invalid、5 stall，其他 0。
- LINE 48 B：time u32、ADC[8] u16、active mask/count/cluster、line state、app state、search direction、flags/reserved、raw/filtered position x1000 i16、derivative x1000 i32、confidence x1000 u16、dynamic base/correction/targets i16、lost_ms u16 saturation。active mask 的 bit i 对应 grayscale channel i；search direction 只允许 `-1 negative side / 0 none / +1 positive side`，不把符号擅自翻译成物理左右；line state `0 idle,1 tracking,2 searching,3 lost_timeout`；app state `0 idle,1 armed,2 running`；flags bit0 valid、1 ambiguous、2 branch latched、3 scan OK、4 calibration valid、5 compiled verified、6 searching、7 target clamped。
- HEALTH 64 B：time/boot/raw cause/fault u32；system state/reset class/checkpoint/health flags u8；task miss[4] saturating u16；UART tx/rx drop、legacy format、legacy telemetry drop、diag100 drop、P2/P3 drop u32；RX/P0/P1/P2/P3 high-water、telemetry queue u8、reserved u16。RX high-water 的单位是 queued wire bytes（0..255），P0–P3 high-water 的单位是各队列同时占用的 slot 数（分别不超过 3/4/4/2）。health flags bit0 scheduler healthy、1 gyro valid、2 grayscale OK、3 line valid、4 storage degraded、5 reset detail valid、6 legacy format nonzero、7 V2 drop nonzero。

CORE/WHEEL period 20–1000 ms 且 20 的倍数；LINE 40–1000 ms 且 20 的倍数；HEALTH 200–5000 ms 且 100 的倍数。订阅默认全关。admission 的 `estimated_bps` 固定按无转义基线计算：每流 `ceil(1000/period_ms) × (2 delimiters + 13 header + payload + 2 CRC)`，四流求和超过 7000 B/s 时整笔拒绝；例如 CORE+WHEEL 都为 20 ms 时是 `(57+77)×50=6700 B/s`。实际发送仍按含 escape 的真实 wire length 扣 7000 B/s token，超过时丢 P2、置 GAP 并计数；估算值不冒充最坏情况保证。

EVENT common 12 B：`event_code:u16,severity:u8,event_schema:u8,time_ms:u32,occurrence_count:u16,body_length:u8,reserved:u8`。rev1 的 code 1–9 均要求 `event_schema=1`；主机不得猜测未知 schema 的 body。occurrence 是当前 latch 合并次数，1 起、u16 饱和；合并后置 GAP，物理完成后下一条重新从 1。severity：`0 INFO,1 WARNING,2 ERROR,3 CRITICAL`。

| Code | 名称/队列 | body |
|---:|---|---|
| 1 | `RESET` P0 | 32 B：boot/raw/previous time/last WWDT/fault/miss，class/current cp/flags/previous cp/boot8/detail cp/state/health |
| 2 | `SAFETY_FAULT` P0 | 36 B：fault times/count/sensors/deadline、system/action、requested/applied PWM |
| 3 | `EMERGENCY_STOP` P0 | 24 B：epoch/fault、system/action/source、targets、requested/applied PWM |
| 4 | `ACTION_STATE` P1 | 28 B：id、state/kind/reason/source、start/deadline、targets、angle/rate |
| 5 | `PARAM_COMMITTED` P1 | 20 B：active/stage revision、u64 change mask、source/boundary |
| 6 | `TELEMETRY_GAP` P1 | 12 B：stream/reason、drop delta、latest sample ID |
| 7 | `STORAGE_FAULT` P1 | 12 B：result/slot/health/generation/save failures |
| 8 | `LINK_STATE` P1；sequence 耗尽终止帧为仲裁器直属 | 16 B：mode/reason/session/next host/next async |
| 9 | `GRAYSCALE_READY` P1 | 40 B：request/scan ID、ADC[8]、error/status/flags、state flags |

这些 body 的其余数值表同样由 JSON 冻结：action source 为 `NONE/V1_UART/V2_RPC/PB11_IR/BOOT_SELF_TEST/INTERNAL=0..5`；emergency source 为 `UNKNOWN/UART_S/UART_SPACE/INTERNAL=0..3`；gap reason 为 `QUEUE_OVERWRITE/TOKEN_BUDGET/PRODUCER_DROP/EMERGENCY_FLUSH/SESSION_TEARDOWN=1..5`；link mode 为 `V1/V2_DRAIN/V2_ARMED/V2_ESTABLISHED/V1_RESYNC=0..4`，link reason 为 `NONE/MODE_REQUEST/READY_SENT/HELLO_OK/PRE_ESTABLISHED_FAILURE/SESSION_CLOSE/EMERGENCY/SEQUENCE_EXHAUSTED=0..7`。布尔值只接受 0/1；Safety sensor bit0/1 为 gyro/grayscale；scheduler task bit0..3 为 5/10/20/100 ms；ADC interrupt raw flags 只使用 `OVERFLOW=0x1,TRIG_OVF=0x2,UNDERFLOW=0x40,MEM0_LOADED=0x100`。image、reset、parameter summary、grayscale、trace 及其余 flags 也必须只使用 JSON 的 known mask，未知位拒绝或由主机标记不兼容，禁止自行扩展。

RESET 的紧凑事件已含 watchdog breadcrumb；完整 reset 只由 RESET_GET 返回。P0 topic 是 event code；同 topic 再发生时保留最新 body、饱和累加 occurrence、置 GAP 并把被覆盖实例计 `p0_drop`。P1 是有界异步通知而非绝对可靠日志：除 `TELEMETRY_GAP` 以 code+stream ID 为 topic 外，其余普通 P1 以 event code 为 topic；同 topic 合并采用同一 latest/occurrence/GAP/drop 语义。满队新 topic 先按 LINK_STATE、再按 TELEMETRY_GAP 的顺序淘汰各类最旧项，否则丢 incoming 并计 `p1_drop`；主机用 ACTION_GET、PARAM/STATUS/RESET 等 RPC 读取最终真值。只有 RPC 明确承诺异步完成的 `GRAYSCALE_CAPTURE_START` 必须在执行前预留对应 P1 slot，成功请求的 READY 事件不得降级到 latest-only P2。Safety fault bits 固定为 emergency/gyro/grayscale/action/scheduler/watchdog/encoder/stall/line lost = `0x001..0x100`；SystemState `BOOT/SELF_TEST/IDLE/RUN/FAULT=0..4`；ResetClass `none/power/external/software/watchdog/debug/hardware/bsl/unknown=0..8`；checkpoint 使用现有 append-only `0..19`。

BULK common 16 B：`kind:u8,schema:u8,reserved:u16(=0),transfer_id:u32,object_id:u32,offset:u16,count:u8,record_size:u8`，其后 data≤80 B；kind 1 为 LINE_TRACE，gap 只使用 frame header 的 `GAP_BEFORE`。line trace info 24 B 为 `capture_id:u32,state_flags:u8,sample_count:u8,capacity:u8,post_remaining:u8,record_size:u8,max_records_per_chunk:u8,reserved:u16,first_time_ms:u32,last_time_ms:u32,overwritten:u32`；state flags bit0 active、bit1 triggered、bit2 frozen。record 固定 30 B：`time:u32,adc[8]:u16,position_x1000:i16,correction_mm_s:i16,active mask/count/cluster/state,search_direction:i8,flags:u8`；search direction 与实时 LINE 使用同一 signed registry，record flags 使用独立 registry，仅 bit0 `BRANCH_LATCHED`，不得复用 bit0 表示 `VALID` 的实时 `line_flags`。每 chunk 1–2 records，`count*record_size` 必须等于 data 长度；offset 是 chronological record index。chunk 只读取 capture ID 匹配的 frozen 记录，活动/已替换 capture 返回 `WRONG_STATE/NOT_FOUND`。accepted chunk 不因普通拥塞丢弃，P3 不足在 admit 前 BUSY；emergency/Safety fault/session teardown 仍可撤销未发送 P3，并把 abort 计入 `p3_drop`，主机随后用新 sequence、同 capture ID/offset 重取。

## 8. reset snapshot v1

固定 76 B，三个 bool 压入 flags bit0 coarse、bit1 detail、bit2 hardfault；不得 memcpy enum/padding：

```text
0 raw_cause:u32; 4 reset_class:u8; 5 current_checkpoint:u8;
6 flags:u8; 7 previous_checkpoint:u8; 8 boot_id:u32;
12 previous_sequence:u32; 16 previous_boot_id:u32; 20 previous_time_ms:u32;
24 previous_last_wwdt_ms:u32; 28 previous_miss_total:u32;
32 previous_fault_bits:u32; 36 previous_uart_tx_drop:u32;
40 previous_uart_rx_drop:u32; 44 previous_telemetry_drop:u32;
48 previous_pending_mask:u32; 52 previous_missed_mask:u32;
56 hardfault_pc:u32; 60 hardfault_lr:u32; 64 hardfault_xpsr:u32;
68 previous_boot_id_low:u8; 69 previous_detail_checkpoint:u8;
70 previous_system_state:u8; 71 previous_telemetry_queue:u8;
72 previous_health_mask:u8; 73 reserved[3]
```

`previous_checkpoint` 严格对应现有 `ResetDiag_GetSnapshot()`：通常是 coarse checkpoint；存在有效 HardFault scratch 时会被选择为 `HARDFAULT`，因此不得解释成“纯 coarse”。这同时保留了 BOOTRST coarse-only 时唯一可用的 previous boot8，以及可能不同的 detail checkpoint。

## 9. 优先队列、背压与资源上限

相对 r13 MAP，`RW_IRAM2` 末端 `0x20206630` 到 `.noinit 0x20207F00` 有 6352 B；不在没有栈高水位证据时缩 16 KiB stack。R3 新静态 RAM 硬上限 2560 B：

| 资源 | 上限 |
|---|---:|
| P0 三个固定 latch（reset/safety/emergency） | `3×72=216 B` |
| replay/RPC result 四项 | `4×152=608 B` |
| P1 async event FIFO 四项 | `4×72=288 B` |
| control-error/BUSY latch | `32 B` |
| P2 四流 latest-only | `4×80=320 B` |
| P3 两条完整 payload slot | `2×112=224 B` |
| 唯一 in-flight wire / RX decoded | `224+112=336 B` |
| parser/mode/token/stats/对齐余量 | `≤536 B` |
| 合计 | `≤2560 B` |

R3 用 `_Static_assert` 验证每类结构和总静态预算，MAP 验证 `RW_IRAM2<=0x20207030`，到 `.noinit` 至少余 3792 B。R3 Code+RO 增量≤12 KiB，最高普通镜像目标≤约 `0x100EF`，参数区仍从 `0x1F800`。

普通仲裁为 `P0 > RPC result > BUSY/protocol error > terminal LINK_STATE > P1 async > P2 > P3`。唯一例外是 next async 已到专用 `0xFFFFFFFE` 的终止态：P0/P1/P2/P3 暂停 eligibility，只允许 RPC result、control-error 后发送 terminal LINK_STATE；P0 latch 保留供 RPC 查询。terminal LINK_STATE 由仲裁器直接合成到唯一 in-flight buffer，不占 P1 且不可丢；P0 固定 latch 同主题合并；P1 按上一节的合并/预留/drop 规则，P3 在 admit 前预留且仅可被安全/会话事件 abort；P2 每流只保留最新。编码到唯一 224 B buffer，ISR 只泵 UART FIFO；发送游标结束、软件/FIFO 空且 `DL_UART_isBusy()==false` 才释放槽。每个 frame 物理完成后重新仲裁，不预装第二个低优先 frame。

V1 继续用旧 1024 B FIFO；切 V2 时按第 3 节冻结所有 producer、处理上层队列并验证完整物理排空，不能只调用一次 `debug_tx_idle()`；V2 不走旧 FIFO。P2 token `7000 B/s, bucket 448 B`，P3 `1000 B/s, bucket 224 B`，按实际含 delimiter 的 wire length扣减；P0/P1 绕过 token。224 B 在 115200 8N1 下最坏 19.444 ms，P0 不截断在途帧；停车仍由 ISR/Safety 完成。

## 10. Golden vectors 与验收

JSON schema、显式 CRC/decoded/wire expected 和生成的 C `.inc` 与 registry 一起提交。`scripts/check_m6_v2_vectors.py --check` 必须严格拒绝 duplicate JSON key、NaN/Inf、数值重复、布局间隙/越界、参数 ID 不完整、CRC/escape/session/frame/stream 漂移和生成文件漂移；普通测试不得静默重写 expected。

最低覆盖：CRC empty/`123456789`/bytes 0..255；全部四个强制转义；HELLO/READY；ping request/response/replay；未知 type/protocol error；generic core telemetry；96 B 最大 payload；分片、直接拼帧、前导噪声、CRC 翻位、截断、非法 escape、恰好 50/100 ms、u32 时间回绕、恰好/超过 224 wire、超过 111 decoded、DISCARD 恢复和 RX/emergency epoch 清帧。合法 wire delimiter 之间不得出现裸 `0x20/0x53/0x7E`。

后续 C harness 还必须覆盖四阶段 replay、P0/P1 预留、P2/P3 背压、HELLO 固定 required/重叠拒绝、未知 version 的 ARMED/ESTABLISHED 分支、`0xFFFFFFFD→terminal 0xFFFFFFFE` 专用序号边界、物理 TX 完成、V1/V2/RESYNC 每条边界、共享参数服务和 ISR 急停。JSON/checker 不能替代 Keil/MAP 或上板日志负载证据；不提前宣称 r13/V2 硬件通过。

冻结 registry/schema 的 canonical SHA-256 分别为 `23fbd056db28c1fedc7987a34d0893594fddb417f34058851667ffc2d5dca0a2` / `95191cda4ea5ed0691126af1fa027ff37b3dd6f920ede9c099cad0b16fce7ede`。当前向量清单为 CRC 3、escape 1、session 2、legacy ASCII 1、resync 1、frame 10、stream 22；`scripts/check_m6_v2_vectors.py --check`、6 项聚焦测试和全量 104 项主机测试均已通过。revision 1 或 schema 内容发生任何变化都必须先提升 registry revision 并同步更新本文、schema、golden vectors、生成 include、检查器、`STATE.md` 和日期证据，不能用 `--write-inc` 覆盖冻结身份。
