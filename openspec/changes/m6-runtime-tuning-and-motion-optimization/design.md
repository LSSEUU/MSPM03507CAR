## Context

The current firmware is a cooperative, safety-arbitrated control system. Wheel speed runs every 5 ms, grayscale and JY61P acquisition run every 10 ms, heading/line/remote control run every 20 ms, and diagnostics run every 100 ms. UART0 already uses interrupt-driven RX/TX rings, and the RX ISR consumes `S` and space as emergency-stop bytes before ordinary command dispatch. All motor requests pass through `System/safety.c`; inactive control paths brake both motors.

Control values are currently compile-time constants in `Config/control_config.h` and `Config/remote_config.h`. The only runtime tuning path changes the speed-diagnostic target and wheel PI gains in RAM through single-character commands. There is no coherent parameter snapshot, validation across related parameters, persistence, or telemetry fast enough to compare line-following oscillation. The validated line-following baseline is 160 mm/s. PB11 remote-control phase two remains incomplete and is therefore a hard prerequisite for high-speed ground testing, although parameter infrastructure and wheels-raised tests can proceed first.

The target is an MSPM0G3507 running at 32 MHz with Keil ARM Compiler 6.22. Main flash occupies `0x00000000-0x0001FFFF`; flash erase granularity is 1 KiB and a sector erase can take up to 200 ms. WWDT0 expires in approximately 31 ms, so flash operations cannot execute from main flash or block without a RAM-resident watchdog service path. UART0 is 115200 8N1, which constrains 50 Hz telemetry to a compact format. Existing safety thresholds, watchdog policy, board configuration, legacy commands, and action arming are compatibility boundaries, not tuning parameters.

Stakeholders are the operator performing bench/on-car validation, the host tuning tool, and later firmware work that must be able to reproduce the accepted M6 parameter set from source and from persisted storage.

## Goals / Non-Goals

**Goals:**

- Provide one validated parameter model for wheel, heading, line-following, and remote-driving controls, with separate compiled defaults, active values, and staged values.
- Provide deterministic UART transactions for inspection, staging, atomic application, rollback, persistence, and telemetry without weakening ISR emergency stop or legacy command behavior.
- Persist active parameters across reset and power loss using two redundant, versioned, CRC-protected main-flash records.
- Improve target shaping, yaw-rate use, turn settling, line estimation, correction smoothness, and speed scheduling while retaining existing safety arbitration.
- Capture non-blocking 50 Hz evidence and automate conservative candidate scoring/rollback.
- Preserve a reproducible 160 mm/s compatibility baseline, then validate 220 mm/s for five laps; treat 250 mm/s for three laps as an extension.

**Non-Goals:**

- Making physical safety limits, encoder plausibility thresholds, stall detection, sensor timeouts, PWM limits, WWDT settings, or fault arbitration runtime-writable.
- Replacing the current cooperative scheduler, UART transport, grayscale hardware, JY61P, or CMSIS-DAP/SWD toolchain.
- Guaranteeing parameter retention through a debugger mass erase or firmware download.
- Automatically entering ROM BSL or using GUI automation when SWD download fails.
- Claiming PB11 phase-two completion, M7 endurance/low-voltage/fault-injection acceptance, or hardware performance from host tests or source inspection.

## Decisions

### 1. Parameter ownership and atomic lifecycle

A new `System/control_params` module owns the public `ControlParams_t` model and all validation. It contains grouped fixed-layout structures:

```c
typedef struct {
    float kp[2];
    float ki[2];
    float filter_alpha;
    float accel_mm_s2;
    float decel_mm_s2;
    float jerk_mm_s3;
    float feedforward_ks[2];
    float feedforward_kv[2];
    float diagnostic_target_mm_s;
    uint32_t feedforward_enabled;
} WheelControlParams_t;

typedef struct {
    float hold_kp;
    float hold_kd;
    float hold_max_correction_mm_s;
    float turn_kp;
    float turn_kd;
    float yaw_rate_filter_alpha;
    float turn_max_speed_mm_s;
    float turn_slow_zone_deg;
} HeadingControlParams_t;

typedef struct {
    float speed_min_mm_s;
    float speed_max_mm_s;
    float position_filter_alpha;
    float derivative_filter_alpha;
    float kp;
    float kd;
    float max_correction_mm_s;
    float correction_slew_mm_s2;
    float curve_slowdown_gain;
    float min_inner_speed_mm_s;
    float search_forward_mm_s;
    float search_turn_mm_s;
} LineControlParams_t;

typedef struct {
    float speed_low_mm_s;
    float speed_mid_mm_s;
    float speed_high_mm_s;
    uint32_t heading_hold_enabled;
} RemoteControlParams_t;

typedef struct {
    WheelControlParams_t wheel;
    HeadingControlParams_t heading;
    LineControlParams_t line;
    RemoteControlParams_t remote;
} ControlParams_t;
```

`uint32_t` is used for persisted booleans so the serialized representation does not depend on compiler `bool` layout. The module stores a const compiled-default object, a staged object, two active slots, an atomic active-slot index, a dirty-bit set, a monotonic parameter revision, and source metadata (`defaults`, `flash_a`, or `flash_b`). A successful apply copies the validated staged object into the inactive slot and flips the index at the next 5 ms boundary; readers take the active pointer once per control update. This avoids exposing a partially copied structure without holding interrupts off for a large copy.

The machine keys and validation bounds are fixed as follows:

| Keys | Bounds / relation |
|---|---|
| `wheel.l.kp`, `wheel.r.kp` | 0 to 3 |
| `wheel.l.ki`, `wheel.r.ki` | 0 to 10 |
| `wheel.filter_alpha` | 0.05 to 1 |
| `wheel.accel`, `wheel.decel` | 100 to 2000 mm/s2 |
| `wheel.jerk` | 1000 to 20000 mm/s3 |
| `wheel.l.ks`, `wheel.r.ks` | 0 to 850 PWM counts |
| `wheel.l.kv`, `wheel.r.kv` | 0 to 5 PWM counts/(mm/s) |
| `wheel.ff_enable` | integer 0 or 1; for each wheel `ks + kv * 400 <= 680` |
| `wheel.test_speed` | 50 to 400 mm/s |
| `heading.hold_kp`, `heading.turn_kp` | 0 to 10 mm/s/deg |
| `heading.hold_kd`, `heading.turn_kd` | 0 to 5 mm/s/(deg/s) |
| `heading.rate_alpha` | 0.05 to 1 |
| `heading.hold_limit` | 0 to 250 mm/s |
| `heading.turn_speed` | 50 to 400 mm/s |
| `heading.slow_zone` | 2 to 60 deg |
| `line.speed_min`, `line.speed_max` | 50 to 400 mm/s; min <= max |
| `line.pos_alpha`, `line.deriv_alpha` | 0.05 to 1 |
| `line.kp` | 0 to 400 mm/s per normalized position |
| `line.kd` | 0 to 100 mm/s per normalized-position/s |
| `line.corr_limit` | 0 to 250 mm/s |
| `line.corr_slew` | 100 to 2000 mm/s2 |
| `line.curve_gain` | 0 to 2 |
| `line.inner_min` | 0 to 200 mm/s and <= `line.speed_min` |
| `line.search_fwd`, `line.search_turn` | 50 to 400 mm/s and sum <= 650 mm/s |
| `remote.speed_low`, `remote.speed_mid`, `remote.speed_high` | 50 to 400 mm/s; low <= mid <= high |
| `remote.heading_hold` | integer 0 or 1 |

Every numeric input must also be finite; textual `nan`, `inf`, overflow, and trailing characters are rejected. The final target sent to either wheel is independently hard-clamped to +/-700 mm/s regardless of parameter validity. Initial defaults preserve existing constants wherever the new algorithm permits; new feedforward is disabled with zero coefficients. The line search-forward default becomes 50 mm/s to satisfy the approved minimum. The accepted on-car configuration is copied back into the compiled-default object only after the full M6 acceptance sequence.

`set` changes only staged data. `revert` replaces staged data with active data. `load` and `defaults` replace staged data with the newest valid stored record or compiled defaults respectively, and require a later `apply`; neither changes motion behavior by itself. `save` writes the active snapshot only. Each successful apply increments the revision once and synchronizes staged to the new active snapshot.

Normal applies are accepted only when the system is fault-free and `IDLE`. During the existing armed/running wheel-speed diagnostic, a pending apply is accepted only if its dirty set contains wheel Kp/Ki and/or `wheel.test_speed`; it is committed at a 5 ms control boundary, resets the affected PI integrator and diagnostic statistics, and lets the jerk-limited target shaper move to the new target. All other active-state applies return `state` without changing active data. This preserves deterministic control boundaries while retaining live PI tuning.

Alternatives considered were individual global variables and immediate per-key writes. They were rejected because they permit invalid cross-parameter combinations and allow a control update to observe a half-applied configuration. A heap-allocated pointer swap was also rejected because the firmware does not otherwise require dynamic allocation.

### 2. UART transaction protocol and compatibility

A bounded parser in `System/control_protocol` consumes bytes currently dispatched by `Diagnostics_Poll`. A leading `:` selects transaction mode; any other byte follows the existing single-character dispatch unchanged. Frames contain no spaces, use lowercase verbs/keys, terminate with CR and/or LF, and are limited to 58 payload characters so the complete line with CRLF is at most 60 bytes. The decimal request ID is 1-65535. A repeated `:` restarts the partial transaction. Overflow, an RX-drop counter change, or an emergency-stop epoch change discards the partial frame and drains to the next line ending.

The accepted grammar is:

```text
:<id>,get,<key>
:<id>,set,<key>,<decimal-value>
:<id>,apply
:<id>,revert
:<id>,load
:<id>,defaults
:<id>,save
:<id>,log,<off|m6>,<period-ms>
```

Decimal values allow an optional leading sign and fractional part, but not exponent notation. `get` on a parameter returns both active and staged values so the host can observe dirty state: `ok,<id>,<key>,<active>,<staged>`. Read-only keys `meta.firmware`, `meta.schema`, `meta.revision`, `meta.source`, and `meta.generation` use the same `get` verb and return one value. Firmware identity is an M6 build ID defined in source; schema starts at `1`.

Successful mutating responses are respectively `ok,<id>,staged,<key>,<value>`, `ok,<id>,applied,<revision>`, `ok,<id>,reverted,<revision>`, `ok,<id>,staged,<source>`, `ok,<id>,saved,<generation>`, and `ok,<id>,log,<mode>,<period>`. Errors are `err,<id>,<code>` with stable codes `syntax`, `unknown`, `readonly`, `range`, `relation`, `state`, `busy`, `storage`, `overflow`, and `unsupported`. Values are formatted with at most six significant digits so all transaction responses remain within 60 characters.

Only one boundary-apply or flash-save request can be pending. A second request receives `busy`. The reply is emitted only after the boundary commit or flash verification has completed. The host uses a longer timeout for `save` and sends no concurrent traffic during it.

UART0 still consumes `S` and space in the RX ISR before either parser. The ISR increments an emergency epoch in addition to invoking the existing safety callback; the parser compares this epoch before executing a complete transaction and discards any pre-emergency partial frame. Thus bytes surrounding an emergency stop cannot accidentally form an apply/save request. Flash save is the only interval with global interrupts disabled, and it is allowed only after the system has already braked in `IDLE`.

Alternatives considered were JSON, a binary protocol, and replacing legacy commands. JSON is too verbose for the RX ring and embedded parser, binary framing is harder to operate manually, and replacement would invalidate established bench procedures. The bounded CSV-like transaction format is sufficient for exact host control and human diagnosis.

### 3. Redundant persistent storage

The Keil scatter load/execution region and project IROM limit are reduced from `0x00020000` to `0x0001F800`. Two sectors are excluded from all normal sections and HEX payload checks:

- slot A: `0x0001F800-0x0001FBFF`
- slot B: `0x0001FC00-0x0001FFFF`

The runtime object is converted to an explicit `ControlParamsPayloadV1` with ordered IEEE-754 binary32 fields and `uint32_t` flags; no compiler padding is included in the CRC or record. Static assertions lock every serialized size and 64-bit alignment. Each sector contains one record:

```text
magic=0x4D365052 | schema_version:u16 | payload_length:u16
generation:u32 | parameter_revision:u32 | payload_v1
crc32:u32 | padding-to-64-bit | commit_marker:u32 | commit_reserved:u32
```

CRC-32 uses reflected polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`, covering the bytes from `magic` through the end of the payload. `commit_marker` is `0x434F4D54`; `commit_reserved` remains erased. A record is valid only when magic, schema, exact payload length, final commit marker, CRC, finite/range/relational validation, and flash readback all pass.

At boot, both records are checked without modifying flash. The newest generation is selected with wrap-safe signed subtraction; an exact tie deterministically selects A. Its revision and source are restored into active and staged state. If neither record is valid, compiled defaults are used and parameter-storage health is reported as `DEGRADED` with the invalid-slot reason, but no motion fault is latched. An unknown schema is invalid rather than being reinterpreted.

Saving is allowed only when all of these are true: no fault is latched, system state is `IDLE`, diagnostics and remote actions are idle, wheel control is disabled, and the safety/motor snapshot confirms both outputs are braked. The invalid or older slot is the target; the currently selected valid slot is never erased first. The target sector is erased, the header/payload/CRC area is programmed and read back, and the aligned final 64-bit word containing the commit marker is programmed last. Only after full reread and CRC validation does source/generation metadata advance. A failed operation leaves the previous committed record authoritative.

The erase/program critical path, flash register helpers, CRC/readback data needed during the operation, and direct WWDT restart helper reside in `.ramfunc`/RAM. It disables interrupts, unprotects only the target sector, issues non-blocking sector erase and 64-bit ECC-generated program commands, polls status from RAM while restarting WWDT0, checks command/failure status, reprotects the sector, restores the prior interrupt mask, and then performs the normal flash readback. The linker MAP test verifies that this call graph has no main-flash dependency. A vendor blocking erase routine is not used because it cannot service the approximately 31 ms watchdog during the 200 ms worst case.

Alternatives considered were a single in-place record, emulated EEPROM, and storing parameters in non-main memory. A single record cannot survive power loss during erase; an emulation layer is unnecessary for infrequent explicit saves; and the current project has no validated writable non-main-memory allocation. Two fixed sectors provide the smallest auditable failure model.

### 4. Wheel controller and target shaping

Each wheel keeps ramped velocity and ramp acceleration. At each actual measured `dt`, the shaper first substitutes zero as the effective target when requested and current velocity have opposite signs. It clamps desired acceleration to the configured acceleration/deceleration limit, moves current acceleration toward that value by at most `jerk * dt`, integrates velocity, and prevents overshoot of the effective target. Only after ramped velocity reaches zero may it accelerate in the opposite direction. Final targets remain limited to +/-700 mm/s.

The speed filter alpha is read from the active snapshot. Runtime filter changes occur only through an idle apply, so filters and controller state are reset before motion resumes. The PI path exposes error, P term, I term, feedforward term, final output, actual `dt`, encoder delta, and cumulative saturation time. Conditional integration uses the combined `P + I + feedforward` result: it integrates only when the combined output is unsaturated or the error would drive it back toward the legal PWM range.

Optional feedforward is `sign(target) * (ks + kv * abs(target))` per wheel and is zero at a zero ramped target. It is disabled in compiled defaults. The host fits coefficients from steady-state 100/160/220/300 mm/s data and enables them only if held-out scoring improves by at least 10% without increasing overshoot or sustained saturation. Final PWM still passes through the existing hard limit and Safety arbitration.

Alternatives considered were a first-order velocity ramp and unconstrained feedforward. The existing first-order ramp has acceleration discontinuities that contribute to pitching and wheel slip; unconstrained feedforward can consume all PI authority as battery voltage changes. The jerk-limited ramp and 80% feedforward relation preserve control headroom.

### 5. Heading, calibration, and remote straight hold

The JY61P parser decodes checksum-valid `0x52` angular-velocity frames in addition to `0x53` angle frames. Z rate uses the documented `raw * 2000 / 32768` deg/s scale and has a separate frame counter/timestamp. Heading uses the filtered direct rate when its age is at most 100 ms; otherwise it falls back to the existing wrapped angle difference using actual `dt`. Both sources pass through `heading.rate_alpha`, and telemetry identifies the selected source. Angle freshness continues to use the existing 250 ms safety timeout.

Calibration still collects 20 unique angle frames, but a candidate window is accepted only when wrapped yaw peak-to-peak is at most 1.0 deg, yaw standard deviation is at most 0.35 deg, and all fresh direct-rate magnitudes are at most 3 deg/s. A rejected window increments a diagnostic counter and restarts collection; heading-dependent actions remain unavailable until a stationary window succeeds.

Turn control computes the existing PD command, then limits its magnitude by a continuous smoothstep envelope based on `abs(error) / heading.slow_zone`. Outside the zone the limit is `heading.turn_speed`; at zero error it is zero. The former discontinuous 35 mm/s minimum is removed. Existing tolerance/settle-sample completion and action timeout remain fixed safety behavior. Positive and negative 90-degree tests verify the published JY61P sign convention.

When `remote.heading_hold` is enabled, the first forward or reverse event captures the current heading and starts heading hold with a signed base speed. Repeats preserve that reference. A direction reversal, left/right command, normal stop, heartbeat expiry, or fault stops heading hold; left/right commands retain direct differential manual control. Forward/reverse actions declare the gyro as a required safety sensor, so stale angle data follows the existing fault-and-brake path. When disabled, remote behavior is identical to the current implementation.

Alternatives considered were continuing to differentiate yaw angle and applying heading hold to manual turns. Direct rate avoids derivative noise and wrap sensitivity, while manual turn hold would fight the operator's requested differential target.

### 6. Line estimation and curve-aware speed scheduling

Each grayscale sample publishes an 8-bit active mask, active count, and contiguous-cluster count. Classification is deterministic: count zero is `none`; mask `0xFF` is `all`; more than one cluster is `multi`; one cluster with one to three active sensors is `track`; and one cluster with four to seven sensors is `wide`. `all`, `multi`, and `wide` are ambiguous. Up to two consecutive ambiguous samples retain the last valid direction while forcing base speed to `line.speed_min`; the third follows the existing line-loss/search path. A normal sample must meet the existing two-sample acquire rule before returning from search, at which point derivative state is reset.

For a valid track sample, raw weighted position is low-pass filtered by `line.pos_alpha`. Derivative is computed from filtered position and actual elapsed time, then low-pass filtered by `line.deriv_alpha`. Start and reacquisition initialize both filters from the current position and set derivative to zero, preventing a derivative kick.

The unconstrained correction is `line.kp * filtered_position + line.kd * filtered_derivative` and is clamped to `line.corr_limit`. Its change per 20 ms update is then limited by `line.corr_slew * dt`. Curve severity is:

```text
severity = clamp(line.curve_gain * max(
    abs(filtered_position),
    min(abs(filtered_derivative) / 4, 1),
    abs(unconstrained_correction) / max(line.corr_limit, 1)), 0, 1)
base = line.speed_max - severity * (line.speed_max - line.speed_min)
```

The outer wheel receives `base + abs(correction)` and the inner wheel receives no less than `line.inner_min`, with left/right chosen by correction sign. Both are finally clamped to the wheel target hard limit. Search mode remains a separate differential `search_forward +/- search_turn` behavior and retains the existing 1500 ms loss timeout/fault path.

Alternatives considered were changing only line Kp/Kd and using a discrete straight/curve threshold. Gain-only tuning cannot remove target discontinuities, and a threshold causes speed steps near its boundary. Continuous severity, correction slew, and an inner-wheel floor address the observed lateral oscillation and low-speed stick-slip directly.

### 7. Non-blocking M6 telemetry

The 20 ms task copies already-computed snapshots into an eight-entry ring of fixed-size integer samples; it does not format text or wait for UART. If full, it drops the newest sample and increments a telemetry-drop counter. Background service formats at most one queued sample per pass and uses a new atomic UART enqueue operation that either reserves the complete line or leaves the sample queued, preventing partial CSV lines when the TX ring lacks space.

`log,m6,<period>` accepts 20-1000 ms in 20 ms multiples; `log,off,0` disables collection. Starting M6 logging emits one header, then integer rows in this fixed order:

```text
M6,time_ms,revision,request_l,request_r,ramp_l,ramp_r,
raw_l,raw_r,filter_l,filter_r,p_l,p_r,i_l,i_r,ff_l,ff_r,
pwm_l,pwm_r,yaw_x100,rate_x100,rate_source,gray_mask,line_info,
position_raw_x1000,position_filter_x1000,derivative_x1000,
dynamic_base,correction,saturation_bits,fault_bits,missed_tasks,
rx_dropped,tx_dropped,telemetry_dropped
```

`line_info` packs active count in the high nibble and cluster count in the low nibble. `saturation_bits` records left/right PWM saturation, correction clamp, correction slew, and wheel-target clamp. All float-to-integer conversions are rounded and saturating. At expected field widths, the line remains below the 115200-baud 50 Hz budget; queue/drop counters make any violated assumption visible.

Alternatives considered were formatting in the scheduled task and printing floating-point CSV. Both increase task jitter and UART occupancy. Integer snapshots preserve the resolution needed for scoring while making transport cost predictable.

### 8. Host tuning and firmware download workflow

`scripts/m6_tune.py` uses pyserial and exposes `probe`, `get`, `set`, `apply`, `defaults`, `load`, `save`, `record`, `score`, and `tune-wheel` subcommands. Auto-discovery probes enumerated serial ports at 115200, uses unique transaction IDs, and accepts a port only when `meta.firmware` has the expected M6 prefix and `meta.schema` equals 1. Every transaction checks the echoed ID and has bounded retries only for transport reads, never repeated motion commands.

Recording preserves raw CSV plus a JSON summary containing firmware ID, schema, revision, source, active parameter snapshot, timestamps, scoring metrics, and abort reason. Wheel metrics are rise time to enter and remain within +/-5% for 300 ms, peak overshoot, last-second mean absolute error, post-settle error zero crossings, saturation ratio, and fault/drop deltas. Ground-run metrics add yaw RMS/peak, line-position RMS, correction jerk RMS, loss events, scheduler misses, and UART/telemetry drops. Any fault, line loss, sustained saturation, missed task, UART drop, telemetry drop, or operator emergency stop is a hard rejection independent of numeric score.

Automatic search stores the current accepted snapshot, changes only one parameter family at a time, and performs at most three coordinate-search rounds. Candidate steps are bounded to 10% of the current value or a key-specific minimum step and are clipped to firmware ranges. A candidate is accepted only when its aggregate normalized error score improves by at least 5% and no hard metric regresses beyond its acceptance limit. Failure sends `S`, stages the last accepted snapshot, and permits no further motion; if a fault is latched, rollback apply waits for the operator to clear the fault and restore `IDLE`. The tool never automatically saves a candidate. `--save-final` is honored only after all requested validation points pass. Feedforward uses separate train/held-out speed points and the stricter 10% rule described above.

`scripts/flash.ps1` reuses `Resolve-KeilUV4`, the production project, and its existing `Horco CMSIS-DAP v2`/serial `4041840e` CMSIS-AGDI configuration. It verifies the probe is enumerated, the project/option files contain the expected probe/serial and TI flash algorithm, and a successful production rebuild exists. It invokes the existing hidden Keil command form `UV4.exe -f <project> -o <log>`, requires exit code zero and `Verify OK`, and writes a dated log. On failure it runs the exact same command once more with no configuration changes. A second failure terminates with a manual-download instruction; it does not open Keil, switch probes/configuration, or attempt BSL. After success it uses `m6_tune.py probe` to verify firmware ID, schema, revision, and startup source from the running target.

Alternatives considered were an unconstrained optimizer and GUI/BSL fallback. Coordinate search is slower but auditable and safe for a physical vehicle. The existing evidence requires one identical SWD retry and then operator intervention; changing transport after failure would expand the diagnostic boundary without evidence.

## Risks / Trade-offs

- [Flash operation executes while main flash is unavailable] -> Keep the complete erase/program/poll/watchdog call graph in RAM, prove placement from the MAP, save only while braked in `IDLE`, and retain the previous committed slot until final verification.
- [Power fails between erase and commit] -> Select only records with the final marker and valid CRC; alternate slots so the former generation remains bootable.
- [Compiler layout or future schema reinterprets persisted bytes] -> Serialize through an explicit V1 payload with static size assertions and reject unknown schema/length values.
- [A debugger download erases both parameter sectors] -> Do not promise retention across flashing; compiled defaults remain a reproducible recovery point and post-flash source is always queried.
- [UART bandwidth or background delay loses evidence] -> Use compact integer rows, an eight-entry snapshot queue, atomic TX enqueue, configurable periods, and explicit RX/TX/telemetry drop counters; any drop invalidates an acceptance run.
- [Runtime parameter changes destabilize active control] -> Validate the complete staged object, atomically swap only at a 5 ms boundary, restrict live apply to diagnostic Kp/Ki/target, reset affected state, and retain hard output limits.
- [Feedforward overfits one battery/bench condition] -> Default it off, bound it to 80% PWM at 400 mm/s, require held-out improvement, and reject any saturation/overshoot regression.
- [Direct JY61P rate is stale, noisy, or has the wrong sign] -> Track independent freshness, filter it, fall back to wrapped angle difference, and require positive/negative turn regression before ground speed increases.
- [Ambiguous sensor masks cause false steering] -> Classify topology, slow and hold direction only briefly, then use the existing bounded search/fault path.
- [Automated tuning causes uncontrolled repeated motion] -> Require the existing arm/confirm path, stop on the first hard failure, limit search rounds, restore the accepted snapshot before any further run, and never auto-save.
- [SWD remains intermittent] -> Preserve the exact configuration, retry the same command only once, and stop for manual download after the second failure.
- [Higher speed is attempted before remote safety validation] -> Gate every high-speed ground step on documented PB11 phase-two completion; only wheels-raised and low-speed work may precede it.

## Migration Plan

1. Create the parameter model, transaction parser, read-only metadata, and 50 Hz telemetry while retaining the existing control equations. Map current constants to initial defaults, run host regressions and Keil Rebuild, download this compatibility build, and record the 160 mm/s baseline before enabling optimized algorithms.
2. Reserve the two flash sectors in both scatter/project memory limits, implement V1 serialization and RAM flash routines, and verify default boot, A/B selection, corruption fallback, save/readback, reset retention, and power-cycle retention. Do not claim retention through download until separately measured.
3. Introduce jerk-limited wheel shaping and expanded PI snapshots, then JY61P rate/turn smoothing, then line topology/filter/scheduling, and finally optional remote heading hold. Keep each subsystem behind its existing safety/action boundary and rerun M2-M5 host regressions plus +/-90-degree and 160 mm/s compatibility checks after each integration step.
4. Add and host-test `m6_tune.py` and `flash.ps1`, including mocked protocol fragmentation, invalid floats/ranges, CRC/generation failure cases, telemetry backpressure, hard-abort rollback, exact one-retry download behavior, and post-flash identity checks.
5. Complete/reconfirm PB11 phase two. Then validate in order: wheels-raised left/right/both at 100/160/220/300 mm/s; low-speed ground start/stop/straight/heading; line following at 100, 160, 180, 200, then 220 mm/s. Require five clean laps at 220 mm/s before attempting 235/250 mm/s, and require three clean laps for the 250 mm/s extension.
6. Apply and explicitly save the accepted parameter set, verify reset and power-cycle reload, then update compiled defaults to the same values. Record measured evidence in a dated M6 validation document, synchronize `STATE.md`, and commit each coherent implementation/verification milestone.

Rollback uses the last accepted active parameter snapshot for tuning regressions and the preceding known-good firmware commit for code regressions. A firmware rollback is rebuilt and flashed through the same two-attempt SWD policy. Stored V1 sectors are ignored by firmware that has no matching schema; if the current M6 firmware sees invalid/unknown storage, it boots compiled defaults with `DEGRADED` status and no latched motion fault.

## Open Questions

None.
