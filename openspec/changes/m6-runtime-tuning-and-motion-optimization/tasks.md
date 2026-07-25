## 1. Runtime Parameter Foundation

- [x] 1.1 Add the grouped `ControlParams_t` defaults, active/staged snapshots, source/revision metadata, descriptor table, and finite/range/relation validation.
- [x] 1.2 Implement staged set/get/defaults/revert/load hooks and atomic 5 ms boundary apply, including the restricted live wheel-diagnostic path.
- [x] 1.3 Add host tests for boundary values, malformed/NaN/Inf inputs, relation failures, atomic application, revision/source metadata, and diagnostic no-bump updates.

## 2. Transaction Protocol and Compatibility

- [x] 2.1 Add the bounded `:<id>,...` CR/LF transaction parser and fixed `ok`/`err` responses while preserving all legacy single-character commands.
- [x] 2.2 Preserve ISR emergency-stop preemption in every parser state and contain fragmented, concatenated, malformed, and overlength frames.
- [x] 2.3 Gate apply/save operations on Safety and diagnostic state and add protocol/legacy/emergency-stop host tests.

## 3. Persistent Parameter Storage

- [x] 3.1 Define the explicit V1 payload, A/B record layout, CRC32, generation comparison, validation, selection, and default-degraded fallback.
- [x] 3.2 Implement alternating commit-last saves with readback verification, exact safe-IDLE gates, sector reprotection, and previous-record preservation.
- [x] 3.3 Implement and place the complete erase/program/status-poll/watchdog critical path in RAM with flash interrupts disabled.
- [x] 3.4 Reserve `0x0001F800-0x0001FFFF` in the scatter/project memory configuration and add MAP/HEX boundary verification.
- [x] 3.5 Add host tests for CRC/generation/tie selection, corruption/schema fallback, interrupted writes, active-only save, and serialization layout.

## 4. Wheel and Heading Optimization

- [x] 4.1 Add per-wheel jerk-limited target shaping with zero-crossing reversal and hard target clamping.
- [x] 4.2 Add runtime speed-filter updates, conditional anti-windup with optional bounded per-wheel feedforward, and full wheel-loop snapshots.
- [x] 4.3 Decode timestamped JY61P `0x52` yaw-rate frames, use fresh direct rate with angle-difference fallback, and expose the source.
- [x] 4.4 Add stationary calibration quality gates, continuous turn near-target slowdown, and optional remote straight heading hold.
- [x] 4.5 Extend host regressions for wheel shaping/feedforward, rate fallback/calibration, turn smoothing, and remote behavior.

## 5. Line-Following Optimization

- [x] 5.1 Report grayscale mask/count/clusters and classify empty, all-black, multi-cluster, wide, and valid track observations.
- [x] 5.2 Add independent position/derivative filters with derivative reset on start/reacquisition and bounded ambiguity handling.
- [x] 5.3 Add curve-aware continuous base-speed scheduling, correction magnitude/slew limiting, and inner-wheel speed floor.
- [x] 5.4 Extend line-following host regressions for topology, reacquisition, derivative kick, scheduling, slew, and hard clamps.

## 6. M6 Telemetry and Host Automation

- [x] 6.1 Add fixed-size 20 ms M6 snapshot capture, bounded ring/drop counters, and background integer CSV formatting/transmission.
- [x] 6.2 Add UART whole-line nonblocking enqueue and `log,m6`/`log,off` protocol control without blocking scheduled tasks.
- [x] 6.3 Add `scripts/m6_tune.py` discovery, transaction, recording, scoring, conservative coordinate search, abort, rollback, and persistence commands.
- [x] 6.4 Add automated tests for host protocol handling, scoring/acceptance limits, one-family search, hard-stop rollback, and telemetry loss/backpressure.

## 7. Build and Download Workflow

- [x] 7.1 Add `scripts/flash.ps1` Keil/project/probe/serial preflight, hidden SWD download, `Verify OK` enforcement, and post-flash identity checks.
- [x] 7.2 Enforce exactly one identical retry and then a manual-download stop with no GUI or automatic BSL fallback; add mocked script tests.
- [x] 7.3 Register all new sources in Keil, run the full host suite, run Keil 6.22 Rebuild, and verify zero errors/warnings plus reserved-sector placement.

## 8. Hardware Validation and Evidence

- [ ] 8.1 Download the compatibility firmware and record the reproducible 160 mm/s baseline with firmware/schema/revision metadata.
- [ ] 8.2 Reconfirm PB11 phase two, then validate raised-wheel 100/160/220/300 mm/s and low-speed ground heading/start/stop behavior.
- [ ] 8.3 Progressively validate line following at 100/160/180/200/220 mm/s and complete the five-lap 220 mm/s hard gate before any 235/250 mm/s extension run.
- [ ] 8.4 Save the accepted parameters, verify reset and power-cycle restoration, and synchronize the accepted values into compiled defaults.
- [x] 8.5 Update the dated M6 validation record and `STATE.md` with strictly evidence-based build, download, bench, and on-car results.
