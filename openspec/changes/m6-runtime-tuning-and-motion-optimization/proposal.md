## Why

M6 requires the car to be tuned and optimized without recompiling firmware, but the current runtime interface only adjusts the diagnostic speed target and wheel PI gains in RAM. The platform also lacks safe parameter persistence and sufficient-rate telemetry to diagnose the visible line-following oscillation while increasing speed beyond the validated 160 mm/s baseline.

## What Changes

- Add a validated, grouped runtime parameter model for wheel speed, heading, line following, and remote-driving behavior.
- Add a transaction-oriented UART protocol for staging, applying, reverting, loading, saving, and restoring parameters while preserving all existing single-character commands and ISR emergency stop behavior.
- Add redundant CRC-protected parameter storage in two reserved main-flash sectors with deterministic default fallback.
- Add 50 Hz non-blocking M6 telemetry and host tools for safe candidate search, scoring, rollback, logging, build, and CMSIS-DAP/SWD download.
- Improve wheel target shaping, JY61P yaw-rate use, turn settling, line-state estimation, derivative filtering, correction slew, and curve-aware speed scheduling.
- Preserve existing Safety arbitration and make PB11 phase-two completion a prerequisite for high-speed ground validation.

## Capabilities

### New Capabilities

- `runtime-control-parameters`: Validated staged/active control parameters and the machine-readable UART transaction protocol.
- `persistent-control-parameters`: Redundant, versioned, CRC-protected parameter storage with power-loss-safe selection and default fallback.
- `motion-optimization-telemetry`: Non-blocking high-rate telemetry, host-side scoring/rollback, and safe tuning automation.
- `optimized-vehicle-control`: Runtime-configurable wheel, heading, line-following, and remote-straight control behavior for smoother and faster motion.
- `firmware-download-workflow`: Repeat-bounded command-line Keil/CMSIS-DAP download with probe and post-flash firmware verification.

### Modified Capabilities

None. The repository has no existing OpenSpec capability specifications.

## Impact

- Firmware interfaces in `Control`, `System`, `Hardware`, `App`, and `Source/main.c` gain parameter/configuration and telemetry integration points.
- The Keil scatter file reserves the final two 1 KiB main-flash sectors; normal code and generated HEX must remain below `0x0001F800`.
- New host tooling depends on the already-approved Python/pyserial environment and the existing Keil 6.22 toolchain.
- Existing safety faults, action arming, emergency stop, watchdog policy, board configuration, and legacy UART commands remain compatible.
