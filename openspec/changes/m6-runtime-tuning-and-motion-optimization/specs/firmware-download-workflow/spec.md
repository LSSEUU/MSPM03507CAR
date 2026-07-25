## ADDED Requirements

### Requirement: Keil and CMSIS-DAP preflight
The command-line download workflow SHALL verify that the configured Keil 6.22 executable and project target exist, that the expected Horco CMSIS-DAP probe is enumerated, and that the probe serial number matches the configured serial number before it starts a download. A failed preflight SHALL perform no erase or programming operation.

#### Scenario: Preflight succeeds
- **WHEN** Keil 6.22, the configured project target, and the expected Horco CMSIS-DAP serial number are all available
- **THEN** the workflow proceeds with the configured Keil CMSIS-DAP/SWD download command

#### Scenario: Probe is absent or mismatched
- **WHEN** no Horco CMSIS-DAP probe is enumerated or its serial number differs from the configured serial number
- **THEN** the workflow reports the failed preflight and exits before invoking any download command

### Requirement: Identical-command two-attempt limit
The workflow SHALL invoke the configured command-line Keil/CMSIS-DAP/SWD download command at most twice. If the first invocation fails, it SHALL repeat the exact same executable, arguments, target, probe, and configuration once; it SHALL not try a third invocation, alter the command, switch probe settings, or use GUI automation.

#### Scenario: First download attempt succeeds
- **WHEN** the first configured download command succeeds
- **THEN** the workflow performs no retry and continues to download verification

#### Scenario: First download attempt fails
- **WHEN** the first configured download command fails for any reason
- **THEN** the workflow invokes the identical command one more time without changing method or configuration

#### Scenario: Second download attempt fails
- **WHEN** the identical second command also fails
- **THEN** the workflow stops, reports both attempts, and instructs the user to download manually without launching a GUI or trying another variant

### Requirement: Download and startup verification
The workflow SHALL treat programming as successful only when the successful command exits with the expected success status and its download log contains `Verify OK`. It SHALL then reconnect to the application UART and confirm the expected firmware identity, parameter schema version, and parameter-load source reported by the running firmware. Missing or mismatched startup evidence SHALL be reported as an unconfirmed download rather than success.

#### Scenario: Program and startup verification both pass
- **WHEN** the download command succeeds with `Verify OK` and the application reports the expected firmware identity, parameter schema, and load source after startup
- **THEN** the workflow reports a confirmed firmware download and records the verification evidence

#### Scenario: Verify OK is missing
- **WHEN** a download invocation exits but its log does not contain `Verify OK`
- **THEN** that invocation is treated as failed and consumes one of the two permitted attempts

#### Scenario: Application identity cannot be confirmed
- **WHEN** programming verification passes but the UART startup response is absent or its firmware identity or parameter schema does not match the expected build
- **THEN** the workflow reports that programming was verified but application startup is unconfirmed and does not claim that the new firmware is running

### Requirement: SWD-only automatic workflow
The automatic workflow SHALL use the configured Keil/CMSIS-DAP/SWD path and SHALL not automatically switch to ROM BSL, simulate board-key input, or ask the user to enter BSL before an SWD build or download. ROM BSL SHALL remain a separately initiated recovery procedure.

#### Scenario: SWD download fails twice
- **WHEN** both permitted SWD download attempts fail
- **THEN** the automatic workflow stops without entering or invoking BSL and requests manual download action

#### Scenario: A separate BSL recovery is explicitly initiated
- **WHEN** the user explicitly starts a supported ROM BSL recovery procedure
- **THEN** the procedure prompts for the manual BSL-plus-reset action at the point it is ready to begin the BSL handshake, not during compilation or an earlier SWD step
