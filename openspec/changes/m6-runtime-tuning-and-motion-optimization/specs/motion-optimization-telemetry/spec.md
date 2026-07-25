## ADDED Requirements

### Requirement: Non-blocking 50 Hz M6 telemetry
The firmware SHALL provide an `M6` integer CSV telemetry mode that captures one motion snapshot every 20 ms when 50 Hz logging is enabled. Each snapshot SHALL identify the active parameter revision and include commanded and shaped wheel targets, raw and filtered wheel speeds, wheel errors, PI terms, integrator values, encoder deltas, control-loop elapsed time, PWM outputs, saturation state and duration, yaw, yaw-rate value and source, line active mask, raw and filtered line position, filtered derivative, scheduled base speed, correction, fault state, scheduler deadline count, UART drop count, and telemetry drop count. The 20 ms task SHALL only capture fixed-size numeric data into a bounded queue; CSV formatting and UART transmission SHALL run outside the time-critical task.

#### Scenario: Telemetry runs at 50 Hz without delaying control
- **WHEN** M6 logging is enabled with a 20 ms period while the vehicle is running
- **THEN** the firmware captures one snapshot per 20 ms release and performs formatting and UART transmission in background service time without blocking the 5 ms, 10 ms, or 20 ms control tasks

#### Scenario: UART backpressure does not block control
- **WHEN** the telemetry queue is full or UART transmission cannot keep pace with snapshot production
- **THEN** the firmware drops telemetry data without waiting, increments a cumulative telemetry-drop counter, and exposes that counter in a subsequent transmitted snapshot

#### Scenario: Logging can be stopped safely
- **WHEN** the host disables M6 logging or an emergency stop is received
- **THEN** the firmware stops producing new M6 snapshots without delaying or weakening the existing Safety stop path

### Requirement: Comparable tuning evidence
The tuning workflow SHALL record a parameterized 160 mm/s compatibility baseline before optimized control behavior is enabled. Candidate and baseline logs SHALL include firmware identity, parameter schema version, active parameter revision, test mode, requested speed, start time, and all M6 telemetry needed to reproduce the score.

#### Scenario: Baseline is captured before optimized candidates
- **WHEN** an M6 tuning session has no compatible 160 mm/s baseline for the connected firmware identity and parameter schema
- **THEN** the host tool records the compatibility baseline before it evaluates any optimized candidate

#### Scenario: Incompatible evidence is rejected
- **WHEN** a candidate log has a different firmware identity, parameter schema, test mode, or requested speed from its comparison baseline
- **THEN** the host tool rejects the comparison and does not accept or save the candidate

### Requirement: Deterministic candidate scoring
The host tuning tool SHALL score each candidate from rise time, overshoot, steady-state error, error zero crossings, PWM saturation, heading RMS, line-position RMS, correction jerk, line-loss events, scheduler deadline misses, communication loss, and telemetry drops. It SHALL change only one parameter family per candidate, perform at most three conservative coordinate-search rounds, and accept a candidate only when its score improves by at least 5 percent over the last accepted configuration without violating any safety or acceptance limit.

#### Scenario: Candidate improves sufficiently
- **WHEN** a candidate changes one parameter family, improves the comparable score by at least 5 percent, and violates no safety or acceptance limit
- **THEN** the host tool marks that candidate as the new last accepted configuration for the next search step

#### Scenario: Candidate improvement is insufficient
- **WHEN** a candidate improves the comparable score by less than 5 percent or changes more than one parameter family
- **THEN** the host tool rejects the candidate and restores the last accepted configuration before another candidate is evaluated

#### Scenario: Search reaches its round limit
- **WHEN** three coordinate-search rounds have completed
- **THEN** the host tool stops automatic candidate generation and reports the best accepted configuration without starting a fourth round

### Requirement: Safety-triggered stop and rollback
The host tuning tool SHALL terminate the active candidate on any firmware fault, line-loss event, sustained PWM saturation, scheduler deadline miss, UART or telemetry loss, or user emergency stop. It SHALL request the existing emergency-stop path when communication remains available, SHALL never clear a latched fault automatically, and SHALL restore the last accepted parameters only after the firmware reaches an eligible safe `IDLE` state; until then it SHALL retain a visible pending-rollback state.

#### Scenario: Unsafe candidate is stopped while communication is available
- **WHEN** an unsafe condition is detected and the UART link is still responsive
- **THEN** the host tool sends the emergency-stop command, rejects the candidate, and does not resume testing until the vehicle is safe and the last accepted parameters have been restored

#### Scenario: Communication is lost during a candidate
- **WHEN** UART communication or required telemetry is lost during an active candidate
- **THEN** the host tool terminates the session, relies on the firmware's existing heartbeat and Safety timeout to stop motion, records rollback as pending, and does not assume that parameters were restored

#### Scenario: Fault remains latched after stopping
- **WHEN** a candidate causes a latched firmware fault
- **THEN** the host tool preserves the fault for user inspection and defers parameter restoration until the user clears the fault and the firmware reports an eligible `IDLE` state
