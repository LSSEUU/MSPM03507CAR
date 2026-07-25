## ADDED Requirements

### Requirement: Jerk-limited wheel targets
The wheel controller SHALL shape requested wheel speed using runtime acceleration, deceleration, and jerk limits at each 5 ms control boundary. A direction reversal SHALL pass through zero before acceleration in the opposite direction, and the final per-wheel target SHALL remain within the existing Safety limit of plus or minus 700 mm/s.

#### Scenario: Same-direction speed step is shaped
- **WHEN** a requested wheel speed changes without reversing direction
- **THEN** the shaped acceleration and change in acceleration remain within the active acceleration, deceleration, and jerk limits on every 5 ms update

#### Scenario: Wheel direction reverses
- **WHEN** a requested wheel speed changes sign
- **THEN** the controller decelerates the shaped target to zero before generating a shaped target with the opposite sign

### Requirement: Evidence-gated wheel feedforward
Each wheel SHALL support an independently configured static-friction and velocity feedforward term of the form `sign(v) * kS + kV * v`. Feedforward SHALL remain disabled with zero contribution unless host evaluation of steady-state data at 100, 160, 220, and 300 mm/s demonstrates at least a 10 percent cross-validated score improvement and no regression in overshoot or sustained saturation.

#### Scenario: Feedforward passes its evidence gate
- **WHEN** a wheel's fitted feedforward improves the cross-validated score by at least 10 percent at the required speeds and introduces neither overshoot nor sustained-saturation regression
- **THEN** the host workflow may activate the fitted `kS` and `kV` values for that wheel

#### Scenario: Feedforward fails its evidence gate
- **WHEN** the required data are incomplete, score improvement is below 10 percent, or overshoot or sustained saturation regresses
- **THEN** the active feedforward contribution for that wheel remains zero

### Requirement: Wheel-loop acceptance behavior
The optimized wheel loop SHALL expose enough state to verify its shaped target, filtered measurement, PI contribution, and saturation behavior. At each of 100, 160, 220, and 300 mm/s on the raised-wheel fixture, steady-state mean absolute speed error SHALL be no greater than 5 percent, overshoot SHALL be no greater than 10 percent, and the settled response SHALL exhibit neither sustained saturation nor sustained oscillation.

#### Scenario: Raised-wheel speed point passes
- **WHEN** a wheel is tested at one of 100, 160, 220, or 300 mm/s under the defined raised-wheel procedure
- **THEN** that speed point passes only if mean absolute steady-state error is at most 5 percent, overshoot is at most 10 percent, and no sustained saturation or oscillation is observed after settling

### Requirement: Direct JY61P yaw-rate feedback
The JY61P receiver SHALL decode `0x52` angular-velocity frames and expose timestamped yaw-rate data. Heading derivative control SHALL use the direct yaw rate while it is fresh, SHALL fall back to yaw-angle differentiation when it is stale or unavailable, and SHALL identify the selected source in M6 telemetry.

#### Scenario: Fresh angular-velocity frame is available
- **WHEN** a valid `0x52` frame is newer than the configured yaw-rate freshness limit
- **THEN** heading derivative control uses the decoded yaw-rate value and telemetry identifies the direct sensor source

#### Scenario: Angular-velocity data becomes stale
- **WHEN** no valid `0x52` frame is available within the yaw-rate freshness limit while valid yaw-angle updates continue
- **THEN** heading derivative control uses filtered yaw-angle differentiation and telemetry identifies the fallback source

### Requirement: Stationary heading calibration
Heading zero calibration SHALL accept samples only while the vehicle is commanded stopped and wheel motion remains below the stationary threshold. It SHALL reject calibration when sample dispersion exceeds the configured limit and SHALL leave the previous valid calibration active after rejection.

#### Scenario: Stable stationary calibration succeeds
- **WHEN** the required number of distinct JY61P samples is collected while the vehicle is stationary and sample dispersion is within the configured limit
- **THEN** the firmware installs the new zero reference and reports calibration success

#### Scenario: Moving or dispersed calibration is rejected
- **WHEN** wheel motion exceeds the stationary threshold or JY61P sample dispersion exceeds the configured limit during calibration
- **THEN** the firmware reports calibration failure and retains the previous valid zero reference

### Requirement: Smooth turn completion
Relative turns SHALL use a continuous near-target speed profile instead of abruptly changing from the prior 35 mm/s minimum command to zero. Turn completion SHALL still require the existing consecutive fresh-frame settling check, and the M4 `+90` degree and `-90` degree regression error SHALL remain within approximately 2 degrees.

#### Scenario: Turn enters the near-target region
- **WHEN** the remaining relative-turn error enters the configured smoothing region
- **THEN** the commanded wheel speed decreases continuously toward zero without a discrete 35 mm/s-to-zero step

#### Scenario: Right-angle regression is evaluated
- **WHEN** the optimized controller completes the standard `+90` degree or `-90` degree test
- **THEN** the action completes only after the required fresh-frame settling sequence and the measured final error is within approximately 2 degrees

### Requirement: Optional remote straight heading hold
Remote forward and reverse commands SHALL optionally hold the heading captured when straight motion begins, using the same bounded heading-correction path as other straight motion. Remote turn commands SHALL remain manual differential commands and SHALL not engage heading hold.

#### Scenario: Heading hold is enabled for remote straight motion
- **WHEN** remote heading hold is enabled and a forward or reverse command starts
- **THEN** the controller captures the current heading and applies bounded correction while that straight command remains active

#### Scenario: Remote turn is commanded
- **WHEN** a remote left or right turn command is active
- **THEN** the controller applies the requested manual differential targets without engaging straight heading hold

### Requirement: Filtered line-state estimation
The line estimator SHALL report the active sensor mask, active-sensor count, and contiguous-cluster count. It SHALL independently filter line position and position derivative, reset derivative state when line following starts or a line is reacquired, and avoid a derivative kick on that first valid sample.

#### Scenario: Line following starts on a valid line
- **WHEN** line following starts with a valid single-cluster observation
- **THEN** the position filter initializes from the observation, derivative state resets to zero, and no derivative kick is applied

#### Scenario: Line is reacquired
- **WHEN** a valid line observation follows a lost or ambiguous interval
- **THEN** derivative state is reinitialized before correction resumes and telemetry reports the current mask, count, and cluster count

### Requirement: Ambiguous line-topology handling
The line controller SHALL classify all-black, multiple-cluster, and abnormally wide observations as ambiguous topology. During a brief ambiguity within the existing line-loss validation window it SHALL retain the last reliable direction and reduce speed; if ambiguity persists through that window it SHALL enter the existing line-loss search and fault flow rather than treating the observation as a centered line.

#### Scenario: Ambiguity is brief
- **WHEN** an all-black, multiple-cluster, or abnormally wide observation clears before the existing line-loss validation window expires
- **THEN** the controller uses the last reliable direction at reduced speed and resumes filtered tracking without declaring a centered line

#### Scenario: Ambiguity persists
- **WHEN** ambiguous topology persists through the existing line-loss validation window
- **THEN** the controller enters the existing line-loss search and timeout behavior

### Requirement: Curve-aware speed and bounded correction
The line controller SHALL schedule base speed between the active minimum and maximum from filtered position error, filtered derivative, and correction utilization. It SHALL reduce speed as curvature demand increases, increase speed only on stable straight segments, limit correction magnitude and correction slew rate, and preserve a configured minimum inner-wheel speed while maintaining the final Safety wheel-target limit.

#### Scenario: Stable straight segment is detected
- **WHEN** filtered error, derivative, and correction utilization remain in their configured straight-segment bands
- **THEN** the scheduled base speed rises smoothly toward the active maximum without violating acceleration, jerk, correction-slew, or wheel-target limits

#### Scenario: Curve demand increases
- **WHEN** filtered error, derivative, or correction utilization indicates increasing curvature demand
- **THEN** the scheduled base speed falls toward the active minimum and the inner-wheel target remains at or above its configured minimum unless Safety requires a lower target or stop

### Requirement: M6 line-following improvement gate
At 160 mm/s, optimized control SHALL reduce both straight-segment line-position RMS and correction jerk by at least 25 percent relative to the parameterized compatibility baseline. High-speed ground validation SHALL remain prohibited until PB11 phase-two acceptance is complete. The hard M6 acceptance SHALL require five continuous laps at 220 mm/s with zero line loss, zero course departure, zero firmware faults, zero scheduler deadline misses, zero UART or telemetry drops, no WWDT reset, and straight-line oscillation no worse than the 160 mm/s compatibility baseline.

#### Scenario: Compatibility-speed improvement is evaluated
- **WHEN** optimized and compatibility-baseline runs at 160 mm/s are compared under the same course and test procedure
- **THEN** optimization passes this gate only if line-position RMS and correction jerk are each reduced by at least 25 percent

#### Scenario: PB11 phase two is incomplete
- **WHEN** PB11 phase-two acceptance has not been recorded
- **THEN** the workflow permits raised-wheel and low-speed diagnostic work but prohibits high-speed ground validation

#### Scenario: Hard 220 mm/s acceptance is attempted
- **WHEN** the vehicle completes the progressive 100, 160, 180, and 200 mm/s gates and then runs five continuous laps at 220 mm/s
- **THEN** M6 passes only if there is no line loss, course departure, firmware fault, scheduler deadline miss, UART or telemetry drop, or WWDT reset and straight-line oscillation is no worse than the 160 mm/s compatibility baseline

#### Scenario: Extended 250 mm/s target is attempted
- **WHEN** the hard 220 mm/s acceptance has passed
- **THEN** the workflow may attempt 235 and 250 mm/s, and records 250 mm/s as an extension pass only after three continuous laps meet the same zero-event safety criteria
