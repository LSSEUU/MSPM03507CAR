## ADDED Requirements

### Requirement: Grouped runtime control parameter model
The system SHALL expose one `ControlParams_t` snapshot containing `wheel`, `heading`, `line`, and `remote` groups. It SHALL maintain separate compiled-default, active, and staged snapshots, together with a schema version, monotonically advancing parameter revision, load source, and staged-dirty indication. Control tasks SHALL read only the active snapshot.

The public parameter keys SHALL be:

- `wheel.left.kp`, `wheel.left.ki`, `wheel.right.kp`, `wheel.right.ki`, `wheel.filter_alpha`, `wheel.accel`, `wheel.decel`, `wheel.jerk`, `wheel.left.ks`, `wheel.left.kv`, `wheel.right.ks`, `wheel.right.kv`, and `wheel.diag_target`;
- `heading.straight.kp`, `heading.straight.kd`, `heading.turn.kp`, `heading.turn.kd`, `heading.rate_alpha`, `heading.correction_limit`, `heading.turn_speed`, and `heading.smooth_zone`;
- `line.speed_min`, `line.speed_max`, `line.position_alpha`, `line.derivative_alpha`, `line.kp`, `line.kd`, `line.correction_limit`, `line.correction_slew`, `line.curve_gain`, and `line.lost_search_speed`; and
- `remote.speed_low`, `remote.speed_medium`, `remote.speed_high`, and `remote.heading_hold`.

#### Scenario: Control code observes only committed values
- **WHEN** a client changes one or more staged parameters without applying them
- **THEN** every control task continues to use the unchanged active snapshot and the staged-dirty indication is set

#### Scenario: Runtime metadata is observable
- **WHEN** a client gets `meta.schema`, `meta.revision`, `meta.source`, or `meta.dirty`
- **THEN** the system returns the current schema version, active revision, active load source, or staged-dirty state respectively

### Requirement: Conservative parameter validation
The system MUST reject non-finite values and MUST validate the complete staged snapshot before activation. Wheel proportional gains SHALL be in `[0,3]`, wheel integral gains in `[0,10]`, and filter alpha values in `[0.05,1]`. Configured nonzero speeds SHALL be in `[50,400]` mm/s; `wheel.diag_target` SHALL additionally permit zero and negative values whose absolute nonzero magnitude is in that range. Acceleration and deceleration SHALL be in `[100,2000]` mm/s^2, jerk SHALL be in `[1000,20000]` mm/s^3, and heading and line correction limits SHALL be in `[0,250]` mm/s. All other numeric keys SHALL be finite and within their compile-time descriptor-table bounds, `remote.heading_hold` SHALL be `0` or `1`, and `line.speed_min` SHALL NOT exceed `line.speed_max`.

Irrespective of configurable values, the final target sent to either wheel controller MUST remain within `[-700,700]` mm/s.

#### Scenario: Boundary values are accepted
- **WHEN** every staged value is finite, lies on or within its declared inclusive bounds, and all cross-field constraints hold
- **THEN** validation succeeds and the snapshot is eligible for atomic application

#### Scenario: Invalid set is side-effect free
- **WHEN** a client attempts to set an unknown key, a non-finite or malformed value, an out-of-range value, or a value for a read-only key
- **THEN** the system returns a specific error and leaves both the active snapshot and the previously staged value unchanged

#### Scenario: Cross-field validation rejects the transaction
- **WHEN** individually valid staged values form an invalid complete snapshot such as `line.speed_min` being greater than `line.speed_max`
- **THEN** apply fails without changing any active parameter or the active revision

#### Scenario: Wheel target remains inside the hard limit
- **WHEN** feedforward, heading correction, line correction, or another runtime-configurable term would produce a wheel target outside `[-700,700]` mm/s
- **THEN** the system clamps the final wheel target to that read-only hard limit

### Requirement: Atomic staging lifecycle
The `set` operation SHALL update only the named value in the staged snapshot. `apply` SHALL validate the entire staged snapshot and atomically replace the active snapshot; no task may observe a mixture of revisions. A successful apply SHALL increment the active parameter revision exactly once and clear the staged-dirty indication. `revert` SHALL copy active values into staged values without changing the active revision. `load` and `defaults` SHALL copy the selected persisted snapshot or compiled defaults into staged values only, and SHALL require a later `apply` before control behavior changes. `save` SHALL persist only the active snapshot even when staged values differ.

#### Scenario: Multiple values commit together
- **WHEN** a client stages several valid changes and issues `apply` while application is permitted
- **THEN** all changes become visible at one control boundary under one new revision and the client receives success only after that boundary

#### Scenario: Failed apply preserves both versions
- **WHEN** full-snapshot validation or the current-state gate rejects `apply`
- **THEN** the active snapshot and revision remain unchanged while the staged snapshot remains available for correction or revert

#### Scenario: Revert discards uncommitted changes
- **WHEN** a client issues `revert` after staging one or more changes
- **THEN** staged values again equal active values, the dirty indication clears, and active control behavior does not change

#### Scenario: Defaults require explicit application
- **WHEN** a client issues `defaults` while non-default active values are in use
- **THEN** compiled defaults replace only the staged snapshot and the active values remain in use until a successful `apply`

#### Scenario: Save ignores uncommitted values
- **WHEN** staged values differ from active values and an otherwise permitted `save` is issued
- **THEN** the persistent record contains the active values and active revision, not the staged changes

### Requirement: State-gated and control-boundary application
Normal `apply` operations SHALL be accepted only in fault-free `IDLE`. During an armed wheel-speed diagnostic, `apply` SHALL permit changes only to `wheel.left.kp`, `wheel.left.ki`, `wheel.right.kp`, `wheel.right.ki`, and `wheel.diag_target`; any other staged difference SHALL cause the whole apply to fail. An accepted diagnostic apply SHALL take effect at a 5 ms wheel-control boundary, preserve the current ramped target as the target-shaper starting point, clear affected controller integral state, and reset the diagnostic statistics window.

#### Scenario: Normal apply is rejected outside IDLE
- **WHEN** a client requests a normal apply while the system is not in fault-free `IDLE`
- **THEN** the system returns `err,<id>,state` without changing active values or revision

#### Scenario: Eligible diagnostic update is synchronized
- **WHEN** an armed wheel-speed diagnostic is running and the only staged differences are eligible wheel gains or diagnostic target
- **THEN** the system commits them together at the next 5 ms boundary, preserves target continuity, clears the affected integral state, and restarts diagnostic statistics

#### Scenario: Ineligible diagnostic update is atomic
- **WHEN** a wheel-speed diagnostic is running and the staged snapshot also changes any non-eligible key
- **THEN** the system rejects the entire apply and no staged change becomes active

### Requirement: Machine-readable transaction protocol
UART0 SHALL accept transaction frames terminated by CR, LF, or CRLF and containing no spaces. Except for numeric signs and decimal points in values, command tokens SHALL be lowercase ASCII. A request SHALL be no more than 60 characters excluding its terminator and SHALL use one of these forms:

```text
:<id>,get,<key>
:<id>,set,<key>,<value>
:<id>,apply
:<id>,revert
:<id>,load
:<id>,defaults
:<id>,save
:<id>,log,<mode>,<period_ms>
```

`<id>` SHALL be an unsigned decimal value in `[0,65535]` and SHALL be echoed in the response. Every response SHALL be no more than 60 characters excluding its terminator and SHALL begin with `ok,<id>` on success or `err,<id>,<code>` on failure. Error codes SHALL distinguish at least `frame`, `too_long`, `verb`, `key`, `value`, `range`, `readonly`, `state`, and `storage`. If a usable ID cannot be parsed, the error response SHALL use ID `0`. Numeric responses SHALL use a bounded canonical decimal representation of no more than six significant digits.

`get` of a writable parameter SHALL return its active and staged values; `set` SHALL echo the staged value; `apply` SHALL return the resulting revision; and persistence operations SHALL report their selected source or generation. `log,m6,<period_ms>` SHALL enable the M6 stream only for a supported period of at least 20 ms, while `log,off,0` SHALL disable it.

#### Scenario: Complete transaction succeeds
- **WHEN** a valid request frame with ID `42` is received with a supported verb and arguments
- **THEN** exactly one bounded response beginning `ok,42` is emitted after the operation has completed

#### Scenario: Fragmented input is reassembled
- **WHEN** a valid transaction frame arrives over multiple UART receive interrupts before its terminator
- **THEN** the parser retains the partial frame and executes the transaction exactly once after termination

#### Scenario: Concatenated frames remain distinct
- **WHEN** multiple valid terminated frames arrive in one receive burst
- **THEN** the parser executes each frame once in arrival order and emits one response carrying each frame's own ID

#### Scenario: Oversized or malformed frame is contained
- **WHEN** a request exceeds 60 characters or violates its field count, casing, whitespace, ID, key, or value grammar
- **THEN** the parser discards that frame through the next terminator, emits one bounded `err` response, and remains ready for the following frame

#### Scenario: Logging period is bounded
- **WHEN** a client requests `log,m6` with a period shorter than 20 ms or requests an unsupported logging mode
- **THEN** the system returns a validation error and leaves the current logging configuration unchanged

### Requirement: Legacy command and emergency-stop compatibility
All existing single-character UART commands SHALL retain their established behavior when the transaction parser is idle. A leading `:` SHALL select transaction parsing until a terminator or frame error. Uppercase `S` and the space character MUST be recognized in the UART RX ISR in every parser state, immediately request braking, latch `FAULT`, and abandon any partial transaction before normal command dispatch.

#### Scenario: Existing single-character command remains available
- **WHEN** a legacy command other than emergency stop is received while no transaction frame is active
- **THEN** it follows the pre-M6 command path with unchanged arming, safety, and response behavior

#### Scenario: Emergency stop preempts a partial transaction
- **WHEN** uppercase `S` or space is received while a transaction frame is incomplete
- **THEN** ISR-level emergency braking and fault latching occur immediately, the partial frame is discarded, and no parameter operation from it is applied

#### Scenario: Transaction traffic cannot bypass safety arbitration
- **WHEN** any transaction stages, applies, loads, or saves parameters
- **THEN** all resulting motor requests still pass through the existing Safety arbitration and existing fault and action-arming rules remain authoritative

### Requirement: Safety configuration remains read-only
Encoder physical limits, encoder jump and stall thresholds, sensor and action timeouts, remote-heartbeat limits, watchdog configuration, PWM hard limits, and other safety gates SHALL NOT be members of the writable runtime parameter table. Attempts to address any exposed safety metadata with `set` SHALL fail as read-only.

#### Scenario: Safety threshold cannot be tuned over UART
- **WHEN** a client attempts to set an encoder limit, stall threshold, timeout, watchdog, PWM hard limit, or other safety-boundary key
- **THEN** the system returns `err,<id>,readonly` or `err,<id>,key` and no runtime or persistent state changes
