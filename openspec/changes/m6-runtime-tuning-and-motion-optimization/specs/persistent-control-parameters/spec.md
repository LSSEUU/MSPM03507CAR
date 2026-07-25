## ADDED Requirements

### Requirement: Reserved redundant flash layout
The linker configuration SHALL limit normal application flash to addresses below `0x0001F800`. Sector A at `0x0001F800` through `0x0001FBFF` and sector B at `0x0001FC00` through `0x0001FFFF` SHALL each be reserved exclusively for one 1 KiB control-parameter record. Normal code, constants, generated load regions, and ordinary HEX content SHALL NOT occupy either sector.

#### Scenario: Application image respects the reservation
- **WHEN** the production target is fully rebuilt
- **THEN** its linker map and generated image place every ordinary application region below `0x0001F800` and leave both parameter sectors unallocated

### Requirement: Versioned self-validating record format
Each persistent record SHALL contain, in order, a magic value, schema version, payload length, generation, parameter revision, canonical `ControlParams_t` payload, CRC32, and commit marker. CRC32 SHALL cover the bytes from the magic value through the final payload byte and SHALL exclude the CRC field and commit marker. A record SHALL be valid only when its magic and commit marker match, its schema version is supported, its payload length is exact for that schema, and its CRC32 verifies.

#### Scenario: Complete record validates
- **WHEN** a committed record has the expected magic, supported schema, exact payload length, and matching CRC32
- **THEN** the record is eligible for boot or runtime selection

#### Scenario: Corruption invalidates one copy
- **WHEN** any covered header or payload bit changes, the record is truncated, the schema or length is unknown, or the commit marker is absent
- **THEN** that sector is rejected without using any parameter from its record

### Requirement: Deterministic boot selection and default fallback
At boot, the system SHALL validate both sectors independently and select the valid record with the newest generation. If only one sector is valid, it SHALL select that sector. Equal valid generations SHALL resolve deterministically in favor of sector A. The selected payload SHALL initialize both active and staged parameter snapshots, preserve its parameter revision, and set the load source to `flash-a` or `flash-b`.

If neither record is valid, the system SHALL initialize active and staged snapshots from compiled defaults, set the load source to `defaults`, and report parameter-storage health as `DEGRADED`. This storage degradation SHALL NOT latch a motion fault or prevent the existing boot/self-test path from reaching `IDLE` when all other checks pass.

#### Scenario: Newest valid generation wins
- **WHEN** both sectors contain valid records with different generations
- **THEN** active and staged parameters are loaded exclusively from the newer generation and report its sector as the source

#### Scenario: Valid copy survives peer corruption
- **WHEN** one sector is invalid and the other contains a valid supported record
- **THEN** the valid record is loaded without combining fields from the invalid record

#### Scenario: Both copies invalid fall back safely
- **WHEN** neither sector contains a valid supported record
- **THEN** compiled defaults become active and staged, the source is `defaults`, storage health is `DEGRADED`, and no motion fault is latched solely for this condition

### Requirement: Runtime load stages selected persistent values
The runtime `load` operation SHALL use the same validation and generation-selection rules as boot and SHALL copy the selected payload into the staged snapshot only. It SHALL NOT change active parameters or the active revision until a later successful `apply`. If neither sector is valid, `load` SHALL stage compiled defaults, report source `defaults` with degraded storage status, and leave active control behavior unchanged.

#### Scenario: Load does not alter running configuration
- **WHEN** a client issues `load` while active parameters differ from the newest valid persisted record
- **THEN** the persisted payload and revision metadata are placed in staged state while active values and active revision remain unchanged

#### Scenario: Load falls back to staged defaults
- **WHEN** a client issues `load` and both records fail validation
- **THEN** compiled defaults are staged, active values remain unchanged, and the response identifies default fallback and degraded storage

### Requirement: Save is gated by a verified safe idle state
The system SHALL permit `save` only when the system is fault-free in `IDLE`, no action is selected, armed, or running, wheel-speed control is disabled, and Safety reports both motors braked with zero PWM request. The active snapshot SHALL pass full validation before any erase or program operation begins. A rejected save SHALL perform no flash erase or write and SHALL leave both records unchanged.

#### Scenario: Safe idle save may start
- **WHEN** all save gates are simultaneously true and the active snapshot is valid
- **THEN** the system may begin writing the active snapshot and its active revision to the inactive record sector

#### Scenario: Motion or fault blocks flash activity
- **WHEN** `save` is requested while a fault is latched, the state is not `IDLE`, an action is pending, wheel control is enabled, or either motor is not confirmed braked at zero PWM
- **THEN** the system returns `err,<id>,state` and performs no erase, program, generation change, or record selection change

#### Scenario: Staged values are not persisted
- **WHEN** a safe-idle save starts while uncommitted staged values differ from active values
- **THEN** the new record contains only the validated active snapshot and active parameter revision

### Requirement: Alternating power-loss-safe commits
Every successful save SHALL target the sector that does not contain the currently selected valid record, or sector A when no valid record exists. The system SHALL erase exactly that 1 KiB target sector, program the header and payload, program the CRC32, and program the commit marker last. Its generation SHALL be one greater than the newest valid generation, starting at `1` when no valid record exists. The previously selected record SHALL remain untouched until the replacement has been fully programmed and verified.

The system SHALL read back and revalidate the new record before reporting success or selecting it for subsequent runtime loads. A failed erase, program, readback, or validation SHALL return `err,<id>,storage`, preserve selection of the previous valid record, and SHALL NOT report the new generation as committed.

#### Scenario: Successful save alternates sectors
- **WHEN** sector A is the selected valid record and a save completes successfully
- **THEN** sector B contains the active snapshot with the next generation and becomes the selected persistent record only after readback validation

#### Scenario: Power fails before commit marker
- **WHEN** power is lost after target-sector erase or partial programming but before its commit marker is fully written
- **THEN** the interrupted target is invalid on the next boot and the previously selected record is loaded

#### Scenario: Power fails after commit marker
- **WHEN** power is lost after the complete new record including its commit marker has been written
- **THEN** the next boot validates both sectors and selects the complete record with the newer generation without relying on volatile bookkeeping

#### Scenario: Readback verification fails
- **WHEN** the newly written target does not pass the same validation used at boot
- **THEN** save reports a storage error and the prior valid record remains authoritative

### Requirement: Flash operations preserve braking and watchdog supervision
The complete erase/program/poll/readback critical path SHALL execute from RAM and SHALL NOT call flash-resident code while flash operations are active. Interrupts whose vectors or handlers require flash SHALL be disabled for that interval. The target sector alone SHALL be unprotected for the operation, and both parameter sectors SHALL be protected again on every success or failure exit.

The RAM polling path SHALL keep both motors in the pre-established braked state and SHALL service WWDT0 often enough to tolerate a 1 KiB sector erase lasting up to 200 ms without a watchdog reset. It SHALL restore the prior interrupt policy only after flash access has completed and protection has been restored.

#### Scenario: Worst-case erase remains supervised
- **WHEN** a permitted sector erase takes the specified worst-case 200 ms
- **THEN** RAM-resident polling feeds WWDT0 within its required window, no flash handler executes, and both motors remain braked

#### Scenario: Flash failure restores protection
- **WHEN** erase, programming, polling, or verification reports an error
- **THEN** the target sector is re-protected, the interrupt policy is restored, the motors remain braked, and the error is returned without selecting the failed record

### Requirement: Persistence guarantee boundaries
Parameters from the newest valid committed record SHALL survive a normal reset and loss and restoration of board power. The system SHALL NOT claim that parameter records survive a debugger-initiated mass erase, full-chip programming operation, or any download method that erases the reserved sectors; those cases SHALL recover through the normal default-fallback behavior unless independently verified for the exact download configuration.

#### Scenario: Reset retains committed parameters
- **WHEN** a successful save is followed by a normal reset or power cycle that does not erase main flash
- **THEN** boot loads the same parameter payload and revision from the newest valid generation

#### Scenario: Mass erase has no retention promise
- **WHEN** a debugger or download tool erases either or both reserved parameter sectors
- **THEN** the system validates whatever remains and safely falls back to compiled defaults when no valid record survives, without claiming download-time retention
