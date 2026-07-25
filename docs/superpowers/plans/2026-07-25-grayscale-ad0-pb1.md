# Grayscale AD0 PB1 Remap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the Ganwei analog grayscale module AD0 address input from PA23/H13-11 to PB1/H13-6, remove the PA23 slow-rise path, and return line following to a safely locked calibration state.

**Architecture:** PB1 becomes a normal push-pull GPIO address output while PA24 and PA26 remain AD1 and AD2. A single address-selection helper writes all three address bits through their own GPIO ports, so synchronous and time-sliced scans share identical behavior and no cross-port mask is possible. Existing ADC acquisition and line-control interfaces remain unchanged; the compiled calibration lock returns to zero until new independent eight-channel data is collected.

**Tech Stack:** MSPM0G3507 DriverLib C, ARM Compiler 6.22/Keil µVision, Python `unittest`, host GCC/Clang control-model tests, Markdown engineering records.

## Global Constraints

- Work only in the existing isolated worktree `C:\Users\Administrator\Desktop\DXS\MSPM03507CAR\User_Project\MSPM03507CAR-analog` on branch `codex/restore-analog-grayscale`.
- Preserve the existing Keil project file and do not regenerate frozen board configuration from old SysConfig input.
- AD0 is PB1/H13-6; AD1 remains PA24; AD2 remains PA26; OUT remains PA25/ADC0 A0_2.
- PA23 and PA21 must remain unconfigured high-impedance pins; UART2 remains receive-only on PA22.
- Keep bounded ADC polling, eight samples per channel, diagnostic reporting, line confirmation, lost-line handling, emergency stop, WWDT, and infrared entry behavior.
- Default `GRAYSCALE_CALIBRATION_VERIFIED` must be `0U`; no line action may start before new calibration is compiled and verified.
- Any source or hardware-state change must update `STATE.md` in the same implementation commit.
- Do not stage ignored Keil build products or unrelated user changes.

---

### Task 1: Remap AD0 and lock invalid calibration

**Files:**
- Modify: `Tests/test_board_configuration.py:94-111`
- Modify: `Tests/test_m5_line_follow.py:49-55`
- Modify: `Source/ti_msp_dl_config.h:109-118`
- Modify: `Hardware/grayscale.c:10-233`
- Modify: `Config/grayscale_config.h:12-24`
- Modify: `STATE.md`
- Modify: `docs/接线说明.md`
- Modify: `docs/天猛星V2_小车主控板_工程参考.md`
- Modify: `docs/2026-07-25-感为模拟灰度模块恢复记录.md`

**Interfaces:**
- Consumes: `GRAY_AD0_PORT`, `GRAY_AD0_PIN`, `GRAY_AD0_IOMUX`, `GRAY_AD1_*`, `GRAY_AD2_*`, `Gray_ReadADC(uint16_t *)`, and the existing `Grayscale_*` public API.
- Produces: PB1-backed `GRAY_AD0_*` macros and a private `static void Gray_SelectChannel(uint8_t channel)` used by both scan paths. The public API and `GrayscaleSnapshot_t` remain unchanged.

- [ ] **Step 1: Write the failing PB1 board/configuration test**

Replace the PA23-specific assertions in `test_grayscale_uses_pa25_adc_with_bounded_scanning` and rename it to `test_grayscale_uses_pb1_address_and_pa25_adc_with_bounded_scanning`. Keep the ADC assertions and add these hand-checked invariants:

```python
self.assertRegex(header, r"#define\s+GRAY_AD0_PORT\s+\(GPIOB\)")
self.assertRegex(header, r"#define\s+GRAY_AD0_PIN\s+\(DL_GPIO_PIN_1\)")
self.assertRegex(header, r"#define\s+GRAY_AD0_IOMUX\s+\(IOMUX_PINCM13\)")
self.assertIn("DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX)", driver)
self.assertIn("DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN)", driver)
self.assertIn("DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN)", driver)
self.assertNotIn("GRAY_AD0_PIN | GRAY_AD1_PIN", driver)
self.assertNotIn("DL_GPIO_enableHiZ(GRAY_AD0_IOMUX)", driver)
self.assertNotIn("DL_GPIO_RESISTOR_PULL_UP", driver)
self.assertNotIn("GRAY_ASYNC_WAIT_AD0", driver)
self.assertNotIn("GRAY_AD0_RISE_TIMEOUT_MS", driver)
```

The production break caught by this test is mapping AD0 back to PA23 or reintroducing a shared GPIO-port mask after AD0 moves to GPIOB.

- [ ] **Step 2: Make the default calibration-lock test exercise the real configuration**

Change `test_unverified_compile_time_calibration_locks_motion` to compile `m5_line_lock_test.c` without a command-line override:

```python
def test_unverified_compile_time_calibration_locks_motion(self):
    self._compile_and_run(ROOT / "Tests" / "m5_line_lock_test.c")
```

The production break caught by this test is shipping a default firmware whose configuration header still enables the invalid paired calibration.

- [ ] **Step 3: Run the two tests and verify RED**

Run:

```powershell
python -m unittest Tests.test_board_configuration.FrozenBoardConfigurationTests.test_grayscale_uses_pb1_address_and_pa25_adc_with_bounded_scanning Tests.test_m5_line_follow.M5LineFollowTests.test_unverified_compile_time_calibration_locks_motion -v
```

Expected: both tests fail. The board test must report missing GPIOB/PB1/PINCM13 or retained PA23 Hi-Z behavior; the lock test must fail `assert(!snapshot.calibration.compiled_verified)` because the current header still sets the lock to `1U`.

- [ ] **Step 4: Change the frozen board macros to PB1**

Set the AD0 definitions in `Source/ti_msp_dl_config.h` to:

```c
#define GRAY_AD0_PORT                                                   (GPIOB)
#define GRAY_AD0_PIN                                             (DL_GPIO_PIN_1)
#define GRAY_AD0_IOMUX                                          (IOMUX_PINCM13)
```

Leave AD1, AD2, OUT, and ADC definitions unchanged.

- [ ] **Step 5: Replace the PA23 state machine with one push-pull address helper**

In `Hardware/grayscale.c`, delete `GRAY_AD0_RISE_TIMEOUT_MS`, `GRAY_ASYNC_WAIT_AD0`, `gray_async_deadline_ms`, `Gray_SetAD0`, and `Gray_SelectChannelAsync`. Replace the address selection code with:

```c
static void Gray_SelectChannel(uint8_t channel)
{
    if ((channel & 0x01U) != 0U) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    if ((channel & 0x02U) != 0U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if ((channel & 0x04U) != 0U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    delay_cycles(GRAY_CH_SWITCH_DELAY_CYCLES);
}
```

Call `Gray_SelectChannel(channel)` before synchronous ADC sampling. In `Grayscale_Task`, make the `GRAY_ASYNC_SELECT` branch call the same helper and then set `gray_async_phase = GRAY_ASYNC_SAMPLE`; retain ADC failure handling and scan completion behavior.

- [ ] **Step 6: Initialize address outputs by physical GPIO port**

Replace the PA23 input/pull-up setup with normal output setup:

```c
DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX);
DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN | GRAY_AD2_PIN);
DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN);
DL_GPIO_enableOutput(GRAY_AD1_PORT, GRAY_AD1_PIN | GRAY_AD2_PIN);
```

Do not configure PA23 anywhere in the grayscale driver. Keep the existing scan order and ADC averaging unchanged.

- [ ] **Step 7: Return the compiled calibration to the safe default**

Set this value in `Config/grayscale_config.h`:

```c
#define GRAYSCALE_CALIBRATION_VERIFIED          0U
```

Retain the old arrays only as locked historical values until the calibration script replaces them; update their comment to state that they are invalid paired data and must not be unlocked.

- [ ] **Step 8: Update current-state and wiring documentation**

Update `STATE.md`, `docs/接线说明.md`, and `docs/天猛星V2_小车主控板_工程参考.md` so the active mapping is `AD0=PB1/H13-6`, PA23 is unused/high impedance, and calibration is locked. Append the following evidence boundary to `docs/2026-07-25-感为模拟灰度模块恢复记录.md`:

```text
黑线移动和固定 AD0 高电平测试确认原 PA23 方案只选择到同一奇偶半组；模块 AD0 固定接 3.3 V 后偶数物理通道正常，因此模块地址输入正常。现改用 PB1/H13-6 推挽输出，原成对标定数据作废并把验证锁恢复为 0；重新接线、独立八路响应和重新标定尚待上板完成。
```

Do not claim the PB1 hardware path is verified before the user downloads and repeats the physical test.

- [ ] **Step 9: Run targeted tests and verify GREEN**

Run the same command from Step 3. Expected: both tests pass.

- [ ] **Step 10: Run the full host regression suite**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
```

Expected: all discovered Python/host-C tests pass with no failures or errors.

- [ ] **Step 11: Review and commit the coherent safe remap**

Run `git diff --check`, inspect `git diff --stat` and `git diff`, then commit only the listed source, tests, state, and documentation:

```powershell
git add -- Tests/test_board_configuration.py Tests/test_m5_line_follow.py Source/ti_msp_dl_config.h Hardware/grayscale.c Config/grayscale_config.h STATE.md docs/接线说明.md docs/天猛星V2_小车主控板_工程参考.md docs/2026-07-25-感为模拟灰度模块恢复记录.md
git commit -m "fix: remap grayscale AD0 to PB1"
```

---

### Task 2: Build the target and record delivery evidence

**Files:**
- Modify: `STATE.md`
- Modify: `docs/2026-07-25-感为模拟灰度模块恢复记录.md`

**Interfaces:**
- Consumes: the PB1 remap committed by Task 1, `scripts/test.ps1`, and `scripts/build.ps1 -Mode Rebuild`.
- Produces: fresh host-test and Keil 6.22 evidence plus an unambiguous manual wiring/download handoff. No firmware API changes.

- [ ] **Step 1: Run fresh full verification**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
```

Expected: host suite has zero failures/errors; Keil log contains `0 Error(s), 0 Warning(s)` and the script reports non-empty AXF and HEX byte counts.

- [ ] **Step 2: Record exact evidence without claiming hardware success**

Update `STATE.md` and the dated restoration record with the exact test count, Keil code/data summary, AXF/HEX byte counts, branch commit, and this boundary:

```text
PB1 固件已通过主机回归和目标构建，但尚未证明车上 AD0 已改接 H13-6、八路可独立选择或新标定有效；默认验证锁保持 0，循迹启动应拒绝。
```

- [ ] **Step 3: Re-run documentation and repository checks**

Run:

```powershell
git diff --check
git status --short
git diff -- STATE.md docs/2026-07-25-感为模拟灰度模块恢复记录.md
```

Expected: only the two evidence documents are modified; ignored `keil/Objects` and build logs are not staged.

- [ ] **Step 4: Commit build evidence**

```powershell
git add -- STATE.md docs/2026-07-25-感为模拟灰度模块恢复记录.md
git commit -m "docs: record PB1 grayscale build evidence"
```

- [ ] **Step 5: Prepare the hardware handoff**

Tell the user to power off, remove the temporary fixed-3.3-V AD0 connection, wire module AD0 to PB1/H13-6, leave PA23/H13-11 disconnected, and manually download the newly built target. The first on-board acceptance is diagnostic only: keep motors disabled, use UART `q`, and prove all eight physical probes affect only their corresponding ADC slot before collecting new calibration.
