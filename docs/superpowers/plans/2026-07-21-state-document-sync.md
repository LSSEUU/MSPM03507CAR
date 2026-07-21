# State Document Synchronization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the stale `CODEREADME.md` with an evidence-based `STATE.md` and enforce synchronized state updates for future engineering changes.

**Architecture:** `STATE.md` is the sole current-state record, while approved specifications and plans retain design history. A repository-wide `AGENTS.md` provides semantic maintenance rules, and a versioned Git pre-commit hook provides a mechanical staged-file check. A PowerShell setup script activates the hook in each clone through local Git configuration.

**Tech Stack:** Markdown, POSIX shell for Git for Windows hooks, PowerShell 5+, Git local configuration.

## Global Constraints

- Do not modify firmware behavior in this migration.
- Do not add the user's unrelated untracked document under `docs/` to any commit.
- Treat source code and generated project configuration as the primary evidence for current state.
- Never claim a target build or hardware test without corresponding evidence.
- Use `STATE.md` as the only current implementation status document after migration.
- Any staged engineering change in the monitored scope must include a staged `STATE.md` change.
- Git hook checks operate on the staged index only.

---

## File Structure

- Delete `CODEREADME.md`: remove the stale duplicate source of truth.
- Create `STATE.md`: current functionality, hardware, validation evidence, known risks, and active milestone.
- Create `AGENTS.md`: repository-wide maintenance requirements for agents.
- Create `.githooks/pre-commit`: staged-file enforcement.
- Create `scripts/setup_git_hooks.ps1`: per-clone hook activation and verification.

### Task 1: Replace the stale status document

**Files:**
- Delete: `CODEREADME.md`
- Create: `STATE.md`

**Interfaces:**
- Consumes: current behavior from `Source/empty.c`, `Source/ti_msp_dl_config.c/.h`, `Hardware/`, `Control/`, the Keil project, and `docs/接线说明.md`.
- Produces: `STATE.md`, the sole current-state document consumed by engineers, agents, and the pre-commit policy.

- [ ] **Step 1: Record the failing migration precondition**

Run:

```powershell
if ((Test-Path STATE.md) -or -not (Test-Path CODEREADME.md)) { exit 1 }
Write-Output "Expected precondition: STATE.md missing and CODEREADME.md present"
```

Expected: exit 0 and the expected-precondition message.

- [ ] **Step 2: Delete `CODEREADME.md` and create `STATE.md` with this complete content**

Use `apply_patch` to delete `CODEREADME.md` and add:

```markdown
# MSPM03507CAR 当前状态

更新日期：2026-07-21

## 1. 文档职责

本文是仓库当前实现状态的唯一权威文档，描述“代码现在是什么”，不保存已经失效的历史说明。设计意图位于 `docs/superpowers/specs/`，实施步骤位于 `docs/superpowers/plans/`，历史状态通过 Git 提交追溯。

修改代码、外设配置、工程配置、构建或测试脚本时，必须在同一提交中同步更新本文。没有构建或上板证据的功能必须标记为未验证。

状态术语：

- **已实现**：源码包含完整逻辑。
- **已接入**：当前主程序调用该逻辑。
- **语法已验证**：记录过实际语法检查命令和结果。
- **已编译**：存在目标工具链成功构建证据。
- **已上板验证**：存在明确硬件测试步骤和结果。
- **未验证**：没有足够证据。
- **不可用**：存在空实现、未配置依赖或已知阻断问题。

## 2. 项目目标与固定硬件

项目面向控制类电子设计竞赛，当前目标是建立可复用的两轮差速小车平台，先完成外设诊断、速度闭环、航向控制和巡线，再在赛题发布后增加应用层任务。

固定硬件：

- 天猛星 MSPM0G3507 48Pin，32 MHz，裸机运行。
- MG310 双直流减速电机和双路 AB 相编码器。
- D153C/TB6612 双路电机驱动板。
- 感为八路灰度传感器。
- JY61P 陀螺仪。
- HC-04 串口透传模块。
- SSD1306 OLED 和状态 LED。

## 3. 当前主程序

入口为 `Source/empty.c`。系统初始化 GPIO、UART0、双路 PWM、编码器 GPIO 中断、灰度接口、JY61P UART2 和 SysTick，随后进入约 5 ms 的轮询循环。

当前演示流程：

1. 串口发送 `G` 启动路径。
2. 两轮使用固定 PWM 400 开环直行。
3. 左编码器累计绝对值达到 1000 后停车约 400 ms。
4. 使用 JY61P yaw 和 PID 原地右转 90°。
5. 转向误差小于 1.5°并连续稳定约 75 ms 后停车约 200 ms。
6. 状态机继续下一段。

代码在 `step >= 5` 时才结束，因此实际执行五段直线和四次转向，不是标准四边形。这是已知缺陷。

## 4. 功能状态矩阵

| 功能 | 源码 | 主程序 | 构建/上板证据 | 当前结论 |
|---|---|---|---|---|
| MSPM0 板级初始化 | 已实现 | 已接入 | 仅主机语法检查 | 目标构建未验证 |
| 双电机方向与 PWM | 已实现 | 已接入 | 接线说明记录正反转和 PWM 已验证 | 可用于当前演示 |
| 双路 AB 编码器计数 | 已实现 | 左轮距离已接入 | 接线说明标记数值待验证 | 硬件精度未验证 |
| JY61P 角度解析 | 已实现 | yaw 已接入转向 | 无完整测试记录 | 硬件可靠性未验证 |
| 通用位置式 PID | 已实现 | 已接入转向 | 无闭环验收记录 | 可演示，未达到比赛基线 |
| 自动右转路径 | 已实现 | 已接入 | 无完整上板记录 | 存在五段直线缺陷 |
| UART0 调试 | 已实现 | 已接入 | 接线说明记录 HC-04 透传 | 当前主要调试入口 |
| 八路灰度扫描 | 已实现 | 仅初始化，未更新 | 无巡线记录 | 未形成巡线功能 |
| 蓝牙 APP 协议 | 部分实现 | 未接入 | DMA 配置为空 | 不可用 |
| OLED | 框架存在 | 未接入 | I2C 写入和字库为空 | 不可用 |
| LED | 框架存在 | 未接入 | 引脚表为空 | 不可用 |
| 双轮速度闭环 | 未实现 | 未接入 | 无 | 不可用 |
| 航向直行纠偏 | 未实现 | 未接入 | 无 | 不可用 |
| 灰度巡线控制 | 未实现 | 未接入 | 无 | 不可用 |
| 故障保护和看门狗 | 未实现 | 未接入 | 无 | 不可用 |

## 5. 当前引脚与外设参数

| 功能 | MCU 引脚 | 配置 |
|---|---|---|
| UART0 TX/RX | PA10 / PA11 | 115200 baud，调试及 HC-04 |
| UART2 TX/RX | PA21 / PA22 | 9600 baud，JY61P |
| 电机 A PWM | PB3 / TIMA1 CCP1 | 4 kHz，比较值 0–999 |
| 电机 B PWM | PB2 / TIMA1 CCP0 | 4 kHz，比较值 0–999 |
| 电机 A 方向 | PA14 / PA13 | AIN1 / AIN2 |
| 电机 B 方向 | PA16 / PA17 | BIN1 / BIN2 |
| 编码器 A | PA8 / PA9 | E1A / E1B，GPIO 上升沿中断 |
| 编码器 B | PB8 / PB9 | E2A / E2B，GPIO 上升沿中断 |
| 灰度地址 | PA23 / PA24 / PA26 | AD0 / AD1 / AD2 |
| 灰度输出 | PA25 | 当前代码按数字输入读取 |

电机 PWM 使用 TIMA1，32 MHz 总线时钟除以 8、周期 1000，对应约 4 kHz。

## 6. 当前串口命令

UART0 参数：115200、8 数据位、无校验、1 停止位。

| 命令 | 功能 |
|---|---|
| `G` / `g` | 启动当前自动路径 |
| `S` / `s` / 空格 | 停止电机并返回空闲状态 |
| `R` / `r` | 阻塞式单次右转 90°测试 |
| `Y` / `y` | 输出 yaw 和路径步数 |
| `E` / `e` | 输出左编码器累计值 |
| `1` / `2` | 增加 / 减少转向 KP |
| `3` / `4` | 增加 / 减少转向 KI |
| `5` / `6` | 增加 / 减少转向 KD |
| `7` / `8` | 增加 / 减少 PID 输出限幅 |
| `Z` / `z` | 将转向 PID 参数归零 |
| `0` | 恢复默认转向 PID 参数 |

## 7. 构建与验证状态

- 目标工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
- 工程包含当前 `Source/`、`Hardware/` 和 `Control/` 业务源文件。
- 仓库脚本期望 Keil 位于 `C:\Keil_v5\UV4\UV4.exe`。
- 2026-07-21 检查时，本机该路径不存在。
- 仓库内没有 AXF、HEX、MAP 或成功构建日志。
- 2026-07-21 使用 MinGW GCC 对 13 个业务和配置 C 文件执行 `-fsyntax-only`，全部返回 0；这不等价于 ARM/Keil 完整构建。
- 接线说明记录电机正反转、PWM 调速和 HC-04 透传已验证；编码器数值仍标记为待验证。

## 8. 已知问题与风险

1. 自动路径多执行一段直线。
2. 直行使用固定 PWM，只读取左编码器距离，没有速度闭环和航向纠偏。
3. `R` 命令使用阻塞循环，期间主调度无法执行其他任务。
4. PID 没有显式采样周期、微分滤波和完善的抗积分饱和策略。
5. JY61P 数据没有超时、启动有效性等待、零偏校准和掉线保护；中断共享数据也没有完整快照机制。
6. 灰度驱动已实现，但主循环不调用 `Grayscale_Update()`。
7. 蓝牙 DMA、OLED I2C/字库、LED 引脚和 APP 上报均为空或不完整。
8. `board.h` 与硬件头文件相互包含，模块边界不清晰。
9. `Source/empty.syscfg` 与生成的 `ti_msp_dl_config.c/.h` 不足以证明配置可安全重新生成，存在覆盖风险。
10. 接线说明内部存在冲突：文字表使用编码器 B 的 PB8/PB9，示意图曾写 PB4/PB5；灰度 PA25 在文档中写 ADC，而当前代码按数字输入读取。
11. 没有传感器超时、堵转、任务超期、急停优先级和看门狗闭环。

## 9. 当前里程碑

已批准的总体设计位于 `docs/superpowers/specs/2026-07-21-competition-car-platform-design.md`。

当前准备进入 M0“工程基线”：统一工程配置来源、建立可重复目标构建、修正文档和配置不一致，并为后续外设诊断提供稳定起点。尚未开始修改固件架构。

## 10. 最近验证记录

| 日期 | 验证 | 结果 |
|---|---|---|
| 2026-07-21 | 检查工程结构、调用关系、引脚配置和 Keil 收录文件 | 完成，只读分析 |
| 2026-07-21 | MinGW GCC 对全部 13 个 C 文件执行语法检查 | 全部通过，仅代表语法层面 |
| 2026-07-21 | 检查本机 Keil 和仓库构建产物 | 未发现 Keil 指定路径及 AXF/HEX |
| 2026-07-21 | 将旧状态说明迁移为 `STATE.md` | 文档迁移，无固件功能变化 |
```

- [ ] **Step 3: Verify the new document and deletion**

Run:

```powershell
$stateFile = 'STATE.md'
$required = @(
  '## 1. 文档职责',
  '## 3. 当前主程序',
  '## 4. 功能状态矩阵',
  '## 5. 当前引脚与外设参数',
  '## 7. 构建与验证状态',
  '## 8. 已知问题与风险',
  '## 9. 当前里程碑',
  '## 10. 最近验证记录'
)
$text = Get-Content -Raw -Encoding UTF8 $stateFile
$missing = $required | Where-Object { -not $text.Contains($_) }
if (-not (Test-Path $stateFile) -or (Test-Path CODEREADME.md) -or $missing.Count -gt 0) { exit 1 }
Write-Output "STATE migration PASS; required_sections=$($required.Count)"
```

Expected: exit 0 and `STATE migration PASS; required_sections=8`.

- [ ] **Step 4: Confirm no firmware files changed**

Run:

```powershell
$changed = git diff --name-only
$unexpected = $changed | Where-Object { $_ -notin @('CODEREADME.md', 'STATE.md') }
if ($unexpected) { $unexpected; exit 1 }
Write-Output "Migration scope PASS"
```

Expected: exit 0 and `Migration scope PASS`.

- [ ] **Step 5: Commit the status migration**

Run:

```powershell
git add -- CODEREADME.md STATE.md
git diff --cached --check
git commit -m "docs: replace stale code readme with state"
```

Expected: one commit deleting `CODEREADME.md` and creating `STATE.md`.

### Task 2: Add agent policy and staged-file enforcement

**Files:**
- Create: `AGENTS.md`
- Create: `.githooks/pre-commit`

**Interfaces:**
- Consumes: `STATE.md` from Task 1 and Git staged paths from `git diff --cached --name-only --diff-filter=ACMRD`.
- Produces: repository-wide semantic rules and a pre-commit program returning 0 when synchronization is satisfied or 1 when it is missing.

- [ ] **Step 1: Verify the policy and hook do not exist**

Run:

```powershell
if ((Test-Path AGENTS.md) -or (Test-Path .githooks/pre-commit)) { exit 1 }
Write-Output "Expected precondition: policy and hook missing"
```

Expected: exit 0 and the expected-precondition message.

- [ ] **Step 2: Create `AGENTS.md` with this complete content**

```markdown
# Repository Working Rules

These instructions apply to the entire repository.

## STATE.md is mandatory

`STATE.md` is the sole source of truth for the current implementation state. Whenever a task changes source code, hardware configuration, project configuration, build/debug configuration, or engineering scripts, update `STATE.md` in the same task and commit.

Update the relevant sections when a change affects:

- implemented, enabled, disabled, or removed functionality;
- the current main flow or system state;
- pins, peripherals, baud rates, timers, PWM, task periods, or wiring;
- commands, telemetry, tuning interfaces, or stored parameters;
- build requirements, generated files, or toolchain evidence;
- known defects, risks, safety behavior, or milestone status;
- actual build, syntax-check, bench-test, or on-car verification results.

Use evidence-based status terms. Do not claim target compilation from a host syntax check, and do not claim hardware verification from source inspection. If a monitored change has no functional effect, add a concise `无功能变化` entry to the latest verification record so the synchronized edit remains meaningful.

Do not bypass, disable, delete, or weaken `.githooks/pre-commit` to avoid updating `STATE.md`. New clones must run `scripts/setup_git_hooks.ps1` once.

Preserve unrelated user files and changes. In particular, never stage unrelated untracked documentation while updating project state.
```

- [ ] **Step 3: Create `.githooks/pre-commit` with this complete content**

```sh
#!/bin/sh
set -eu

changed_files=$(git diff --cached --name-only --diff-filter=ACMRD)

if [ -z "$changed_files" ]; then
    exit 0
fi

requires_state=0
triggered_files=""

while IFS= read -r changed_file; do
    case "$changed_file" in
        Source/*|Control/*|Hardware/*|System/*|App/*|Config/*|Tests/*|syscfg/*|scripts/*|.vscode/*|keil/*.uvprojx|keil/*.uvoptx|keil/*.sct|keil/*.py|*.c|*.h|*.s|*.S|*.syscfg|*.uvprojx|*.sct|*.lds|*.cmd)
            requires_state=1
            triggered_files="${triggered_files}${changed_file}
"
            ;;
    esac
done <<EOF
$changed_files
EOF

if [ "$requires_state" -eq 0 ]; then
    exit 0
fi

if printf '%s\n' "$changed_files" | grep -qx 'STATE.md'; then
    exit 0
fi

printf '%s\n' 'ERROR: engineering files changed without a staged STATE.md update.' >&2
printf '%s\n' 'Files requiring synchronization:' >&2
printf '%s' "$triggered_files" >&2
printf '%s\n' 'Update and stage STATE.md, then commit again.' >&2
exit 1
```

- [ ] **Step 4: Verify both hook paths with an isolated temporary Git index**

Run:

```powershell
$temporaryIndex = Join-Path ([System.IO.Path]::GetTempPath()) ("state-sync-" + [guid]::NewGuid().ToString('N') + '.index')
$previousIndex = $env:GIT_INDEX_FILE
try {
    $env:GIT_INDEX_FILE = $temporaryIndex
    git read-tree HEAD

    $codeBlob = ('synthetic engineering change' | git hash-object -w --stdin).Trim()
    git update-index --add --cacheinfo 100644 $codeBlob scripts/hook-test.txt
    & sh .githooks/pre-commit
    if ($LASTEXITCODE -ne 1) { throw 'Hook did not reject a code-only change.' }
    Write-Output 'Hook rejection PASS'

    $stateBlob = ('synthetic state update' | git hash-object -w --stdin).Trim()
    git update-index --cacheinfo 100644 $stateBlob STATE.md
    & sh .githooks/pre-commit
    if ($LASTEXITCODE -ne 0) { throw 'Hook rejected a synchronized change.' }
    Write-Output 'Hook acceptance PASS'
}
finally {
    if (Test-Path -LiteralPath $temporaryIndex) {
        Remove-Item -LiteralPath $temporaryIndex -Force
    }
    if ($null -eq $previousIndex) {
        Remove-Item Env:GIT_INDEX_FILE -ErrorAction SilentlyContinue
    } else {
        $env:GIT_INDEX_FILE = $previousIndex
    }
}
```

Expected: the first hook call prints the synchronization error, followed by `Hook rejection PASS` and `Hook acceptance PASS`; the wrapper exits 0.

- [ ] **Step 5: Record the new policy in `STATE.md`**

Append this row under `STATE.md` section 10 before staging:

```markdown
| 2026-07-21 | 增加状态文档同步规则和 Git 提交检查 | 无固件功能变化 |
```

Then stage and inspect the Task 2 files:

```powershell
git add -- STATE.md AGENTS.md .githooks/pre-commit
git diff --cached --check
git diff --cached --name-only
```

Expected staged files: `.githooks/pre-commit`, `AGENTS.md`, and `STATE.md`.

- [ ] **Step 6: Commit the repository policy and hook**

Run:

```powershell
git commit -m "chore: add state synchronization policy"
```

Expected: one commit creating `AGENTS.md` and `.githooks/pre-commit` and updating `STATE.md`.

### Task 3: Add per-clone hook setup and finish verification

**Files:**
- Create: `scripts/setup_git_hooks.ps1`
- Modify: `STATE.md`

**Interfaces:**
- Consumes: `.githooks/pre-commit` from Task 2 and the current Git worktree root.
- Produces: local Git setting `core.hooksPath=.githooks` and a reproducible setup command for every clone.

- [ ] **Step 1: Verify the setup script is absent**

Run:

```powershell
if (Test-Path scripts/setup_git_hooks.ps1) { exit 1 }
Write-Output "Expected precondition: setup script missing"
```

Expected: exit 0 and the expected-precondition message.

- [ ] **Step 2: Create `scripts/setup_git_hooks.ps1` with this complete content**

```powershell
$ErrorActionPreference = 'Stop'

$repositoryRootText = (& git rev-parse --show-toplevel 2>$null).Trim()
if (-not $repositoryRootText) {
    throw 'Not inside a Git repository.'
}

$repositoryRoot = (Resolve-Path -LiteralPath $repositoryRootText).Path
$currentDirectory = (Resolve-Path -LiteralPath (Get-Location).Path).Path
if ($currentDirectory -ne $repositoryRoot) {
    throw "Run this script from the repository root: $repositoryRoot"
}

$hookPath = Join-Path $repositoryRoot '.githooks\pre-commit'
if (-not (Test-Path -LiteralPath $hookPath -PathType Leaf)) {
    throw "Missing hook: $hookPath"
}

& git config --local core.hooksPath .githooks
if ($LASTEXITCODE -ne 0) {
    throw 'Failed to configure core.hooksPath.'
}

$configuredPath = (& git config --local --get core.hooksPath).Trim()
if ($configuredPath -ne '.githooks') {
    throw "Unexpected core.hooksPath: $configuredPath"
}

Write-Output 'Git hooks enabled: core.hooksPath=.githooks'
```

- [ ] **Step 3: Record the setup mechanism in `STATE.md`**

Add this paragraph to section 1 after the synchronization rule:

```markdown
每次新克隆仓库后，在仓库根目录执行 `powershell -ExecutionPolicy Bypass -File scripts/setup_git_hooks.ps1`，启用版本化的提交检查。
```

Replace the Task 2 verification row with the final row:

```markdown
| 2026-07-21 | 增加 `AGENTS.md`、版本化预提交检查和启用脚本 | hook 拒绝/接受路径均验证通过，无固件功能变化 |
```

- [ ] **Step 4: Run the setup script and verify local configuration**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup_git_hooks.ps1
$configuredPath = git config --local --get core.hooksPath
if ($configuredPath -ne '.githooks') { exit 1 }
Write-Output "Hook configuration PASS"
```

Expected: both the setup message and `Hook configuration PASS`, exit 0.

- [ ] **Step 5: Stage the setup script and final state, then run the active hook**

Run:

```powershell
git add -- scripts/setup_git_hooks.ps1 STATE.md
git diff --cached --check
& sh .githooks/pre-commit
if ($LASTEXITCODE -ne 0) { exit 1 }
git diff --cached --name-only
```

Expected staged files:

```text
STATE.md
scripts/setup_git_hooks.ps1
```

- [ ] **Step 6: Verify the final repository state**

Run:

```powershell
$requiredFiles = @('STATE.md', 'AGENTS.md', '.githooks/pre-commit', 'scripts/setup_git_hooks.ps1')
$missing = $requiredFiles | Where-Object { -not (Test-Path -LiteralPath $_) }
$oldExists = Test-Path -LiteralPath CODEREADME.md
$configuredPath = git config --local --get core.hooksPath
if ($missing.Count -gt 0 -or $oldExists -or $configuredPath -ne '.githooks') { exit 1 }
Write-Output "State synchronization implementation PASS"
```

Expected: exit 0 and `State synchronization implementation PASS`.

- [ ] **Step 7: Commit the enforcement mechanism**

Run:

```powershell
git commit -m "chore: enforce state document synchronization"
git show --stat --oneline --summary HEAD
git status --short
```

Expected: commit succeeds through the active hook. The only remaining untracked path is the user's pre-existing unrelated document under `docs/`.

## Plan Self-Review Result

- Specification coverage: all file operations, state content, agent rules, hook scope, activation, failure path, success path, and unrelated-file preservation are assigned to Tasks 1–3.
- Placeholder scan: no incomplete implementation markers are present.
- Interface consistency: the hook path is consistently `.githooks/pre-commit`, the activation setting is consistently `.githooks`, and every monitored change stages `STATE.md` before a successful commit.
