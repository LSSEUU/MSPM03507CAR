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

## Workflow defaults

- The user authorizes creation of isolated Git worktrees by default for planned implementation work. Do not ask for worktree consent again unless the target path or operation introduces a new material risk.
- Codex may use Git autonomously to manage in-scope repository changes. Create an intentional commit whenever a coherent implementation or verification milestone is complete.
- After a design and implementation plan are approved, proceed directly with implementation when no genuine blocker or material user decision remains. Do not ask the user to choose an execution style merely as a formality.
- Stop and ask only when required information cannot be discovered safely, a verification repeatedly fails, or proceeding would materially expand scope or risk.

## Problem investigation limits

- When a problem occurs, locate and document its probable cause before changing the implementation. Attempt a fix only when the evidence indicates a high probability that it addresses that cause.
- If two attempted fixes do not solve the same problem, stop producing further experimental outputs or speculative changes. Summarize the evidence, attempted fixes, remaining cause boundary, and the next diagnostic decision instead of trying variants blindly.
