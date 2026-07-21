# 状态文档同步计划（已完成）

日期：2026-07-21
状态：已完成

## 目标

用 `STATE.md` 取代已经过时的 `CODEREADME.md`，并建立仓库级约束，保证代码、硬件配置、工程配置和构建工具发生变化时同步更新当前状态。

设计依据：`docs/superpowers/specs/2026-07-21-state-document-sync-design.md`。

## 最终产物

| 文件 | 作用 |
|---|---|
| `STATE.md` | 仓库当前实现状态的唯一权威记录 |
| `AGENTS.md` | 规定状态同步、证据边界和默认开发流程 |
| `.gitattributes` | 保证 Git hook 使用 LF 行尾 |
| `.githooks/pre-commit` | 工程文件变更但未同步暂存 `STATE.md` 时拒绝提交 |
| `scripts/setup_git_hooks.ps1` | 为每个新克隆启用版本化 Git hook |

旧的 `CODEREADME.md` 已删除，不再保留第二份可能过期的项目状态说明。

## 验证结果

- Hook 能拒绝没有同步 `STATE.md` 的工程变更。
- Hook 能接受工程文件与 `STATE.md` 同步提交。
- 本仓库已配置 `core.hooksPath=.githooks`。
- `STATE.md`、仓库规则、Hook 和启用脚本均已纳入版本控制。
- 迁移过程没有改变固件功能。

## 对应提交

- `f7baeec`：删除旧状态说明并创建 `STATE.md`。
- `0cd52b4`：增加状态同步规则和预提交检查。
- `ca43f4d`：增加并启用每个克隆所需的 Hook 配置脚本。

## 后续维护

当前规则以 `AGENTS.md` 和 `STATE.md` 为准。本文件只记录该迁移任务已经完成，不再保存一次性执行命令、旧版 `STATE.md` 全文、脚本源码副本或逐步实施清单；需要追溯实现细节时使用上述提交和设计文档。
