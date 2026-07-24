# MSPM03507CAR

立创·天猛星 MSPM0G3507-64P 两轮差速小车固件。M0–M5 已完成；当前主线是 PB11 红外遥控阶段二车辆验收，随后进入 M6/M7。

## 新对话最小上下文

1. 先运行 `git status --short`，识别并保留已有用户改动。
2. 阅读 [`STATE.md`](STATE.md) 的“快速启动”和“当前待办”；它是当前实现、验证结论和风险的唯一事实来源。
3. 只在任务需要时按 [`docs/README.md`](docs/README.md) 读取一份对应记录或硬件资料，不要批量加载全部验证记录和 PDF。

常用入口：

- [`docs/接线说明.md`](docs/接线说明.md)：当前实物接线。
- [`docs/PB11红外遥控器使用指南.md`](docs/PB11红外遥控器使用指南.md)：当前遥控操作与安全要求。
- [`docs/2026-07-24-PB11红外遥控接入验证记录.md`](docs/2026-07-24-PB11红外遥控接入验证记录.md)：当前红外阶段二的已通过项与待验项。
- [`docs/2026-07-24-下载链路诊断记录.md`](docs/2026-07-24-下载链路诊断记录.md)：下载失败的证据边界和固定重试流程。

## 常用命令

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
```

Keil 入口是 `Source/main.c`。`Source/ti_msp_dl_config.c/.h` 是受回归测试保护的唯一板级配置，不要重新引入旧 `.syscfg` 覆盖它。当前证据和待办以 `STATE.md` 为准，不从 README 推断工程状态。
