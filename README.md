# MSPM03507CAR

立创·天猛星 MSPM0G3507-64P 两轮差速小车固件。当前已完成 M0（工程基线）和 M1（外设诊断），下一开发节点是 M2 的确定性调度、速度闭环和灰度标定。

## 从这里开始

- [`STATE.md`](STATE.md)：当前实现、接线、验证结论和已知风险的唯一事实来源。
- [`docs/README.md`](docs/README.md)：文档用途与阅读顺序。
- [`docs/接线说明.md`](docs/接线说明.md)：当前实物接线。
- [`docs/2026-07-21-M0-M1验证记录.md`](docs/2026-07-21-M0-M1验证记录.md)：M0/M1 的台架证据。
- [`docs/superpowers/specs/2026-07-21-competition-car-platform-design.md`](docs/superpowers/specs/2026-07-21-competition-car-platform-design.md)：总体设计与里程碑边界；其中历史路径不代表当前代码结构。

## 常用命令

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
```

当前 Keil 入口是 `Source/main.c`。`Source/ti_msp_dl_config.c/.h` 是受回归测试保护的唯一板级配置，不要重新引入旧 `.syscfg` 覆盖它。
