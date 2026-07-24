# MSPM03507CAR

立创·天猛星 MSPM0G3507-64P 两轮差速小车固件。当前已完成 M0–M4；M5 临时数字八路灰度模块已通过通道、架空控制、失线停车和运行期诊断复核，降激进调参版落地 22.4 s 表现基本正常，等待完整单圈与多圈验收。

## 从这里开始

- [`STATE.md`](STATE.md)：当前实现、接线、验证结论和已知风险的唯一事实来源。
- [`docs/README.md`](docs/README.md)：文档用途与阅读顺序。
- [`docs/接线说明.md`](docs/接线说明.md)：当前实物接线。
- [`docs/2026-07-21-M0-M1验证记录.md`](docs/2026-07-21-M0-M1验证记录.md)：M0/M1 的台架证据。
- [`docs/2026-07-24-M5临时数字灰度模块适配记录.md`](docs/2026-07-24-M5临时数字灰度模块适配记录.md)：M5 软件与临时数字灰度模块适配边界。
- [`docs/superpowers/specs/2026-07-21-competition-car-platform-design.md`](docs/superpowers/specs/2026-07-21-competition-car-platform-design.md)：总体设计与里程碑边界；其中历史路径不代表当前代码结构。

## 常用命令

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
```

当前 Keil 入口是 `Source/main.c`。`Source/ti_msp_dl_config.c/.h` 是受回归测试保护的唯一板级配置，不要重新引入旧 `.syscfg` 覆盖它。
