# STM32F103 + BQ76920 BMS

基于 STM32F103C8T6、BQ76920（带 CRC）和 RT-Thread Nano 的五串电池管理学习工程。

## 当前进度

- RT-Thread 移植、72MHz 时钟、DWT 时基和调试串口初始化。
- 软件 I2C，以及 BQ76920 寄存器读写与 CRC8 校验。
- AFE 唤醒、基础采样配置、出厂校准参数缓存。
- 单体电压、电流、外部 NTC 温度读取与物理量换算。
- 独立的主机仿真测试、上板冒烟测试入口和开发记录。

当前已通过编译与部分主机模拟验证，实际板上通信、读数及精度仍待验证。主函数尚未接入 AFE 测试入口；充放电 MOS 和均衡保持关闭。保护阈值、故障恢复、充放电策略及 SOC 等尚未完成。

## 目录

| 路径 | 内容 |
|---|---|
| `BMSCore/` | AFE 初始化、校准和采集接口 |
| `Hardware/` | 软件 I2C、BQ76920 寄存器及 CRC 驱动 |
| `User/` | 主函数、时钟、DWT、串口及中断 |
| `Driver/` | STM32 HAL、CMSIS 和启动文件 |
| `ThridParty/RT-Thread/` | RT-Thread 内核及移植层，沿用现有目录拼写 |
| `Tests/` | 主机测试和独立上板测试 |
| `tools/` | 辅助分析脚本 |
| [记录手册.md](记录手册.md) | 学习笔记、寄存器说明与开发记录 |

## 构建与验证

使用 Keil MDK，打开 `Proj.uvprojx`，选择工程配置的 ARM Compiler 5 构建。首次构建时如输出目录不存在，创建 `Objects/`、`Listings/`。

上板测试参考 [Tests/afe_board/README.md](Tests/afe_board/README.md)。入口 `AFE_Test_Run()` 内部调用初始化，输出串口日志并保存调试报告；接入主线程前建议将其栈调整为至少 2KB。软件 I2C 和 CRC 的主机测试见各自测试目录说明。

本工程当前用于学习和开发验证，尚未完成实际充放电与保护功能验证。

## 第三方代码

STM32 HAL、CMSIS、RT-Thread 等第三方代码保留各自的版权和许可声明；具体条件以对应源文件及附带许可文本为准。
