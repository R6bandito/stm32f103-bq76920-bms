# 软件 I2C 主机协议仿真

在 `MyBmsProj` 目录使用 MinGW g++：

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Wno-missing-field-initializers -I Tests/soft_i2c/stubs Tests/soft_i2c/test.cpp -o Objects/soft_i2c_host_test.exe
.\Objects\soft_i2c_host_test.exe
```

测试直接包含实际 `Hardware/i2c_driver.c`，用 C++ 寄存器对象捕获 BSRR 写入、模拟 IDR 电平与从机 ACK，不改生产代码。单独关闭的 `missing-field-initializers` 警告来自 C 语言合法的 `{0}` 写法被当作 C++ 编译；目标工程仍由 ARMCC 按 C 编译。

覆盖：ODR 预置、MSB 位序、每字节 8+1 拍、首次/重复 START、STOP 后新事务、ACK/NACK 出口电平、NACK 自动 STOP、SDA/SCL 卡低、时钟拉伸、整个 API 共享超时、STOP 失败释放、参数与超时溢出检查、DWT 不可用/中途停计数/32 位回绕。

仿真不能验证真实上拉电阻、上升沿、实际 SCL 频率、BQ76920 唤醒或板上 ACK。未接入 Keil 工程，不随 MCU 程序编译。

2026-11-17补充接收原语验证：从机发送MSB字节、连续读后主机ACK/末字节NACK、读阶段SCL拉伸、0x00/0xFF/非对称位型、参数拦截、失败不写输出，以及第九拍高电平阶段计时异常后的SCL拉低/SDA释放。

公开接口补充验证：未START时读/应答不打时钟；`I2c_Send_ACK_NACK`按用户约定非零发ACK、零发NACK；事务中的入口参数错误与读取超时由驱动内部中止并释放引脚。
