# BQ76920 CRC8主机验证

在`MyBmsProj`目录执行：

```powershell
gcc -std=c99 -Wall -Wextra -I Tests/bq76920_crc8/stubs Tests/bq76920_crc8/read_regs_test.c -o Objects/bq76920_crc8_test.exe
.\Objects\bq76920_crc8_test.exe
```

直接包含实际`.c`测试其static函数，不扩大生产头文件接口。使用独立的逐位模2长除法作参考，校验空输入、标准`123456789 -> 0xF4`、教学例子`0x80 -> 0x89`、全部单字节、首个读数据CRC输入，以及写地址0x10下全部寄存器/数据字节组合。

2026-11-17加入ReadRegs/ReadReg后，以`read_regs_test.c`一并运行原数学检查和公开I2C接口模拟：验证寄存器指针、重复START、首个CRC包含读地址、后续CRC只含数据、中间ACK/最后NACK、CRC错误NACK+STOP、逐步骤I2C错误映射及参数拦截。`test.c`保留原数学测试，由新测试包含调用。

WriteReg按用户要求保持空函数体，因此主机编译暂时保留其缺少返回值/参数未使用警告，不为测试补假实现。测试不验证GPIO电气波形或实际AFE通信。
