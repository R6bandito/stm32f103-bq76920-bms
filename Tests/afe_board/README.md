# AFE上板冒烟测试

入口：`AFE_Test_Run(timeout_ms)`；报告：`AFE_Test_GetReport()`。文件已加入Keil的`Tests/AFE Board`组，尚未在main中调用，也未烧录。

## 明天接入

1. 在RT用户main或一个独占AFE的线程中调用，不能放在board初始化、ISR或调度器启动前。
2. 当前`RT_MAIN_THREAD_STACK_SIZE=1024`。若在main跑，先在`ThridParty/RT-Thread/port/rtconfig.h`将其调到2048，给格式化日志留空间；本次未修改此配置。
3. 核对电池采样接线与5V/3.3V，首轮不接充电器和外部负载，不启动其他AFE调用者。
4. 入口内部调用AFE_Init，外部不用再重复Init。它不会打开充放电MOS或均衡。

后续main中可以这样接入（本次只给示意，未改main）：

```c
#include "AFE_Test.h"

static volatile AFEState_t test_result;
static const volatile AFE_TestReport_t * volatile test_report;

/* 放在用户main/线程函数中： */
test_report = AFE_Test_GetReport();
test_result = AFE_Test_Run(20u);
```

运行约11秒加通信/日志时间，共5轮，随后返回。timeout_ms是底层单次I2C预算，不是整套测试总时长。

## 测试内容

| 用例 | 预期 |
|---|---|
| 各Get接口传NULL、非法枚举数值5和-1（合法数值0～4） | AFE_EPARAM，输出变量不被改写 |
| AFE_Init | AFE_EOK；失败立即结束并保留错误码 |
| 首轮等待2200ms | 给电压/电流及约2s更新的温度留出首轮等待 |
| 状态读取 | API成功；故障位单独记录并计失败；首轮检查CC_READY |
| 五节电压 | 每节读数状态成功；人工与万用表核对，对应VC1～VC5 |
| 电压求和 | 仅五节都成功时，sum才是完整总和，不是独立读取BAT寄存器 |
| 电流 | 单位mA；MOS关闭、无外接负载时重点看合理性，不硬性要求绝对0 |
| 温度 | 单位0.1℃，253表示25.3℃，与探头实际环境核对 |
| 连续5轮 | 保留每轮读数、返回码和收集起止tick |

各用例只使用公开AFE接口。除Init的基础配置外不写寄存器，不清故障、不打开MOS/均衡，不在实物上故意制造短路、过压等故障。

## 结果怎么看

日志经现有debug UART直接发送，不依赖当前尚未完成的fputc，也未修改通用日志框架。每个字段旁边的`result=0`表示对应API成功；非0时该数值无效。总和日志中的`valid=1`表示五节读数均成功。

调试器查看`test_report`：

- `phase`：参数测试、初始化、采样、结束。
- `init_result`、`final_result`：初始化与整套测试的执行结果。
- `passed_checks`、`failed_checks`：接口/状态检查计数。
- `samples_done`、`samples[0..4]`：完成轮数与每轮结果。
- `observed_faults`：测试期间见到的故障位汇总，不含CC_READY。
- `uart_failures`：局部日志发送失败次数；单独记录，不改变AFE测试返回码。

CC_READY是锁存状态。本测试不清它，不把后续轮次的1当成新转换通知；采集时间戳也不意味着电压、电流、温度同时采样。AFE_EOK不代替万用表/温度计验证、首轮接线检查或正式保护功能验证。

## 验证与删除

主机对测试入口模拟验证了：正常流程、初始化失败、采样CRC失败、故障状态、UART发送失败、参数检查未按预期返回及零超时参数。Keil构建日志为`Objects/afe_board_test_build.log`，0 Error、2条原有警告（cpuport.c未使用变量、sys.c日志函数缺少返回）；尚未上板。

用完后移除未来main中的调用和include，再删除Keil的`Tests/AFE Board`组及本目录即可。Keil和VS Code的`Tests/afe_board`头文件搜索路径也可一并移除；AFE业务接口和底层驱动没有引用测试文件。
