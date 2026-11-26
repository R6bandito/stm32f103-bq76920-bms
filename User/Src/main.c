/* BMS application entry, called by the RT-Thread main thread.
 * Board clocks, debug UART, DWT, heap and RT tick are initialized beforehand.
 */
#include "rtthread.h"
#include "sys.h"
#include "bms_app.h"

int main( void ) {
    HAL_StatusTypeDef res;
    /* The AFE I/O thread can report an initialization failure immediately. */
    res = Bms_FaultThreadCreate();
    if (res != HAL_OK)
        return 1;

    res = Bms_AfeIoThreadCreate();
    if (res != HAL_OK)
        return 1;

    if (SystemConsoleMailboxInit() == HAL_OK) {
        /* Logging is optional. The start routine cleans up on failure. */
        (void)SystemConsoleThreadStart();
    }

    res = Bms_ConsoleThreadCreate();
    if (res != HAL_OK)
        LOG_ERROR("BmsConsoleThread Start Failed!");

    while(1) {
        rt_thread_mdelay(2000u);
    }
}
