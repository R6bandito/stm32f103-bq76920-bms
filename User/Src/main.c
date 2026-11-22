/* BMS application entry, called by the RT-Thread main thread.
 * Board clocks, debug UART, DWT, heap and RT tick are initialized beforehand.
 */
#include "rtthread.h"
#include "AFE_Test.h"
#include "sys.h"

int main( void ) {
    if (SystemConsoleMailboxInit() != HAL_OK)
        return 1;

    /* No application workers yet. Returning ends only the main thread;
     * the RT-Thread scheduler and idle thread remain active.
     */
    AFE_Test_Run(20);
    while(1) {
        rt_thread_mdelay(1000);
    }
}
