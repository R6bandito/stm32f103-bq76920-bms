/* BMS application entry, called by the RT-Thread main thread.
 * Board clocks, debug UART, DWT, heap and RT tick are initialized beforehand.
 */
int main( void ) {


    /* No application workers yet. Returning ends only the main thread;
     * the RT-Thread scheduler and idle thread remain active.
     */
    return 0;
}
