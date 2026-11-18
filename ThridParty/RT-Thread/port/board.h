#ifndef BMS_RT_BOARD_H
#define BMS_RT_BOARD_H

/* Board adapter, not an RT-Thread public kernel API. Called once per tick ISR. */
void rt_os_tick_callback(void);

#endif /* BMS_RT_BOARD_H */
