#ifndef HARDWAREHEADER__H__
#define HARDWAREHEADER__H__

#include "RobotisDef.h"

#define THE_DEVICE15 "/sys/class/gpio_sw/PH15/data"
#define THE_DEVICE16 "/sys/class/gpio_sw/PH16/data"

int Hardware_open_port();       //打开端口
int Hardware_open_beng();       //打开气泵
int Hardware_close_beng();      //关闭气泵
int Hardware_write_gpio(int num, UINT8_T value);
int Hardware_close_port();      //关闭端口

#endif
