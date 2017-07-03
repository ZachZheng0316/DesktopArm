#ifndef HARDWAREHEADER__H__
#define HARDWAREHEADER__H__

#define THE_DEVICE15 "/sys/class/gpio_sw/PH15/data"
#define THE_DEVICE16 "/sys/class/gpio_sw/PH16/data"

int open_port();    //打开端口
int open_beng();    //打开气泵
int close_beng();   //关闭气泵
int close_port();   //关闭端口

#endif
