#ifndef ROBOTISDEF_H_
#define ROBOTISDEF_H_

//定义数据类型
typedef char                INT8_T;
typedef short int           INT16_T;
typedef int                 INT32_T;

typedef unsigned char       UINT8_T;
typedef unsigned short int  UINT16_T;
typedef unsigned int        UINT32_T;

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b) ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l) ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l) ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w) ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w) ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

//异常错误代码
#define PING_NO             (2)
#define FIRMWARE_ERROR      (3)
#define OVERHEAT_ERROR      (4)
#define OVERLOAD_ERROR      (5)
#define VOLTAGE_ERROR       (6)
#define ANGLELIMIT_ERROR    (7)
#define RANGE_ERROR         (8)
#define CHECK_ERROR         (9)
#define INST_ERROR          (10)
#define FILE_NO             (11)
#define RANGEOUT_ERROR      (12)
#define ARMINST_ERROR       (13)
#define TXFAIL_ERROR        (15)
#define TXERROR_ERROR       (16)
#define RXFAIL_ERROR        (17)
#define RXWAITING_ERROR     (18)
#define RXTIMEOUT_ERROR     (19)
#define RXCORRUPT_ERROR     (20)
#define SERVOPORT_ERROR     (22)
#define PCPORT_ERROR        (23)
#define PUMP_ERROR          (24)
#define LEDPORT_ERROR       (25)
#define LED_ERROR           (26)
#define OTHER_ERROR         (255)

//舵机错误代码
#define ERRBIT_VOLTAGE      (1)
#define ERRBIT_ANGLE        (2)
#define ERRBIT_OVERHEAT     (4)
#define ERRBIT_RANGE        (8)
#define ERRBIT_CHECKSUM     (16)
#define ERRBIT_OVERLOAD     (32)
#define ERRBIT_INSTRUCTION  (64)

#define COMM_TXSUCCESS      (0)
#define COMM_RXSUCCESS      (1)
#define COMM_TXFAIL         (2)
#define COMM_RXFAIL         (3)
#define COMM_TXERROR        (4)
#define COMM_RXWAITING      (5)
#define COMM_RXTIMEOUT      (6)
#define COMM_RXCORRUPT      (7)

//灯具内存地址
#define ADDR_MODE     (0x00) //模式选择地址
#define ADDR_CONTROL  (0x01) //LED控制权
#define ADDR_KEYNUM   (0x02) //每个键的数目
#define ADDR_TIMEOUT  (0x03) //演奏键值超时
#define ADDR_ZERO     (0x04) //写1清零
#define ADDR_GOALKEY1 (0x05) //目标键值1(1-8)
#define ADDR_GOALKEY2 (0x06) //目标键值2(9-16)
#define ADDR_GOALKEY3 (0x07) //目标键值3(17-24)
#define ADDR_RTKEYVALUE1  (0x40) //实时键值1(1-8)
#define ADDR_RTKEYVALUE2  (0x41) //实时键值2(9-16)
#define ADDR_RTKEYVALUE3  (0x42) //实时键值3(17-24)
#define ADDR_PCKEYVALUE1  (0x43) //记录的键值1(1-8)
#define ADDR_PCKEYVALUE2  (0x44) //记录的键值2(9-16)
#define ADDR_PCKEYVALUE3  (0x45) //记录的键值3(17-24)
#define ADDR_MOVEADDVALUE (0x46) //滑动增量
#define ADDR_LEFTENTER    (0x48) //左键确认值
#define ADDR_RIGHTENTER   (0x49) //右键确认值

//机械臂的种类
#define AXISID      (0x10)
#define DELTAID     (0X20)
#define SCARAID     (0x30)
#define DFIVEID     (0x40)

#endif
