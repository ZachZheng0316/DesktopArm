#ifndef XQTORH7HEADER__H__
#define XQTORH7HEADER__H__

#include "PacketHandler.h"

//舵机寄存器地址
//Flash Area
#define SERVOTYPE       (0)
#define FIVEARMID       (2)
#define ID              (3)
#define BAUD            (4)
#define ANGLEUP         (6)
#define ANGLEDOWN       (8)
#define TEMPRERATURE    (11)
#define VOLTAGE         (12)
#define CURRENT         (14)
#define ANGLEOFFSET     (20)
//RAM Area
#define CYCLENUM        (23)
#define ENABLE          (24)
#define LED             (25)
#define PID_P           (26)
#define PID_I           (27)
#define PID_D           (28)
#define PROTECTEDMODE   (29)
#define GOALPOSITION    (30)
#define MOVINGSPEED     (32)
#define PRESENDPOSITION (36)
#define PRESENTSPEED    (38)
#define PRESENTVOLTAGE  (42)
#define PRESENTTEMPERATURE (43)
#define PRESENTCURRENT  (68)
#define GOALACCE        (73)

namespace ROBOTIS{
class XQtorH7{
private:
    PacketHandler *pPacket;
    int ServoNum;

public:
    XQtorH7(char deviceName[], int baud, int _servoNum);
    bool SuccessOpenDevice(); //检测舵机初始化是否成功打开设备

    int set_one_servo_bytes(int byteNum, int id, int address, int value);
    int get_one_servo_bytes(int byteNum, int id, int address);
    int sync_set_many_servo(int byteNum, int id[], int addr, int value[]);
    int wait_for_one_servo_exten(int id, int exten);
    int wait_for_many_servo_exten(int exten[]);

    int returnServoAnomaly();
    int returnCommuAnomaly(int _commStatus);

    bool servoIsPingOK();
};
}

#endif
