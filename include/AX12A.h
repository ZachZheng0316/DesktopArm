#ifndef AX12AHEADER__H__
#define AX12AHEADER__H__

#include "PacketHandler.h"

//舵机寄存器地址
#define ID                  (3)     //ID地址
#define Baud_Rate           (4)     //波特率地址
#define MAX_Torque          (14)    //最大波特率
#define Alarm_Shutdown      (18)    //出错标识
#define Torque_Enable       (24)    //扭矩开关
#define CW_Slope            (28)
#define CCW_Slope           (29)
#define Goal_Position       (30)    //目标刻度
#define Moving_Speed        (32)    //运动速度
#define Torque_Limit        (34)    //最大扭矩
#define Present_Position    (36)    //当前位置
#define Present_Speed       (38)    //当前速度
#define Present_Load        (40)    //当前负载
#define Present_Voltage     (42)    //当前电压
#define Present_Temperature (43)    //当前温度
#define Moving              (46)    //判断Goal_Position是否完成
#define Punch               (48)    //Punch

namespace ROBOTIS{
class AX12A{
private:
    PacketHandler *pPacket;
    int servoNum;

public:
    AX12A(char deviceName[], int baud, int _servoNum);
    bool SuccessOpenDevice(); //检测舵机初始化是否成功打开设备

    int set_one_servo_bytes(int byteNum, int id, int address, int value);
    int get_one_servo_bytes(int byteNum, int id, int address);
    int wait_for_one_servo_exten(int id, int exten);
    int wait_for_many_servo_exten(int exten[]);

    int returnServoAnomaly();
    int returnCommuAnomaly(int _commStatus);

    bool servoIsPingOK();
};

}

#endif
