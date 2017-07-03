#ifndef MX28ATHEADER__H__
#define MX28ATHEADER__H__

#include "PacketHandler.h"

//信息发送状态码
#define COMM_TXSUCCESS      (0)
#define COMM_RXSUCCESS      (1)
#define COMM_TXFAIL         (2)
#define COMM_RXFAIL         (3)
#define COMM_TXERROR        (4)
#define COMM_RXWAITING      (5)
#define COMM_RXTIMEOUT      (6)
#define COMM_RXCORRUPT      (7)

//舵机寄存器地址
#define ID                  (3)  //ID地址
#define Baud_Rate           (4)  //波特率地址
#define MAX_Torque          (14) //最大波特率
#define Alarm_Shutdown      (18) //出错标识
#define Torque_Enable       (24) //扭矩开关
#define D_Gain              (26) //D增益
#define I_Gain              (27) //I增益
#define P_Gain              (28) //P增益
#define Goal_Position       (30) //目标刻度
#define Moving_Speed        (32) //运动速度
#define Torque_Limit        (34) //最大扭矩
#define Present_Position    (36) //当前位置
#define Present_Speed       (38) //当前速度
#define Present_Load        (40) //当前负载
#define Present_Voltage     (42) //当前电压
#define Present_Temperature (43) //当前温度
#define Moving              (46) //判断Goal_Position是否完成
#define Punch               (48) //Punch
#define Goal_Acceleration   (73) //目标加速度

namespace ROBOTIS{
class MX28AT{
private:
    PacketHandler *pPacket;
    int ServoNum;

public:
    MX28AT(char deviceName[], int baud, int _servoNum);
    bool SuccessOpenDevice(); //检测舵机初始化是否成功打开设备

    int set_one_servo_bytes(int byteNum, int id, int addr, int value);
    int get_one_servo_bytes(int byteNum, int id, int addr);
    int sync_set_many_servo(int byteNum, int id[], int addr, int value[]);
    int reg_set_many_servo(int byteNum, int id[], int addr, int value[]);
    int wait_for_one_servo_exten(int id, int exten);
    int wait_for_many_servo_exten(int exten[]);

    int returnCommuAnomaly(int _commStatus);
    int returnServoAnomaly();

    bool servoIsPingOK();

    bool checkServoOk(int servoNum, int id[]);  //检测舵机是否都在
    void set_one_servo_byte(int id, int addr, int value);
    void set_one_servo_word(int id, int addr, int value);
    void set_many_servo_byte(int servoNum, int id[], int addr, int value[]);
    void set_many_servo_word(int servoNum, int id[], int addr, int value[]);
    int get_one_servo_byte(int id, int addr);
    int get_one_servo_word(int id, int addr);
    void wait_for_one_servo(int id);
    void wait_for_many_servo(int servoNum, int id[]);
    void wait_for_one_servo_exten(int id, int exten);
    void wait_for_many_servo_exten(int servoNum, int id[], int exten[]);
    void set_pen_height(int id, int address, int value);
    bool IsMoving(int id);
};

}

#endif
