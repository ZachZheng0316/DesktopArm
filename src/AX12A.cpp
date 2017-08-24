#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "RobotisDef.h"
#include "Dmath.h"
#include "AX12A.h"
#include "PacketHandler.h"

using namespace std;
using namespace ROBOTIS;

#define UPPERLIMIT  (1000)

float RcvWaitTime_AX12A = 5.0f;

/****************************************
函数意义：
    初始化函数
参数意义：
    deviceName:设备名称
    baud:波特率
    _servoNum:舵机个数
*****************************************/
AX12A::AX12A(char deviceName[], int baud, int _servoNum)
{
    pPacket = new PacketHandler(deviceName, baud);
    servoNum = _servoNum;
}

/***************************
函数意义：
    检测舵机初始化是否成功打开设备
参数意义：
    无参数
返回值：
****************************/
bool AX12A::SuccessOpenDevice()
{
    return pPacket->isSuccessInit();
}

/***********************************************************
函数意义：
    控制单个舵机单字节信息
参数意义：
    byteNum:1:表示写入1个字节；2：表示写入2个字节
    id:舵机id
    address:寄存器地址
    value:要写入的值
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
**********************************************************/
int AX12A::set_one_servo_bytes(int byteNum, int id, int address, int value)
{
    int commStatus = 0, writeUpperLimit = UPPERLIMIT;
    int _servoAbnormal = 0, _commAbnormal = 0;

    while(writeUpperLimit >= 0) {
        if(1 == byteNum)
            pPacket->write_byte(id, address, value);
        else if(2 == byteNum)
            pPacket->write_word(id, address, value);
        else {
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus) {
            //检测舵机是否存在异常
            _servoAbnormal = returnServoAnomaly();
            if((_servoAbnormal == 9) || (_servoAbnormal == 10)){
                delay_ms(10.0); //延迟
                pPacket->clear_port();
            }
            else {
                pPacket->clear_port();
                break;
            }
        }
        else{
            _commAbnormal = returnCommuAnomaly(commStatus);
            printf("\r%s: %d: id(%d) _commAbnormal(%d) writeUpperLimit(%d) ", __FILE__, __LINE__, id, _commAbnormal, writeUpperLimit);
            if(RXTIMEOUT_ERROR == _commAbnormal) {
                pPacket->setBiasWaitTime(RcvWaitTime_AX12A);
                RcvWaitTime_AX12A += 0.5;
            }
        }
        writeUpperLimit -= 1;
    }

    if((commStatus != COMM_RXSUCCESS) || (_servoAbnormal != 0))
       printf("\n");

    RcvWaitTime_AX12A = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime_AX12A);

    if(writeUpperLimit >= 0) {
        return _servoAbnormal;
    }
    else{
        if(_servoAbnormal)
            return _servoAbnormal;
        if((18 == _commAbnormal) || (19 == _commAbnormal) || (20 == _commAbnormal)){
            if(!servoIsPingOK())
                return 2;
            return _commAbnormal;
        }
        else
            return _commAbnormal;
    }
}

/******************************************
函数意义：
    获取单个舵机单字节信息
参数意义：
    byteNum:字节个数
    id:舵机ID
    address:地址
返回值：
    当执行成功时，返回要获取的值；
    当执行失败时，返回异常代码(异常代码用负数表示)
*******************************************/
int AX12A::get_one_servo_bytes(int byteNum, int id, int address)
{
    int value, commStatus = 0, readUpperLimit = UPPERLIMIT;
    int _commAbnormal = 0, _servoAbnormal = 0;

    while(readUpperLimit >= 0) {
        if(1 == byteNum) {
            value = abs(pPacket->read_byte((UINT8_T)id, (UINT8_T)address));
            value %= 256;
        }
        else if(2 == byteNum) {
            value = abs(pPacket->read_word((UINT8_T)id, (UINT8_T)address));
            value %= 4096;
        }
        else{
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }

        commStatus = pPacket->get_result();
        if(commStatus == COMM_RXSUCCESS) {
            //检测舵机是否有异常
            _servoAbnormal = returnServoAnomaly();
            if(!_servoAbnormal) { //舵机没有异常
                RcvWaitTime_AX12A = 5.0f;
                pPacket->setBiasWaitTime(RcvWaitTime_AX12A);
                pPacket->clear_port();
                break;
            }
            else{ //舵机存在异常
                if((_servoAbnormal ==9) || (_servoAbnormal == 10)) { //存在可能修复的异常
                    delay_ms(10.0);
                    pPacket->clear_port();
                }
                else {
                    //存在无法修复的异常
                    //清空管道，准备重复发送
                    break;
                }
            }
        }
        else {//与舵机通信失败
            delay_ms(10.0);
            _commAbnormal = returnCommuAnomaly(commStatus);
            //如果是因为超时造成通信错误，延长时间
            if(19 == _commAbnormal) {
                pPacket->setBiasWaitTime(RcvWaitTime_AX12A);
                RcvWaitTime_AX12A += 0.5;
            }
        }

        if(0 == readUpperLimit%100)
            printf("%s: %d: _commStatus(%d) _servoAbnormal(%d) readUpperLimit(%d)\n", __FILE__, __LINE__, _commAbnormal, _servoAbnormal, readUpperLimit);
        readUpperLimit -= 1;
    }

    RcvWaitTime_AX12A = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime_AX12A);

    //当writeUpperLimit > 0时，表示写入成功或者由于舵机异常造成跳出循环
    if(readUpperLimit >= 0) {
        if(!_servoAbnormal) //如果不存在异常
            return value;
        else
            return -_servoAbnormal;
    }
    else{ //如果writeUpperLimit < 0，表示循环结束，有可能是舵机异常或者通信异常
        if(_servoAbnormal)
            return -_servoAbnormal;

        if((18 == _commAbnormal) || (19 == _commAbnormal) || (20 == _commAbnormal)) {
            if(!servoIsPingOK()) //判断舵机是否在线
                return -2;
        }
        return -_commAbnormal;
    }
}

/****************************************************
函数意义：
    等待所有舵机停止运动
参数意义：
    id:舵机id
    exten:目标刻度的容错范围
返回值：
    如果执行成功，返回0;
    如果执行失败，返回错误代码(负数表示)。
*****************************************************/
int AX12A::wait_for_one_servo_exten(int id, int exten)
{
    int goal_pos, value, diff;

    goal_pos = get_one_servo_bytes(2, id, Goal_Position);
    if(goal_pos < 0)
        return goal_pos;
    printf("%s: %d: id(%d)--Goal_Position(%d)\n", __FILE__, __LINE__, id, goal_pos);

    //获取目标位置
    do{
        value = get_one_servo_bytes(2, id, Present_Position);
        if(value < 0)
            return value;
        diff = abs(goal_pos - value);
        printf("%s: %d: id(%d)--Goal_Position(%d)--Present_Position(%d)--diff(%d)--exten(%d)\n", __FILE__, __LINE__, id, goal_pos, value, diff, exten);
    }while(diff >= exten);
    //printf("%s: %d: id(%d)--Present_Position(%d)--exten(%d)\n", __FILE__, __LINE__, id, value, exten);
}

/***********************************************
函数意义：
    检测所有舵机是否运动到目标位置附近。
    如果是，正常退出；
    如果不是，继续检测；
    如果出现异常，返回异常值。
参数：
    exten[]:距离目标位置的宽度矩阵，与舵机相对应
返回值：
    当舵机运动到目标位置附近时，返回0；
    当舵机出现异常时，返回异常值(返回的异常值是负数)。
***********************************************/
int AX12A::wait_for_many_servo_exten(int exten[])
{
    int i, abnormal = 0;

    for(i = 0; i < servoNum; i++) {
        abnormal = wait_for_one_servo_exten(i+1, exten[i]);
        if(abnormal < 0)
        return abnormal;
    }
    return 0;
}

int AX12A::returnServoAnomaly()
{
    if(pPacket->get_rxpacket_error(ERRBIT_VOLTAGE))
        return 6;
    else if(pPacket->get_rxpacket_error(ERRBIT_ANGLE))
        return 7;
    else if(pPacket->get_rxpacket_error(ERRBIT_OVERHEAT))
        return 4;
    else if(pPacket->get_rxpacket_error(ERRBIT_RANGE))
        return 8;
    else if(pPacket->get_rxpacket_error(ERRBIT_CHECKSUM))
        return 9;
    else if(pPacket->get_rxpacket_error(ERRBIT_OVERLOAD))
        return 5;
    else if(pPacket->get_rxpacket_error(ERRBIT_INSTRUCTION))
        return 10;
    else //不存在异常返回0
        return 0;
}

int AX12A::returnCommuAnomaly(int _commStatus)
{
    if(COMM_TXFAIL == _commStatus)
        return 15;
    else if(COMM_TXERROR == _commStatus)
        return 16;
    else if(COMM_RXFAIL == _commStatus)
        return 17;
    else if(COMM_RXWAITING == _commStatus)
        return 18;
    else if(COMM_RXTIMEOUT == _commStatus)
        return 19;
    else if(COMM_RXCORRUPT == _commStatus)
        return 20;
    else if(COMM_RXSUCCESS == _commStatus)
        return 0;
    else if(COMM_TXSUCCESS == _commStatus)
        return 0;
    else
        return 255; //未知错误
}

bool AX12A::servoIsPingOK()
{
    int _servoIndex = 1, _gbCommStatus;
    int detectLimitUpNum = 20; //ping的上限次数

    while(_servoIndex <= servoNum) {
        detectLimitUpNum = 20;
        while(detectLimitUpNum >= 0) {
            if(pPacket->ping(_servoIndex))
                break;
            else{
                delay_ms(50.0);
                detectLimitUpNum -= 1;
            }
        }
        if(detectLimitUpNum < 0)
            return false;
        _servoIndex += 1;
    }
    return true;
}
