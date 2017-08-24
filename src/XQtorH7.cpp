#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "RobotisDef.h"
#include "Dmath.h"
#include "XQtorH7.h"
#include "PacketHandler.h"

using namespace std;
using namespace ROBOTIS;

#define UPPERLIMIT  (500)

float RcvWaitTime = 5.0f;

/****************************************
函数意义：
    初始化函数
参数意义：
    deviceName:设备名称
    baud:波特率
    _servoNum:舵机个数
*****************************************/
XQtorH7::XQtorH7(char deviceName[], int baud, int _servoNum)
{
    pPacket = new PacketHandler(deviceName, baud);
    ServoNum = _servoNum;
}

/***************************
函数意义：
    检测舵机初始化是否成功打开设备
参数意义：
    无参数
返回值：
****************************/
bool XQtorH7::SuccessOpenDevice()
{
    return pPacket->isSuccessInit();
}

/*********************************************
函数意义：
    控制单个或多个舵机单字节信息
参数意义：
    byteNum:byteNum==1:表示写入1个字节；
            byteNum==2:表示写入2个字节
    id:表示舵机id
    addr:表示要写入的地址
    value:表示要写入的数值
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码；
*********************************************/
int XQtorH7::set_one_servo_bytes(int byteNum, int id, int address, int value)
{
    int commStatus = 0, writeUpperLimit = UPPERLIMIT;
    int _servoAbnormal = 0, _commAbnormal = 0;

    while(writeUpperLimit >= 0) {
        if(1 == byteNum)
            pPacket->write_byte((UINT8_T)id, (UINT8_T)address, (UINT8_T)value);
        else if(2 == byteNum)
            pPacket->write_word((UINT8_T)id, (UINT8_T)address, (UINT16_T)value);
        else {
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }
        _servoAbnormal = 0; _commAbnormal = 0;
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus) {
            //检测舵机是否存在异常
            _servoAbnormal = returnServoAnomaly();
            if((_servoAbnormal==9)||(_servoAbnormal==10)) {
                delay_ms(10.0); //延迟
            }
            else {
                //pPacket->clear_port();
                break;
            }
        }
        else {
            _commAbnormal = returnCommuAnomaly(commStatus);
            printf("\r%s: %d: id(%d) _commAbnormal(%d) writeUpperLimit(%d) ", __FILE__, __LINE__, id, _commAbnormal, writeUpperLimit);
            if(RXTIMEOUT_ERROR == _commAbnormal) {
                pPacket->setBiasWaitTime(RcvWaitTime);
                RcvWaitTime += 0.5f;
            }
            delay_ms(30.0);
        }
        writeUpperLimit -= 1;
    }

    if((commStatus != COMM_RXSUCCESS) || (_servoAbnormal != 0))
        printf("\n");

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    if(writeUpperLimit >= 0) {
        return _servoAbnormal;
    }
    else {
        if(_servoAbnormal)
            return _servoAbnormal;
        if((18==_commAbnormal)||(19==_commAbnormal)||(20==_commAbnormal)) {
            if(!servoIsPingOK())
                return 2;
            return _commAbnormal;
        }
        else
            return _commAbnormal;
    }
}

/********************************************************
函数意义：
    获取单个舵机单字节信息
参数意义：
    byteNum:byteNum==1:表示获取1个字节信息
            byteNum==2:表示获取2个字节信息
    id：舵机ID
    addr:表示舵机的地址
返回值：
    当执行成功时，返回要获取的值
    当执行失败时，返回异常代码（异常代码用负数表示）
********************************************************/
int XQtorH7::get_one_servo_bytes(int byteNum, int id, int address)
{
    int value, commStatus = 0, readUpperLimit = UPPERLIMIT;
    int _commAbnormal = 0, _servoAbnormal = 0;

    while(readUpperLimit >= 0) {
        if(1 == byteNum) {
            value = abs((int)pPacket->read_byte(id, address));
            value %= 255;
        }
        else if(2 == byteNum) {
            value = abs((int)pPacket->read_word(id, address));
            value %= 4096;
        }
        else{
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }

        _commAbnormal = 0; _servoAbnormal = 0;
        commStatus = pPacket->get_result();
        if(commStatus == COMM_RXSUCCESS) {
            //检测舵机是否存在异常
            _servoAbnormal = returnServoAnomaly();
            if(!_servoAbnormal) {
                //舵机没有异常
                RcvWaitTime = 5.0f;
                pPacket->setBiasWaitTime(RcvWaitTime);
                //pPacket->clear_port();
                break;
            }
            else {
                //舵机存在异常
                if((_servoAbnormal == 9) || (_servoAbnormal==10)) {
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
        else{
            //与舵机通信失败
            _commAbnormal = returnCommuAnomaly(commStatus);
            //如果是因为超时造成通信错误，延长时间
            if(19==_commAbnormal) {
                pPacket->setBiasWaitTime(RcvWaitTime);
                RcvWaitTime += 0.5f;
            }
        }

        delay_ms(10.0); //延时
        if(0 == readUpperLimit%100)
            printf("%s: %d: _commStatus(%d) _servoAbnormal(%d) readUpperLimit(%d)\n", __FILE__, __LINE__, _commAbnormal, _servoAbnormal, readUpperLimit);
        readUpperLimit -= 1;
        //pPacket->clear_port();
    }

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    //当writeUpperLimit > 0时，表示写入成功或者由于舵机异常造成跳出循环
    if(readUpperLimit >= 0) {
        if(!_servoAbnormal) //如果不存在异常
            return value;
        else
            return -_servoAbnormal;
    }
    else{
        //如果writeUpperLimit <0，表示循环结束，有可能是舵机异常
        //或者是通信异常造成的异常
        if(_servoAbnormal)
            return -_servoAbnormal;

        if((18==_commAbnormal)||(19==_commAbnormal)||(20==_commAbnormal)) {
            if(!servoIsPingOK())
                return -2;
        }
        return -_commAbnormal;
    }
}

/*********************************************************
函数意义：
    以sync的方式控制多个舵机的寄存器
参数意义：
    byteNum:
        byteNum==1:表示控制舵机的单字节信息
        byteNum==2:表示控制舵机的双字节信息
    id:表示舵机组
    addr:表示要写入的地址
    value:表示要写入的值
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
*********************************************************/
int XQtorH7::sync_set_many_servo(int byteNum, int id[], int addr, int value[])
{
    int commStatus = 0, writeUpperLimit = UPPERLIMIT;
    int _servoAbnormal = 0, _commAbnormal = 0;

    while(writeUpperLimit >= 0) {
        if(1 == byteNum){
            UINT8_T _value[ServoNum];
            for(int i = 0; i < ServoNum; i++)
                _value[i] = (UINT8_T)value[i];
            pPacket->sync_write_byte(ServoNum, id, (UINT8_T)addr, _value);
        }
        else if(2 == byteNum) {
            UINT16_T _value[ServoNum];
            for(int i = 0; i < ServoNum; i++)
                _value[i] = (UINT16_T)value[i];
            pPacket->sync_write_word(ServoNum, id, (UINT8_T)addr, _value);
        }
        else {
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }
        _servoAbnormal = 0; _commAbnormal = 0;
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus) {
            //pPacket->clear_port();
            break;
        }
        else {
            _commAbnormal = returnCommuAnomaly(commStatus);
            printf("\r%s: %d: id(%d) _commAbnormal(%d) writeUpperLimit(%d) ", __FILE__, __LINE__, id, _commAbnormal, writeUpperLimit);
            if(RXTIMEOUT_ERROR == _commAbnormal) {
                pPacket->setBiasWaitTime(RcvWaitTime);
                RcvWaitTime += 0.5;
            }
        }
        writeUpperLimit -= 1;
    }

    if((commStatus != COMM_RXSUCCESS))
        printf("\n");

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    if(writeUpperLimit >= 0)
        return 0;
    else {
        return _commAbnormal;
    }
}

/******************************************************
函数意义：
    等待指定舵机运动停止
参数意义：
    id：舵机id
    exten：目标可能的容错范围
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码(负数表示)
******************************************************/
int XQtorH7::wait_for_one_servo_exten(int id, int exten)
{
    int goal_pos, value, diff;

    goal_pos = get_one_servo_bytes(2, id, GOALPOSITION);
    if(goal_pos < 0)
        return goal_pos;
    printf("%s: %d: id(%d)--Goal_Position(%d)\n", __FILE__, __LINE__, id, goal_pos);

    //获取目标位置
    do{
        value = get_one_servo_bytes(2, id, PRESENDPOSITION);
        if(value < 0)
            return value;
        diff = abs(goal_pos - value);
        delay_ms(20.0); //延迟20ms
    }while(diff>=exten);
    printf("%s: %d id(%d)--Present_Position(%d)--exten(%d)\n", __FILE__, __LINE__, id, value, exten);

    return 0;
}

/*************************************************
函数意义:
    检测所有舵机是是否运动到目标位置附近
    如果是，正常退出；
    如果不是，继续检测；
    如果出现异常，返回异常值。
参数：
    exten[]:距离目标位置的宽度矩阵，与舵机对应
返回值：
    当舵机运动到目标位置时，返回0；
    当舵机出现异常时，返回异常值(返回的异常值是负数)
*************************************************/
int XQtorH7::wait_for_many_servo_exten(int exten[])
{
    int i, abnormal = 0;

    for(i = 0; i < ServoNum; i++) {
        abnormal = wait_for_one_servo_exten(i+1, exten[i]);
        if(abnormal < 0)
            return abnormal;
    }
    return 0;
}

//返回舵机异常
int XQtorH7::returnServoAnomaly()
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

//返回通信异常
int XQtorH7::returnCommuAnomaly(int _commStatus)
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

//检查舵机是否都连接正常
bool XQtorH7::servoIsPingOK()
{
    int _servoIndex = 1, _gbCommStatus;
    int detectLimitUpNum = 20; //ping的上限次数
    bool result = true;

    while(_servoIndex <= ServoNum) {
        detectLimitUpNum = 20;
        while(detectLimitUpNum >= 0) {
            if(pPacket->ping(_servoIndex)) {
                break;
            }
            else{
                delay_ms(50.0f);
                detectLimitUpNum -= 1;
            }
        }
        if(detectLimitUpNum < 0) {
            printf("%s: %d: id(%d)---->false\n", __FILE__, __LINE__,_servoIndex);
            result = false;
        }
        else
            printf("%s: %d: id(%d)---->ok\n", __FILE__, __LINE__, _servoIndex);
        _servoIndex += 1;
    }

    return result;
}
