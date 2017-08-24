#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include "Dmath.h"
#include "MX28AT.h"
#include "PacketHandler.h"

using namespace std;
using namespace ROBOTIS;

#define UPPERLIMIT  (500) //指令重发上限

float RcvWaitTime = 5.0f; //设置延时偏置量

MX28AT::MX28AT(char deviceName[], int baud, int _servoNum)
{
    pPacket = new PacketHandler(deviceName, baud);
    ServoNum = _servoNum;
}

//检测是否成功，打开设备
bool MX28AT::SuccessOpenDevice()
{
    return pPacket->isSuccessInit();
}

/****************************************
函数意义：
    控制单个或者多个舵机单字节信息
参数意义：
    betyNum:byteNum=1:表示写入1个字节
            byteNum=2:表示写入2个字节
    id:表示舵机id
    addr:表示要写入的地址
    value:表示要写入的数值
返回值：
    如果执行成功，返回0。
    如果执行失败，返回错误代码。
****************************************/
int MX28AT::set_one_servo_bytes(int byteNum, int id, int addr, int value)
{
    int commStatus = 0, writeUpperLimit = UPPERLIMIT;
    int _servoAbnormal = 0, _commAbnormal = 0;

    while(writeUpperLimit >= 0) {
        if(1 == byteNum)
            pPacket->write_byte((UINT8_T)id, (UINT8_T)addr, (UINT8_T)value);
        else if(2 == byteNum)
            pPacket->write_word((UINT8_T)id, (UINT8_T)addr, (UINT16_T)value);
        else {
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }
        _servoAbnormal = 0; _commAbnormal = 0;
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus) {
            //检测舵机是否存在异常
            _servoAbnormal = reutrnServoAnomaly();
            if((_servoAbnormal==9)||(_servoAbnormal==10))
                delay_ms(10.0); //延时10ms
            else
                break;
        }
        else {
            _commAbnormal = returnCommuAnomaly(commStatus);
            printf("\r%s: %d: id(%d) _commAbnormal(%d) writeUpperLimit(%d) ", __FILE__, __LINE__, id, _commAbnormal, writeUpperLimit);
            if(RXTIMEOUT_ERROR == _commAbnormal) {
                RcvWaitTime += 0.5f;
                pPacket->setBiasWaitTime(RcvWaitTime);
            }
            delay_ms(30.0);
        }
        writeUpperLimit -= 1;
    }

    if((commStatus != COMM_RXSUCCESS)||(_servoAbnormal != 0))
        printf("\n");

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    if(writeUpperLimit >= 0)
        return 0;
    else {
        if(_servoAbnormal)
            return _servoAbnormal;
        if((RXTIMEOUT_ERROR == _commAbnormal) || (RXCORRUPT_ERROR)) {
            if(!servoIsPingOK())
                return PING_NO;
            return _commAbnormal;
        }
        else
            return _commAbnormal;
    }
}

/**********************************************
函数意义：
    获取单个舵机单字节信息
参数意义：
    byteNum:byteNum==1:表示获取1个字节信息
            byteNum==2:表示获取2个字节信息
    id:舵机ID
    addr:表示舵机的地址
返回值：
    当执行成功时，返回要获取的值
    当执行失败时，返回异常代码(异常代码用负数表示)
***********************************************/
int MX28AT::get_one_servo_bytes(int byteNum, int id, int addr)
{
    int value, commStatus = 0, readUpperLimit = UPPERLIMIT;
    int _commAbnormal = 0, _servoAbnormal = 0;

    while(readUpperLimit >= 0) {
        if(1 == byteNum) {
            value = abs((int)pPacket->read_byte(id, addr));
            value %= 255;
        }
        else if(2 == byteNum) {
            value = abs((int)pPacket->read_word(id, addr));
            value %= 4096;
        }
        else{
            printf("%s: %d: byteNum is error\n", __FILE__, __LINE__);
            return OTHER_ERROR;
        }
        _commAbnormal = 0; _servoAbnormal = 0;
        commStatus = pPacket->get_result();
        if(commStatus == COMM_RXSUCCESS) {
            //检测舵机是否异常
            _servoAbnormal = reutrnServoAnomaly();
            if(!_servoAbnormal)
                break;
            else {
                //舵机存在异常
                if((CHECK_ERROR == _servoAbnormal)||(INST_ERROR == _servoAbnormal))
                    delay_ms(10.0);//延长10ms
                else //存在无法修复的问题
                    break;
            }
        }
        else {
            //与舵机通信失败
            _commAbnormal = returnCommuAnomaly(commStatus);
            //如果是因为超时造成的，延长时间
            if(RXTIMEOUT_ERROR == _commAbnormal) {
                RcvWaitTime += 0.5;
                pPacket->setBiasWaitTime(RcvWaitTime);
            }
        }

        delay_ms(10.0); //延时10ms
        if(0 == readUpperLimit%100) {
            printf("%s: %d: _commStatus(%d) _servoAbnormal(%d) readUpperLimit(%d)\n", __FILE__, __LINE__, _commAbnormal, _servoAbnormal, readUpperLimit);
        }
        readUpperLimit -= 1;
    }

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    if(readUpperLimit >= 0) {
        if(!_servoAbnormal)
            return value;
        else
            return -_servoAbnormal;
    }
    else {
        if(_servoAbnormal)
            return -_servoAbnormal;

        if((RXTIMEOUT_ERROR == _commAbnormal)||(RXCORRUPT_ERROR == _commAbnormal)) {
            if(!servoIsPingOK())
                return -PING_NO;
        }
        return -_commAbnormal;
    }
}

/*********************************************
函数意义：
    以sync的方式控制多个舵机的寄存器
参数意义：
    byteNum:byteNum=1:表示控制舵机的单字节信息
            byteNum=2:表示控制舵机的双字节信息
    id[]:表示舵机组
    addr:表示要写入的地址
    value:表示要写入的值
返回值:
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
*********************************************/
int MX28AT::sync_set_many_servo(int byteNum, int id[], int addr, int value[])
{
    int commStatus = 0, writeUpperLimit = UPPERLIMIT;
    int _servoAbnormal = 0, _commAbnormal = 0;

    while(writeUpperLimit >= 0) {
        if(1 == byteNum) {
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
        if(COMM_RXSUCCESS == commStatus)
            break;
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

    if(commStatus != COMM_RXSUCCESS)
        printf("\n");

    RcvWaitTime = 5.0f;
    pPacket->setBiasWaitTime(RcvWaitTime);

    if(writeUpperLimit >= 0)
        return 0;
    else{
        return _commAbnormal;
    }
}

/***********************************************
函数意义：
    等待指定舵机运动停止
参数意义：
    id:舵机id
    exten:目标可能的容错范围
返回值：
    如果执行成功，返回0;
    如果执行失败，返回错误代码(负数表示)
***********************************************/
int MX28AT::wait_for_one_servo_exten(int id, int exten)
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
        delay_ms(20.0); //延迟20ms
    }while(diff>=exten)
    printf("%s: %d: id(%d)--Present_Position(%d)--exten(%d)\n", __FILE__, __LINE__, id, value, exten);

    return 0;
}

/********************************************************
函数意义：
    检测所有舵机是否运动到目标位置附近
    如果是，正常退出
    如果不是，继续检测
    如果出现异常，返回异常值
参数意义：
    exten[]:距离目标位置的宽度矩阵，与舵机对应
返回值：
    当舵机运动到目标位置时，返回0；
    当舵机出现异常时，返回异常值(返回的异常值是负数)
********************************************************/
int MX28AT::wait_for_many_servo_exten(int exten[])
{
    int i, abnormal = 0;

    for(i = 0; i < ServoNum; i++) {
        abnormal = wait_for_one_servo_exten(i+1, exten[i]);
        if(abnormal < 0)
            return abnormal;
    }
    return 0;
}


/*检测舵机是否都在
当舵机都存在的时候，返回true;
当舵机并不完全都存在的时候，返回false;
*/
bool MX28AT::checkServoOk(int servoNum, int id[])
{
    bool ret = true;

    for(int i = 0; i < servoNum; i++) {
        if(pPacket->ping(id[i]))
            printf("%s: %d: the servo is here ... %d\n", __FILE__, __LINE__, id[i]);
        else
            ret = false;
    }

    return ret;
}

//函数意义：
//  向指定舵机写入相应的值。如果写入失败，那么函数会一直重复写入
//的过程，直到写入成功。当失败次数达到1000次时，会显示失败数据。
//函数参数：
//  id：舵机ID
//  addr：舵机寄存器地址
//  value：要写入的值
//返回值：
//  无返回值。直到写入成功，则函数返回。
void MX28AT::set_one_servo_byte(int id, int addr, int value)
{
    int writeFailedNum = 0;
    UINT8_T commStatus;

    while(true){
        if(pPacket->write_byte((UINT8_T)id, (UINT8_T)addr, (UINT8_T)value))
            break;
        else{
            writeFailedNum ++;
            if(writeFailedNum >= 1000) {
                commStatus = pPacket->get_result();
                pPacket->PrintCommStatus(commStatus);
            }
            delay_ms(10.0);
        }
        printf("%s: %d: write byte failed is %d\n", __FILE__, __LINE__, writeFailedNum);
    }
}

//函数意义：
//  向指定舵机写入相应的值。如果写入失败，那么函数会一直重复写入
//的过程，直到写入成功。当失败次数达到1000次时，会显示失败数据。
//函数参数：
//  id：舵机ID
//  addr：舵机寄存器地址
//  value：要写入的值
//返回值：
//  无返回值。直到写入成功，则函数返回。
void MX28AT::set_one_servo_word(int id, int addr, int value)
{
    int writeFailedNum = 0;
    UINT8_T commStatus;

    while(true){
        if(pPacket->write_word((UINT8_T)id, (UINT8_T)addr, (UINT16_T)value))
            break;
        else{
            writeFailedNum ++;
            if(writeFailedNum >= 1000) {
                commStatus = pPacket->get_result();
                pPacket->PrintCommStatus(commStatus);
            }
            delay_ms(10.0);
        }
        printf("%s: %d: write word failed is %d\n", __FILE__, __LINE__, writeFailedNum);
    }
}


//函数意义：
//  指定舵机的个数及编号，同时设置多个舵机的属性
//函数参数：
//  servoNum：舵机的个数
//  id[]：舵机的ID编号
//  addr：属性地址
//  value：地址值
//返回值：
//  无返回值
void MX28AT::set_many_servo_byte(int _servoNum, int id[], int addr, int value[])
{
    int writeFailedNum = 0;
    UINT8_T commStatus, _value[_servoNum];

    for(int i = 0; i < _servoNum; i++)
        _value[i] = (UINT8_T)value[i];

    while(true){
        if(pPacket->sync_write_byte(_servoNum, id, (UINT8_T)addr, _value))
            break;
        else{
            writeFailedNum ++;
            if(writeFailedNum >= 1000) {
                commStatus = pPacket->get_result();
                pPacket->PrintCommStatus(commStatus);
            }
            delay_ms(10.0);
        }
        printf("%s: %d: sync write byte failed is %d\n", __FILE__, __LINE__, writeFailedNum);
    }
}


//函数意义：
//  向指定舵机写入相应的值。如果写入失败，那么函数会一直重复写入
//的过程，直到写入成功。当失败次数达到1000次时，会显示失败数据。
//函数参数：
//  addr：舵机寄存器地址
//  value[]：要写入的值
//返回值：
//  无返回值。直到写入成功，则函数返回。
void MX28AT::set_many_servo_word(int _servoNum, int id[], int addr, int value[])
{
    int writeFailedNum = 0;
    UINT8_T commStatus;
    UINT16_T _value[_servoNum];

    for(int i = 0; i < ServoNum; i++)
        _value[i] = (UINT16_T)value[i];

    while(true){
        if(pPacket->sync_write_word(_servoNum, id, (UINT8_T)addr, _value))
            break;
        else{
            writeFailedNum ++;
            if(writeFailedNum >= 1000) {
                commStatus = pPacket->get_result();
                pPacket->PrintCommStatus(commStatus);
            }
            delay_ms(5.0);
        }
        printf("%s: %d: sync write word failed is %d\n", __FILE__, __LINE__,  writeFailedNum);
    }
}

//函数意义：
//  向指定舵机读取相应的值。如果读取失败，那么函数会一直重复读取
//的过程，直到读取成功。当失败次数达到1000次时，会显示失败数据。
//函数参数：
//  id  : 舵机ID
//  addr：舵机寄存器地址
//返回值：
//  返回正确的读取值
int MX28AT::get_one_servo_byte(int id, int addr)
{
    int writeFailedNum = 0, result;
    UINT8_T commStatus;

    while(true){
        result = (int)pPacket->read_byte((UINT8_T)id, (UINT8_T)addr);
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus)
            return result;
        else {
            if(writeFailedNum ++ >= 1000)
                pPacket->PrintCommStatus(commStatus);
            delay_ms(10.0);
        }
        printf("%s: %d:read byte failed num is %d\n", __FILE__, __LINE__,  writeFailedNum);
    }
}

int MX28AT::get_one_servo_word(int id, int addr)
{
    int writeFailedNum = 0, result;
    UINT8_T commStatus;

    while(true){
        result = (int)pPacket->read_word((UINT8_T)id, (UINT8_T)addr);
        commStatus = pPacket->get_result();
        if(COMM_RXSUCCESS == commStatus)
            return result;
        else {
            if(writeFailedNum ++ >= 1000)
                pPacket->PrintCommStatus(commStatus);
            delay_ms(10.0);
        }
        printf("%s: %d: read word failed num is %d\n", __FILE__, __LINE__,  writeFailedNum);
    }
}

//等待指定舵机停止，函数才执行完毕
//如果连续10次，moving都为0则表示，运动停止
void MX28AT::wait_for_one_servo(int id)
{
    int moving, value, read_num = 0;

    do{
        moving = get_one_servo_byte(id, Moving);
        if(0 == moving)
            read_num ++;
        else
            read_num = 0;
    }while(read_num <= 10);
}

//当所有舵机的moving == 0，则表示运动停止
void MX28AT::wait_for_many_servo(int _servoNum, int id[])
{
    int moving;
    bool IsMoving = false;

    do{
        IsMoving = false;
        for(int i = 0; i < _servoNum; i++) {
            moving = get_one_servo_byte(id[i], Moving);
            if(moving == 1)
                IsMoving = true;
        }
    }while(IsMoving);
}

//等舵机运动到目标位置的某一范围
void MX28AT::wait_for_one_servo_exten(int id, int exten)
{
    int goal_pos, value, diff;

    goal_pos = get_one_servo_word(id, Goal_Position);

    do{
        value = get_one_servo_word(id, Present_Position);
        diff = abs(goal_pos - value);
    }while(diff >= exten);
}

//等待多个舵机运动到目标位置的某一范围
void MX28AT::wait_for_many_servo_exten(int _servoNum, int id[], int exten[])
{
    int goal_pos[_servoNum], value, diff;
    bool IsMoving = false;

    for(int i = 0; i < ServoNum; i++)
        goal_pos[i] = get_one_servo_word(id[i], Goal_Position);

    do{
        IsMoving = false;

        for(int i = 0; i < _servoNum; i++) {
            value = get_one_servo_word(id[i], Present_Position);
            diff = abs(goal_pos[i] - value);
            if(diff > exten[i])
                IsMoving = true;
        }
    }while(IsMoving);
}

/*设置特殊舵机的高度：例如scara的高度舵机*/
void MX28AT::set_pen_height(int id, int address, int value)
{
    pPacket->set_pen_height(id, address, value);
}

//检测指定舵机是否在运动
bool MX28AT::IsMoving(int id)
{
    int ret;

    ret = get_one_servo_byte(id, Moving);

    if(ret == 0)
        return false;
    else
        return true;
}
