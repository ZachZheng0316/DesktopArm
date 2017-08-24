#include "LampsControl.h"
#include "RobotisDef.h"
#include <stdio.h>

using namespace std;
using namespace ROBOTIS;

float RcvWaitTime = 5.0f;

/****************************************
函数意义：
    LampsControl初始化函数
参数意义：
    deviceNum:设备序号
    baud:设备botelv
返回值：
    无
****************************************/
LampsControl::LampsControl(char deviceName[], int baud)
{
    pLampsPro = new LampsProtocol(deviceName, baud);
}

/***********************************
函数意义：
    检测端口设备是否可以打开
参数意义：
    无
返回值：
    如果执行成功，返回true；
    如果执行失败，返回false;
***********************************/
bool LampsControl::SuccessOpenDevice()
{
    return pLampsPro->isSuccessInit();
}

/*****************************************
函数意义：
    往灯具中写入数据
参数意义：
    _id:灯具id
    _addr:起始地址
    _value:数据
    _byteNum:要写入的数据个数
返回值：
    如果执行成功，返回0;
    如果执行失败，返回错误代码(正数)
*****************************************/
int LampsControl::set_Lamps_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T* _value, UINT8_T _byteNum)
{
    int commStatus, writeUpperLimit = 80;

    while(writeUpperLimit >= 0) {
        pLampsPro->write_bytes(_id, _addr, _value, _byteNum);
        commStatus = pLampsPro->get_result();
        if(commStatus == COMM_RXSUCCESS) {
            if(returnServoAnomaly()) {
                RcvWaitTime = 5.0f;
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                return LED_ERROR;
            }
            else {
                RcvWaitTime = 5.0f;
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                return 0;
            }
        }
        else{
            if(RXTIMEOUT_ERROR == commStatus) {
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                RcvWaitTime += 0.5;
            }
        }
        printf("%s: %d: commStatus(%d) writeUpperLimit(%d)\n", __FILE__, __LINE__, commStatus, writeUpperLimit);
        writeUpperLimit -= 1;
    }

    RcvWaitTime = 5.0f;
    pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
    return commStatus;
}

/*****************************************
函数意义：
    读取灯具的数据
参数意义：
    _id:灯具id
    _addr:起始地址
    _byteNum:寄存器个数
返回值：
    如果执行成功，返回0；
    如果执行失败，返回错误代码。
******************************************/
int LampsControl::get_Lamps_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T _byteNum)
{
    int commStatus, readUpperLimit = 80;

    while(readUpperLimit >= 0) {
        pLampsPro->read_bytes(_id, _addr, _byteNum);
        commStatus = pLampsPro->get_result();
        if(COMM_RXSUCCESS == commStatus) {
            if(returnServoAnomaly()) {
                RcvWaitTime = 5.0f;
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                return LED_ERROR;
            }
            else {
                RcvWaitTime = 5.0f;
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                return 0;
            }
        }
        else{
            if(COMM_RXTIMEOUT == commStatus) {
                pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
                RcvWaitTime += 0.5f;
            }
        }
        readUpperLimit -= 1;
    }

    RcvWaitTime = 5.0f;
    pLampsPro->setBiasRcvWaitTime(RcvWaitTime);
    return commStatus;
}

/******************************************
函数意义：
    读取具体的数据
参数意义：
    _index:读取数据的标签
返回值：
    返回_index寄存器的数值
******************************************/
UINT8_T LampsControl::read_Lamps_StatusPacket(int _index)
{
    return pLampsPro->get_rxpacket_parameter(_index);
}

/**************************************
函数意义：
    检测灯具异常
参数意义：
    无
返回值：
    如果灯具存在异常，返回true
    如果灯具不存在异常，返回false;
**************************************/
bool LampsControl::returnServoAnomaly()
{
    return pLampsPro->get_rxpacket_error(0xff);
}

/*************************************************
函数意义：
    检测通信异常
参数意义：
    _commStatus:通信异常代码
返回值：
    如果存在通信异常,返回异常代码(正数)
    如果不存在通信异常，返回0；
*************************************************/
int LampsControl::returnCommuAnomaly(int _commStatus)
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
