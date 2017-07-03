#include <iostream>
#include <string.h>
#include <stdio.h>
#include "PortHandler.h"
#include "RobotisDef.h"
#include "ProtocolV3.h"

using namespace std;
using namespace ROBOTIS;

ProtocolV3::ProtocolV3(char deviceName[], int baud)
{
    pPort = new PortHandler(deviceName, baud); //申请串口设备
    memset(inst, 0, INST_LENGTH);
    memset(statue, 0, INST_LENGTH);
}

ProtocolV3::~ProtocolV3()
{
    closePort();
    delete pPort;
}

/*******************************
函数意义：
    初始化一体机的端口
参数意义：
    无参数
返回值：
    如果执行成功，返回true;
    如果执行失败，返回false;
*******************************/
bool ProtocolV3::isSuccessInit()
{
    return pPort->OpenPort();
}

/*
接受指令信息和反馈信息
flag == 0:表示接收的是指令消息
flag == 1:表示接收的是反馈信息
当接收成功时，返回1
当接收失败时，返回0
*/
bool ProtocolV3::receiveMessage(int flag)
{
    UINT8_T _checksum = 0, _idx = 0, _s = 0;
    int retNum = 0;
    int _real_length = 0;
    int _data_length = 6;
    int result = true;

    //清空数据缓冲区
    memset(inst, 0, INST_LENGTH);

    while (true) {
        //接收数据
        retNum = pPort->ReadPort((UINT8_T *)&inst[_real_length], _data_length - _real_length);
        _real_length += retNum;

        /*显示接收到数据*/
        if(_real_length > 0) {
            //显示接收到的消息
            for(int _index = _real_length - retNum; _index < _real_length; _index++)
                printf("%x ", inst[_index]);
            fflush(stdout);
        }

        /*如果实际数据长度 >= 给定的数据长度： 接收正确*/
        if(_real_length >= _data_length) {
            //找到数据头
            for(_idx = 0; _idx < (_real_length - 1); _idx++) {
                if((inst[_idx] == 0xff) && (inst[_idx+1] == 0xff))
                    break;
            }

            /*如果数据头在开始位置*/
            if(0 == _idx) {
                //如果是指令包：指令包的长度在[3]处
                if(0 == flag)
                    _data_length = (int)inst[3];
                else    //如果是反馈包：反馈包的长度在[2]处
                    _data_length = (int)inst[2];

                //如果实际长度 < 数据包的长度，则继续接收数据
                if(_real_length < _data_length)
                    continue;

                //如果数据长度 <6或者>128,都表示数据错误
                if((_data_length < 6) || (_data_length > 128)) {
                    result = false;
                    break;
                }

                /*检查cheSum是否正确*/
                for(_idx = 2; _idx < _data_length - 1; _idx++)
                    _checksum += inst[_idx];
                _checksum = ~_checksum;
                if(_checksum == inst[_data_length-1]) {
                    result = true;
                    break;
                }
                else {
                    result = false;
                    break;
                }
            }
            else { //数据头不在0位置
                for(_s = 0; _s < _real_length - _idx; _s++)
                    inst[_s] = inst[_idx + _s];
                _real_length -= _idx;
            }
        }
        else { //实际数据长度 <= 给定的数据长度，接收失败
            result = false;
            break;
        }
    }
    return result;
}

/*
函数意义：
    发送数据
参数意义：
    根据flag的状态信息发送不同种类的信息
    flag == 0x10:发送ping指令
    flag == 0x01:发送ping指令的反馈包，value表示机械臂ID
    flag == 0x30:发送写指令
    flag == 0x03:反馈写指令
    flag == 0x40:发送signal指令
    flag == 0x04:发送signal反馈包
    addr： 表示地址
    byteNum: 表示要访问的空间有几个字节
    pValue: 表示要返回的数据,前两位取固定值pValue[0]:机械臂ID
返回值：
    无返回值
*/
void ProtocolV3::sendMessage(int flag, int addr, int byteNum, UINT8_T *pValue)
{
    int i = 0, length , realLen;
    UINT8_T armID = pValue[0];
    UINT8_T checkSum = 0x00;

    //指令包
    memset(statue, 0, INST_LENGTH);

    //设置头
    statue[0] = 0xff;
    statue[1] = 0xff;

    //发送ping指令
    if(0x10 == flag) {
        statue[2] = 0x01; //机械臂个数
        statue[3] = 0x07; //数据长度
        statue[4] = 0x01; //PING指令
        statue[5] = 0x00; //error
        length = 7;
    }
    else if(0x01 == flag) {
        //发送ping的反馈包
        statue[2] = 0x06;  //数据长度
        statue[3] = armID; //机械臂ID
        statue[4] = 0x00;  //错误代码
        length = 6;
    }
    else if(0x30 == flag) {
        //发送写指令
        statue[2] = 0x01;  //ID个数
		statue[3] = 0x09; //长度
		statue[4] = 0x03;  //写指令
		statue[5] = armID; //ID
		statue[6] = (unsigned char)addr;   //Addr
        for(int _s = 1; _s <= byteNum; _s++) //Value
            statue[6+_s] = (unsigned char)pValue[_s];
		length = 8 + byteNum;
        statue[3] = (unsigned char) length;
    }
    else if(0x03 == flag) {
        //发送WRITE指令反馈包
        statue[2] = 0x08;  //长度
        statue[3] = armID; //机械臂ID
        statue[4] = (UINT8_T)addr; //Addr
        statue[5] = 0x00;          //error
        statue[6] = (UINT8_T)pValue[1];
        length = 8;
    }
    else if(0x40 == flag) {
        //发送SIGNALE指令
        statue[2] = 0x01; //舵机ID个数
        //statue[3] = 0x09; //长度
        statue[4] = 0x04; //SIGNAL指令
        statue[5] = armID;//舵机ID
        statue[6] = (UINT8_T)addr;   //Addr
        for(int _s = 1; _s <= byteNum; _s++) //Addr上的值
            statue[6+_s] = pValue[_s];
        length = 8 + byteNum;
        statue[3] = (UINT8_T)length;
    }

    //计算和校验
    for(i = 2; i <= length - 2; i++)
        checkSum += statue[i];
    statue[length - 1] = ~checkSum;

    /*发送数据*/
    while (true) {
        realLen = pPort->WritePort(statue, length);
        if(realLen != length) {
            printf("%s: %d: failed send data\r", __FILE__, __LINE__);
            fflush(stdout);
        }
        else{
            printf("%s: %d: success send packet: ", __FILE__, __LINE__);
            for(i = 0; i < realLen; i++)
                printf("%x ", statue[i]);
            printf("\n"); fflush(stdout);
            break;
        }
    }
}

//获取指令
UINT8_T ProtocolV3::getInst(int _index)
{
    return inst[_index];
}

//清空数据发送短
void ProtocolV3::clearSendPort()
{
    pPort->clearSendPort();
}

//清空数据接收端
void ProtocolV3::clearReceivePort()
{
    pPort->ClearReceivePort();
}

//关闭端口
void ProtocolV3::closePort()
{
    pPort->ClosePort();
}
