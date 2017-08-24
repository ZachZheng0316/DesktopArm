#include <iostream>
#include <string.h>
#include <stdio.h>
#include "LampsProtocol.h"
#include "RobotisDef.h"
#include "PortHandler.h"

using namespace std;
using namespace ROBOTIS;

//常见指令参数的位置
#define HEADER0 (0) //在指令和返回值中
#define HEADER1 (1) //在指令和返回值中
#define ID      (2) //在指令和返回值中
#define LENGTH  (3) //在指令和返回值中
#define INST    (4) //在指令中
#define ERRBIT  (4) //在返回值中
#define ADDR    (5) //在指令中

//指令类型
#define INST_READ   (2)
#define INST_WRITE  (3)

//通信错误
#define COMM_TXSUCCESS  (0)
#define COMM_RXSUCCESS  (1)
#define COMM_TXFAIL     (2)
#define COMM_RXFAIL     (3)
#define COMM_TXERROR    (4)
#define COMM_RXWAITING  (5)
#define COMM_RXTIMEOUT  (6)
#define COMM_RXCORRUPT  (7)

//舵机内部错误
//由返回值的EBBIT确定

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

/*******************************************
函数意义：
    申请相应的串口设备
返回值：
    无返回值
********************************************/
LampsProtocol::LampsProtocol(char _deviceName[], int baud)
{
    pPort = new PortHandler(_deviceName, baud);
    //申请串口设备
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    memset(gStatusPacket, 0, MAXNUM_RXPARAM);
    gCommStatus = COMM_RXSUCCESS;
    gIsUsing = false; //管道未被占用
}

LampsProtocol::~LampsProtocol()
{
    packet_terminate();
}

/********************************************
函数意义：
    判断串口设备是否打开成功
返回值：
    如果打开成功，返回true;
    如果打开失败，返回false;
********************************************/
bool LampsProtocol::isSuccessInit()
{
    return pPort->OpenPort();
}

/*******************************************
函数意义：
    关闭串口设备
返回值：
    无返回值
*******************************************/
void LampsProtocol::packet_terminate()
{
    pPort->ClearPort(); //清空端口
    pPort->ClosePort(); //关闭串口
}

/************set packet methods**************/
/*********************************************
函数意义：
    设置指令的ID
参数意义：
    id:灯具的id
返回值：
    无
*********************************************/
void LampsProtocol::set_txpacket_id(UINT8_T _id)
{
    gInstructPacket[ID] = _id;
}

/**********************************************************
函数意义：
    设置命令类型
参数意义：
    instruct:命令类型
返回值：
    无
***********************************************************/
void LampsProtocol::set_txpacket_instruct(UINT8_T instruct)
{
    gInstructPacket[INST] = instruct;
}

/*************************************************************
函数意义：
    设置数值
参数意义：
    index:相对ADDR的地址
    value:数值
返回值：
    无
**************************************************************/
void LampsProtocol::set_txpacket_parameter(UINT8_T index, UINT8_T value)
{
    gInstructPacket[ADDR + (int)index] = value;
}

/*****************************************************
函数意义：
    设置指令包的长度
参数意义：
    lengh:指令的长度
返回值：
    无
******************************************************/
void LampsProtocol::set_txpacket_length(UINT8_T length)
{
    gInstructPacket[LENGTH] = length;
}

/*****************************************
函数意义：
    获取状态包是否中是否存在错误
参数意义：
    errbit:错误标志位
返回值：
    如果存在错误，返回true
    如果不存在错误，返回false
*****************************************/
bool LampsProtocol::get_rxpacket_error(UINT8_T errbit)
{
    if(gStatusPacket[ERRBIT] & errbit)
        return true;
    else
        return false;
}

/*****************************************
函数的意义：
    获取状态包的长度
参数意义：
    无参数
返回值：
    无返回值
*****************************************/
UINT8_T LampsProtocol::get_rxpacket_length(void)
{
    return gStatusPacket[LENGTH];
}

/********************************************
函数意义：
    返回状态包指定地址的数据
参数意义：
    index：相对ADDR的地址
返回值：
    返回状态包指定地址的数据
********************************************/
UINT8_T LampsProtocol::get_rxpacket_parameter(UINT8_T index)
{
    return gStatusPacket[ADDR + (int)index];
}

/****packet communication methods***************/
/***********************************
函数意义：
    发送指令包
参数意义：
    无参数
返回值：
    如果发送成功，返回true；
    如果发送失败，返回false；
***********************************/
bool LampsProtocol::tx_packet(void)
{
    UINT8_T _checksum = 0;
    UINT8_T _total_packet_length = gInstructPacket[LENGTH] + 4;
    UINT8_T _written_packet_length = 0;

    //判断管道是否被占用
    if(gIsUsing) {
        gCommStatus = COMM_TXFAIL;
        printf("%s: %d: tx_packet gIsUsing is true\n", __FILE__, __LINE__);
        return false;
    }
    gIsUsing = true;

    //检查数据长度是否过长
    if(_total_packet_length > MAXNUM_TXPARAM) {
        gIsUsing = false;
        gCommStatus = COMM_TXERROR;
        printf("%s: %d: tx_packet is too long\n", __FILE__, __LINE__);
        return false;
    }

    //检查指令是否错误
    if(gInstructPacket[INST] != INST_READ
        && gInstructPacket[INST] != INST_WRITE) {
            gIsUsing = false;
            gCommStatus = COMM_TXERROR;
            printf("%s： %d: tx_packet's inst is error\n", __FILE__, __LINE__);
            return false;
        }

    //设置头文件
    gInstructPacket[HEADER0] = 0xff;
    gInstructPacket[HEADER1] = 0xff;

    //设置checksum
    for(int _idx = 2; _idx < _total_packet_length - 1; _idx++)
        _checksum += gInstructPacket[_idx];
    gInstructPacket[_total_packet_length - 1] = ~_checksum;

    //printf("%s: %d: ", __FILE__, __LINE__);
    //for(int i = 0; i < 15; i++)
    //    printf("%x ", gInstructPacket[i]);
    //printf("\n");

     //发送指令
     _written_packet_length = pPort->WritePort(gInstructPacket, _total_packet_length);
     if(_total_packet_length != _written_packet_length) {
         gIsUsing = false;
         gCommStatus = COMM_TXFAIL;
         printf("%s: %d: senf tx_packet failed\n", __FILE__, __LINE__);
         return false;
     }

     //设置延迟时间
    if(gInstructPacket[INST] == INST_READ)
        pPort->SetPacketTimeout((UINT16_T)(gInstructPacket[ADDR+1]+6));
    else
        pPort->SetPacketTimeout((UINT16_T)6);

    gCommStatus = COMM_TXSUCCESS;

    gIsUsing = true;

    return true;
}

/***********************
函数意义：
    接收数据
参数意义：
    无
返回值：
    如果执行成功，返回true
    如果执行失败，返回Failed
***********************/
bool LampsProtocol::rx_packet(void)
{
    int _result = COMM_RXFAIL;
    UINT8_T _checksum = 0;
    UINT8_T _rx_length = 0, _retNum = 0;
    UINT8_T _wait_length = 6;

    //检测gIsUsing是否正在被使用
    if(!gIsUsing) {
        gIsUsing = false;
        gCommStatus = COMM_RXFAIL;
        printf("%s: %d: gIsUsing IS Failed\n", __FILE__, __LINE__);
        return false;
    }

    //从管道读取数据
    while (true) {
        //从管道读取数据
        _retNum = pPort->ReadPort((UINT8_T*)&gStatusPacket[_rx_length], _wait_length - _rx_length);
        _rx_length += _retNum;

        //显示接收到的数据
        //for(int _index = _rx_length - _retNum; _index < _rx_length; _index++)
        //    printf("%x ", gStatusPacket[_index]);

        if(_rx_length >= _wait_length) {
            UINT8_T _idx = 0;

            //找到数据头
            for(_idx = 0; _idx < (_rx_length - 1); _idx++) {
                if(gStatusPacket[_idx] == 0xff
                    && gStatusPacket[_idx+1] == 0xff)
                    break;
            }

            //判断数据头是否在首位
            if(0 == _idx) {
                //检测id是否正确
                if(gStatusPacket[ID] != gInstructPacket[ID]) {
                    for(UINT8_T _s = 0; _s < _rx_length; _s++)
                        gStatusPacket[_s] = gInstructPacket[1+_s];
                    _rx_length -= 1;
                    continue;
                }

                //重新计算接收数据包的长度
                _wait_length = gStatusPacket[LENGTH] + 4;
                if(_rx_length < _wait_length)
                    continue;

                //计算checksum
                for(int _i = 2; _i < _wait_length - 1; _i++)
                    _checksum += gStatusPacket[_i];
                _checksum = ~_checksum;
                if(gStatusPacket[_wait_length-1]==_checksum) {
                    gCommStatus = COMM_RXSUCCESS;
                    gIsUsing = false;
                    return true;
                }
                else {
                    gCommStatus = COMM_RXCORRUPT;
                }
            }
            else{
                //数据头不在_idx = 0处
                for(UINT8_T _s = 0; _s < _rx_length - _idx; _s++)
                    gStatusPacket[_s] = gStatusPacket[_idx+_s];
                _rx_length -= _idx;
            }
        }
        else{
            //检测数据是否超时
            if(pPort->IsPacketTimeout()) {
                if(_rx_length == 0)
                    gCommStatus = COMM_RXTIMEOUT;
                else
                    gCommStatus = COMM_RXCORRUPT;
                gIsUsing = false;
                return false;
            }
        }
    }
}

/**********************************
函数意义：
    发送指令并接收状态包
参数意义：
    无
返回值：
    如果执行成功，返回true；
    如果执行失败，返回false;
**********************************/
bool LampsProtocol::txrx_packet(void)
{
    //发送指令
    tx_packet();
    if(gCommStatus != COMM_TXSUCCESS)
        return false;

    do{
        rx_packet();
    }while(gCommStatus == COMM_RXWAITING);
    if(gCommStatus != COMM_RXSUCCESS)
        return false;
    return true;
}

UINT8_T LampsProtocol::get_result(void)
{
    return gCommStatus;
}

/******************************************
函数意义：
    读取灯具的寄存器数据
参数意义：
    _id:灯具ID
    _addr:寄存器起始地址
    _byteNum:寄存器的个数
返回值：
    如果执行成功，返回true；
    如果执行失败，返回false。
*******************************************/
bool LampsProtocol::read_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T _byteNum)
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);

    gInstructPacket[ID] = _id;
    gInstructPacket[INST] = INST_READ;
    gInstructPacket[ADDR] = _addr;
    gInstructPacket[ADDR+1] = _byteNum;
    gInstructPacket[LENGTH] = 4;

    return txrx_packet();
}

/******************************************
函数意义：
    在灯具寄存器中写入数据
参数意义：
    _id:灯具id
    _addr:寄存器起始地址
    _value:要写入的数据
    _byteNum:要写入数据的个数
返回值：
    如果执行成功，返回true；
    如果执行失败，返回false。
******************************************/
bool LampsProtocol::write_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T *_value, UINT8_T _byteNum)
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    gInstructPacket[ID] = _id;
    gInstructPacket[INST] = INST_WRITE;
    gInstructPacket[ADDR] = _addr;
    for(int _index = 1; _index <= _byteNum; _index++)
        gInstructPacket[ADDR+_index] = _value[_index-1];
    gInstructPacket[LENGTH] =  3+_byteNum;

    return txrx_packet();
}

//设置等待延迟时间
void LampsProtocol::setBiasRcvWaitTime(float _time)
{
    pPort->setBiasWaitTime(_time);
}
