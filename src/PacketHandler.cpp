#include <iostream>
#include <string.h>
#include <stdio.h>
#include "Dmath.h"
#include "PortHandler.h"
#include "RobotisDef.h"
#include "PacketHandler.h"

using namespace std;
using namespace ROBOTIS;

#define BROADCAST_ID        (0xFE)    // 254
#define MAX_ID              (0xFC)    // 252

#define HEADER0             (0)
#define HEADER1             (1)
#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)

/* Instruction for DXL Protocol1.0 */
#define INST_PING       (1)
#define INST_READ       (2)
#define INST_WRITE      (3)
#define INST_REG_WRITE  (4)
#define INST_ACTION     (5)
#define INST_SYNC_WRITE (131)

//ERROR TYPE
#define ERRBIT_VOLTAGE      (1)  //bit0
#define ERRBIT_ANGLE        (2)  //bit1
#define ERRBIT_OVERHEAT     (4)  //bit2
#define ERRBIT_RANGE        (8)  //bit3
#define ERRBIT_CHECKSUM     (16) //bit4
#define ERRBIT_OVERLOAD     (32) //bit5
#define ERRBIT_INSTRUCTION  (64) //bit6

#define COMM_TXSUCCESS      (0)
#define COMM_RXSUCCESS      (1)
#define COMM_TXFAIL         (2)
#define COMM_RXFAIL         (3)
#define COMM_TXERROR        (4)
#define COMM_RXWAITING      (5)
#define COMM_RXTIMEOUT      (6)
#define COMM_RXCORRUPT      (7)

/****************************************
函数原型
    PacketHandler(char deviceIndex[], int baud)
参数意义：
    deviceName:端口设备名称
    baud:发送数据的波特率
返回值：
    无返回值
函数意义：
    初始化变量，设置串口路径和波特率
*****************************************/
PacketHandler::PacketHandler(char deviceName[], int baud)
{
    pPort = new PortHandler(deviceName, baud); //申请串口设备
    memset(gInstructPacket, 0, MAXNUM_RXPARAM);
    memset(gStatusPacket, 0, MAXNUM_RXPARAM);
    gCommStatus = COMM_RXSUCCESS;
    gIsUsing = false;//管道未被占用
}

//是否成功初始化串口设备
//成功初始化，返回true
//初始化失败，返回false
bool PacketHandler::isSuccessInit()
{
    return pPort->OpenPort();
}

void PacketHandler::clear_port()
{
    pPort->ClearPort();
}

//关闭串口设备
void PacketHandler::packet_terminate()
{
    pPort->ClearPort();   //清空端口
    pPort->ClosePort();   //关闭串口
}

/*****************************************
函数意义：
    设置接收数据的偏置等待时间
函数意义：
    _tm：等待时间
返回值：
    无返回值
******************************************/
void PacketHandler::setBiasWaitTime(float _tm)
{
    pPort->setBiasWaitTime(_tm);
}

/*****set packet methods*********/
//设置舵机ID
void PacketHandler::set_txpacket_id(UINT8_T id)
{
    gInstructPacket[ID] = id;
}

//设置命令
void PacketHandler::set_txpacket_instruct(UINT8_T instruct)
{
    gInstructPacket[INSTRUCTION] = instruct;
}

//设置数值
void PacketHandler::set_txpacket_parameter(UINT8_T index, UINT8_T value)
{
    gInstructPacket[PARAMETER + (int)index] = value;
}

//设置指令包的长度
void PacketHandler::set_txpacket_length(UINT8_T length)
{
    gInstructPacket[LENGTH] = length;
}

/****utility for value ***********/
int PacketHandler::dxl_makeword(int lowbyte, int highbyte)
{
    unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int PacketHandler::dxl_get_lowbyte(int word)
{
    unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int PacketHandler::dxl_get_highbyte(int word)
{
    unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

/*****get packet methods*********/
bool PacketHandler::get_rxpacket_error(UINT8_T errbit)
{
    if(gStatusPacket[ERRBIT] & errbit)
        return true;
    else
        return false;
}

UINT8_T PacketHandler::get_rxpacket_length(void)
{
    return gStatusPacket[LENGTH];
}

UINT8_T PacketHandler::get_rxpacket_parameter(UINT8_T index)
{
    return gStatusPacket[PARAMETER + (int)index];
}

/****packet communication methods***************/
int PacketHandler::tx_packet(void)
{
    UINT8_T _checksum = 0;
    UINT8_T _total_packet_length = gInstructPacket[LENGTH] + 4;
    UINT8_T _written_packet_length = 0;

    //判断管道是否被占用
    while(gIsUsing)
        printf("%s: %d: tx_packet gIsUsing is true\n", __FILE__, __LINE__);
    gIsUsing = true;    //设置本函数占用管道

    //check max packet length
    if(_total_packet_length > MAXNUM_TXPARAM) {
        gIsUsing = false;
        gCommStatus = COMM_TXERROR;
        return COMM_TXERROR;
    }

    //检查命令是否错误
    if(gInstructPacket[INSTRUCTION] != INST_PING
       && gInstructPacket[INSTRUCTION] != INST_READ
       && gInstructPacket[INSTRUCTION] != INST_WRITE
       && gInstructPacket[INSTRUCTION] != INST_ACTION
       && gInstructPacket[INSTRUCTION] != INST_REG_WRITE
       && gInstructPacket[INSTRUCTION] != INST_SYNC_WRITE) {
           gIsUsing = false;
           gCommStatus = COMM_TXERROR;
           return COMM_TXERROR;
       }

    //make packet header
    gInstructPacket[HEADER0] = 0xff;
    gInstructPacket[HEADER1] = 0xff;

    //add a checksum to the packet:except header，checksum
    for(int _idx = 2; _idx < _total_packet_length - 1; _idx++)
        _checksum += gInstructPacket[_idx];
    gInstructPacket[_total_packet_length - 1] = ~_checksum;

    //现实发送的指令

    printf("%s: %d: send inst: ", __FILE__, __LINE__);
    for(int __i = 0; __i < _total_packet_length; __i++)
        printf("%x ", gInstructPacket[__i]);
    printf("\n");


    //tx packet
    _written_packet_length = pPort->WritePort((UINT8_T*)gInstructPacket, _total_packet_length);
    if(_total_packet_length != _written_packet_length) {
        gIsUsing = false;
        gCommStatus = COMM_TXFAIL;
        return COMM_TXFAIL;
    }

    //设置延时时间
    //超时时间=本指令长度+反馈包长度
    if(gInstructPacket[INSTRUCTION] == INST_READ)
        pPort->SetPacketTimeout((UINT16_T)(gInstructPacket[LENGTH] + gInstructPacket[PARAMETER + 1] + 10));
    else
        pPort->SetPacketTimeout((UINT16_T)(gInstructPacket[LENGTH] + 10));

    gCommStatus = COMM_TXSUCCESS;
    return COMM_TXSUCCESS;
}

int PacketHandler::rx_packet(void)
{
    int _result = COMM_RXFAIL;
    UINT8_T _checksum = 0;
    UINT8_T _rx_length = 0;
    UINT8_T _wait_length = 6;
    UINT8_T retNum = 0;
    //minimum length(HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM)

    //检测是否已经有write命令
    if(!gIsUsing) {
        printf("%s: %d rx_packet:gIsUsing is false\n", __FILE__, __LINE__);
        gIsUsing = false;
        gCommStatus = COMM_RXFAIL;
        _result = COMM_RXFAIL;
        return COMM_RXFAIL;
    }

    //检测发送指令是广播信号
    if(gInstructPacket[ID] == BROADCAST_ID) {
        gCommStatus = COMM_RXSUCCESS;
        gIsUsing = false;
        _result = COMM_RXSUCCESS;
        return COMM_RXSUCCESS;
    }

    //printf("%s: %d: recv Statue: \n", __FILE__, __LINE__);
    while (true) {
        //从管道读取数据
        retNum = pPort->ReadPort((UINT8_T*)&gStatusPacket[_rx_length], _wait_length - _rx_length);
        _rx_length += retNum;

        //显示接收到的数据
        for(int __i = _rx_length - retNum; __i < _rx_length; __i++)
            printf("%x ", gStatusPacket[__i]);

        //接收数据大于最小数据量
        if(_rx_length >= _wait_length) {
            UINT8_T _idx = 0;

            //find packet header
            for(_idx = 0; _idx < (_rx_length - 1); _idx++) {
                if(gStatusPacket[_idx] == 0xFF && gStatusPacket[_idx+1] == 0xFF)
                    break;
            }

            //found at the beginning of the packet
            if(0 == _idx) {
                //检测ID不正确
                if(gStatusPacket[ID] != gInstructPacket[ID]) {
                    for(UINT8_T _s = 0; _s < _rx_length; _s++)
                        gStatusPacket[_s] = gInstructPacket[1 + _s];
                    _rx_length -= 1;
                    continue;
                }

                //重新计算接收数据包的长度
                _wait_length = gStatusPacket[LENGTH] + 4;
                if(_rx_length < _wait_length) {
                    continue;
                }

                //calculate checksum
                for(int _i = 2; _i < _wait_length - 1; _i++)
                    _checksum += gStatusPacket[_i];
                _checksum = ~_checksum;
                if(gStatusPacket[_wait_length - 1] == _checksum) {
                    gCommStatus = COMM_RXSUCCESS;
                    _result = COMM_RXSUCCESS;
                }
                else {
                    gCommStatus = COMM_RXCORRUPT;
                    _result = COMM_RXCORRUPT;
                }
                break;
            }
            else{
                //header不在_idx = 0处
                //移除非必须的数据
                for(UINT8_T _s = 0; _s < _rx_length - _idx; _s++)
                    gStatusPacket[_s] = gStatusPacket[_idx + _s];
                _rx_length -= _idx;
            }
        }
        else {
            //接收的数据小于最小的数据量
            //检测数据超时
            if(pPort->IsPacketTimeout()) {
                if(_rx_length == 0) {
                    gCommStatus = COMM_RXTIMEOUT;
                    _result = COMM_RXTIMEOUT; //接收数据包超时
                }
                else {
                    gCommStatus = COMM_RXCORRUPT;
                    _result = COMM_RXCORRUPT; //接收数据包错误
                }
                break;
            }
        }
    }
    gIsUsing = false;
    return _result;
}

/**********************************
函数意义:
    给舵机发送指令
参数意义:
    无参数
返回值：
    如果执行成功，返回true;
    如果执行失败，返回false;
**********************************/
bool PacketHandler::txrx_packet(void)
{
    //清空端口
    //clear_port();

    //发送指令
    tx_packet();
    if(gCommStatus != COMM_TXSUCCESS)
        return false;

    do{
        rx_packet();
    }while(gCommStatus == COMM_RXWAITING);
    if (gCommStatus != COMM_RXSUCCESS)
        return false;

    return true;
}

UINT8_T PacketHandler::get_result(void)
{
    return gCommStatus;
}

/*********High communication methods*********/
bool PacketHandler::ping(UINT8_T id)
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_PING;
    gInstructPacket[LENGTH] = 2;

    return txrx_packet();
}

UINT8_T PacketHandler::read_byte(UINT8_T id, UINT8_T addr)
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);

    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_READ;
    gInstructPacket[PARAMETER] = addr;
    gInstructPacket[PARAMETER + 1] = 1;
    gInstructPacket[LENGTH] = 4;

    txrx_packet();

    return gStatusPacket[PARAMETER];
}

bool PacketHandler::write_byte(UINT8_T id, UINT8_T addr, UINT8_T value)
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_WRITE;
    gInstructPacket[PARAMETER] = addr;
    gInstructPacket[PARAMETER + 1] = value;
    gInstructPacket[LENGTH] = 4;
    return txrx_packet();
}

UINT16_T PacketHandler::read_word(UINT8_T id, UINT8_T addr)
{
    UINT16_T result;
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_READ;
    gInstructPacket[PARAMETER] = addr;
    gInstructPacket[PARAMETER + 1] = 2;
    gInstructPacket[LENGTH] = 4;
    txrx_packet();
    result = DXL_MAKEWORD(gStatusPacket[PARAMETER], gStatusPacket[PARAMETER + 1]);
    return result;
}

bool PacketHandler::write_word(UINT8_T id, UINT8_T addr, UINT16_T value)
{
    //清空指令包
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);

    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_WRITE;
    gInstructPacket[PARAMETER] = addr;
    gInstructPacket[PARAMETER + 1] = DXL_LOBYTE(value);
    gInstructPacket[PARAMETER + 2] = DXL_HIBYTE(value);
    gInstructPacket[LENGTH] = 5;

    return txrx_packet();
}

//函数意义：
//  同时给多个舵机写入byte值
//参数意义：
//  _servoNum：舵机的个数
//  id[]：舵机的编号
//  addr：属性地址
//  _value：要写入的值
//返回值：
//  写入成功，返回true；
//  写入失败，返回false;
bool PacketHandler::sync_write_byte(int _servoNum, int id[], UINT8_T addr, UINT8_T _value[])
{
    //清空指令包
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);
    //Make syncwrite packet
    set_txpacket_id((UINT8_T)BROADCAST_ID);
    set_txpacket_instruct((UINT8_T)INST_SYNC_WRITE);
    set_txpacket_parameter(0, (UINT8_T)addr);
    set_txpacket_parameter(1, (UINT8_T)1);
    for(int i = 0; i < _servoNum; i++) {
        set_txpacket_parameter(2 + 2 * i, (UINT8_T)id[i]);
        set_txpacket_parameter(2 + 2 * i + 1, _value[i]);
    }
    set_txpacket_length((UINT8_T)(2 * _servoNum + 4));

    return txrx_packet();
}

//函数意义：
//  同时给多个舵机写入word值
//参数意义：
//  _servoNum：舵机的个数
//  id[]：舵机的编号
//  addr：属性地址
//  _value：要写入的值
//返回值：
//  写入成功，返回true；
//  写入失败，返回false;
bool PacketHandler::sync_write_word(int _servoNum, int id[], UINT8_T addr, UINT16_T _value[])
{
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);

    set_txpacket_id(BROADCAST_ID);
    set_txpacket_instruct(INST_SYNC_WRITE);
    set_txpacket_parameter(0, addr);
    set_txpacket_parameter(1, 2);
    for(int i = 0; i < _servoNum; i++) {
        set_txpacket_parameter(2 + 3 * i, (UINT8_T)id[i]);
        set_txpacket_parameter(2 + 3 * i + 1, DXL_LOBYTE(_value[i]));
        set_txpacket_parameter(2 + 3 * i + 2, DXL_HIBYTE(_value[i]));
        //printf("%s: %d: i(%d) DXL_LOBYTE(%x) DXL_HIBYTE(%x)\n", __FILE__, __LINE__, i, DXL_LOBYTE(_value[i]), DXL_HIBYTE(_value[i]));
    }
    set_txpacket_length((2 + 1) * _servoNum + 4);
    return txrx_packet();
}

//设置scara末端的高度
void PacketHandler::set_pen_height(int id, int address, int value)
{
    //清空指令包
    memset(gInstructPacket, 0, MAXNUM_TXPARAM);

    gInstructPacket[ID] = id;
    gInstructPacket[INSTRUCTION] = INST_WRITE;
    gInstructPacket[PARAMETER] = (unsigned char)address;
    gInstructPacket[PARAMETER + 1] = DXL_LOBYTE(value);
    gInstructPacket[PARAMETER + 2] = DXL_HIBYTE(value);
    gInstructPacket[LENGTH] = 5;

    tx_packet();

    gIsUsing = 0;
}

//打印通信状态
void PacketHandler::PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
	case COMM_TXFAIL:
		printf("\rCOMM_TXFAIL: Failed transmit instruction packet!");
		break;

	case COMM_TXERROR:
		printf("\rCOMM_TXERROR: Incorrect instruction packet!");
		break;

	case COMM_RXFAIL:
		printf("\rCOMM_RXFAIL: Failed get status packet from device!");
		break;

	case COMM_RXWAITING:
		printf("\rCOMM_RXWAITING: Now recieving status packet!");
		break;

	case COMM_RXTIMEOUT:
		printf("\rCOMM_RXTIMEOUT: There is no status packet!");
		break;

	case COMM_RXCORRUPT:
		printf("\rCOMM_RXCORRUPT: Incorrect status packet!");
		break;

	default:
		printf("\rThis is unknown error code!");
		break;
	}
    fflush(stdout);
}

//打印错误码
void PacketHandler::PrintErrorCode(void)
{
    if(get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("\nInput voltage error!");

	if(get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("\nAngle limit error!");

	if(get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("\nOverheat error!");

	if(get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("\nOut of range error!");

	if(get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("\nChecksum error!");

	if(get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("\nOverload error!");

	if(get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("\nInstruction code error!");

	fflush(stdout);
}
