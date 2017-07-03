#ifndef LAMPSPROTOCOLHANDLER_H_
#define LAMPSPROTOCOLHANDLER_H_

#include "RobotisDef.h"
#include "PortHandler.h"

#define MAXNUM_TXPARAM  (160) //指令包的最大长度
#define MAXNUM_RXPARAM  (160)  //状态包的最大长度

namespace ROBOTIS
{
class LampsProtocol
{
private:
    UINT8_T gInstructPacket[MAXNUM_TXPARAM]; //指令数据包
    UINT8_T gStatusPacket[MAXNUM_RXPARAM];   //状态数据包
    INT32_T gCommStatus;                     //通信状态
    bool gIsUsing;                           //管道被占用标志
    PortHandler *pPort;                      //串口指针

public:
    LampsProtocol(char deviceName[], int baud);
    ~LampsProtocol();

    bool isSuccessInit();   //成功初始化
    void packet_terminate();

    /*******set packet methods*********/
    void set_txpacket_id(UINT8_T id); //设置舵机ID
    void set_txpacket_instruct(UINT8_T instruct); //设置命令
    void set_txpacket_parameter(UINT8_T index, UINT8_T value); //设置数值
    void set_txpacket_length(UINT8_T length); //指令包的长度

    /*****get packet methods*********/
    bool get_rxpacket_error(UINT8_T errbit);
    UINT8_T get_rxpacket_length(void);
    UINT8_T get_rxpacket_parameter(UINT8_T index);

    /****packet communication methods***************/
    bool tx_packet(void);
    bool rx_packet(void);
    bool txrx_packet(void);
    UINT8_T get_result(void);

    /*********High communication methods*********/
    bool read_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T _byteNum);
    bool write_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T *_value, UINT8_T _byteNum);
    void setBiasRcvWaitTime(float _time);
};
}
#endif
