#ifndef PACKETHANDLER_H_
#define PACKETHANDLER_H_

#include "RobotisDef.h"
#include "PortHandler.h"

#define MAXNUM_TXPARAM  (160) //指令包的最大长度
#define MAXNUM_RXPARAM  (80)  //状态包的最大长度

namespace ROBOTIS
{
class PacketHandler
{
private:
    UINT8_T gInstructPacket[MAXNUM_TXPARAM]; //指令数据包
    UINT8_T gStatusPacket[MAXNUM_RXPARAM]; //状态数据包
    INT32_T gCommStatus; //通信状态
    bool gIsUsing;       //管道被占用标志
    PortHandler *pPort;  //串口指针

public:
    PacketHandler(char deviceName[], int baud);
    ~PacketHandler();

    bool isSuccessInit(); //成功初始化
    void packet_terminate();
    void clear_port();
    void setBiasWaitTime(float _tm);

    /*******set packet methods*********/
    void set_txpacket_id(UINT8_T id); //设置舵机ID
    void set_txpacket_instruct(UINT8_T instruct); //设置命令
    void set_txpacket_parameter(UINT8_T index, UINT8_T value);//设置数值
    void set_txpacket_length(UINT8_T length); //指令包的长度

    /*****get packet methods*********/
    bool get_rxpacket_error(UINT8_T errbit);
    UINT8_T get_rxpacket_length(void);
    UINT8_T get_rxpacket_parameter(UINT8_T index);

    /****utility for value ***********/
    int dxl_makeword(int lowbyte, int highbyte);
    int dxl_get_lowbyte(int word);
    int dxl_get_highbyte(int word);

    /****packet communication methods***************/
    int tx_packet(void);
    int rx_packet(void);
    bool txrx_packet(void);
    UINT8_T get_result(void);

    /*********High communication methods*********/
    bool ping(UINT8_T id);
    UINT8_T read_byte(UINT8_T id, UINT8_T addr);
    bool write_byte(UINT8_T id, UINT8_T addr, UINT8_T value);
    UINT16_T read_word(UINT8_T id, UINT8_T addr);
    bool write_word(UINT8_T, UINT8_T addr, UINT16_T value);
    bool sync_write_byte(int _servoNum, int id[], UINT8_T addr, UINT8_T _value[]);
    bool sync_write_word(int _servoNum, int id[], UINT8_T addr, UINT16_T _value[]);
    void set_pen_height(int id, int address, int value); //设置scara末端的高度
    void PrintCommStatus(int CommStatus); //打印通信状态
    void PrintErrorCode(void); //打印错误码
};
}

#endif
