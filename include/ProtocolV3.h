#ifndef PROTOCOLHEADER__H__
#define PROTOCOLHEADER__H__

#include "RobotisDef.h"
#include "PortHandler.h"

#define INST_LENGTH (64)

namespace ROBOTIS{
class ProtocolV3{
private:
    UINT8_T inst[INST_LENGTH];   //存储指令数据
    UINT8_T statue[INST_LENGTH]; //存储状态数据包

    PortHandler *pPort;

public:
    ProtocolV3(char deviceName[], int baud);
    ~ProtocolV3();

    bool isSuccessInit();       //成功初始化
    bool receiveMessage(int flag);  //接受数据
    void sendMessage(int flag, int addr, int valueNum, UINT8_T *pValue); //发送数据

    UINT8_T getInst(int _index); //获取指令
    void clearSendPort();    //清空数据发送短
    void clearReceivePort(); //清空数据接收端
    void closePort();        //关闭端口
};
}

#endif
