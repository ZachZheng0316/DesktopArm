#ifndef REALTIMEPROHEADER_H_
#define REALTIMEPROHEADER_H_

#include <stdio.h>
#include "RobotisDef.h"
#include "SerialCommuni.h"

using namespace ROBOTIS;

namespace ROBOTIS{
class RealTimePro{
private:
    int x;
    int y;//坐标
    char flag; //运动或者停止的标志
    long Num;

    UINT8_T* pPacket;
    SerialCommuni* pCommuni;

public:
    RealTimePro(char *deviceName, int baudNum);
    ~RealTimePro();
    bool openProSuccess();
    bool readProSuccess();               //成功读取协议数据
    bool anaProSuccess(UINT8_T* packet); //成功分析出协议数据
    void closeProSucess();  //关闭协议端口

    void savePacket(FILE* fp);
    int returnX();
    int returnY();
    char returnFlag();
};
}


#endif
