#ifndef LEAPCONTROLPROHEADER_H_
#define LEAPCONTROLPROHEADER_H_

#include <stdio.h>
#include "RobotisDef.h"
#include "PortHandler.h"

using namespace ROBOTIS;

namespace ROBOTIS{
class LeapControlPro{
private:
    int xyz3D[3];

    int bengOpen;
    UINT8_T* pPacket;
    PortHandler* pPort;

public:
    LeapControlPro(int deviceIndex, int baudNum);
    ~LeapControlPro();
    bool openProSuccess();
    bool readProSuccess();  //成功读取协议数据
    void anaProSuccess();   //成功分析出协议数据
    void closeProSucess();  //关闭协议端口

    void savePacket(FILE* fp); //存储数据到文件fp中
    void returnXYZ3D(int value[], char _bengOpen);
};
}


#endif
