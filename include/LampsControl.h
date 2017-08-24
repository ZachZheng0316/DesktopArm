#ifndef LAMPSCONTROLHEADER__H__
#define LAMPSCONTROLHEADER__H__

#include "LampsProtocol.h"

namespace ROBOTIS{
class LampsControl{
private:
    LampsProtocol *pLampsPro;

public:
    LampsControl(char deviceName[], int baud);
    bool SuccessOpenDevice(); //检测端口设备是否可以打开

    int set_Lamps_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T* _value, UINT8_T _byteNum);
    int get_Lamps_bytes(UINT8_T _id, UINT8_T _addr, UINT8_T _byteNum);
    UINT8_T read_Lamps_StatusPacket(int _index);

    bool returnServoAnomaly(); //检测灯具异常
    int returnCommuAnomaly(int _commStatus); //检测通信异常
};

}

#endif
