#ifndef SERIALCOMMUNIHEADER_H_
#define SERIALCOMMUNIHEADER_H_

#include "RobotisDef.h"

namespace ROBOTIS{
class SerialCommuni
{
private:
    UINT32_T serial_fd;
    UINT32_T baudrate;
    char* deviceName;

public:
    SerialCommuni(char _deviceName[], UINT32_T _baudrate);
    ~SerialCommuni();

    UINT32_T getCFlagBaud(UINT32_T baudrate);   //转化波特率输出形式

    bool openCommuniPort();
    UINT32_T receiveMessage(UINT8_T *pPacket, UINT32_T packetLen);
    UINT32_T sendMessage(UINT8_T *pPacket, UINT32_T packetLen);
    void clearCommuniPort();
    void closeCommuniPort();
};
}
#endif
