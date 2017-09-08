#ifndef PORTHANDLER__H__
#define PORTHANDLER__H__

#include "RobotisDef.h"

namespace ROBOTIS
{
class PortHandler
{
private:
    int socket_fd_;		   //串口ID
    int baudrate_;		   //波特率数值
    char deviceName[30];   //设备路径

    double packet_start_time_; //数据包开始发送时间
    double packet_timeout_;	   //数据包超时时间
    double tx_time_per_byte;   //发送数据包每byte的时间
    float  biasWaitTime;

    int GetCFlagBaud(const int baudrate); //获取波特率标志
    double GetCurrentTime();              //获取当前时间，单位为秒
    double GetTimeSinceStart();	          //计算耗时

public:
    PortHandler(int portIndex, int baudrate);
    PortHandler(char portName[], int baudrate);
    virtual ~PortHandler() { ClosePort(); }

    bool OpenPort();	//打开串口
    void ClosePort();	//关闭串口
    void ClearPort();	//清空串口
    void ClearReceivePort(); //清空输入串口
    void clearSendPort();    //清空输出串口

    int GetBytesAvailable(); //获取要读取的数据字节数

    int ReadPort(UINT8_T *packet, int length);	//读取串口数据
    int WritePort(UINT8_T *packet, int length);	//发送串口数据

    void SetPacketTimeout(UINT16_T packet_length);//设置数据包延时时间
    void SetPacketTimeout(double msec);	          //设置数据包延时时间
    bool IsPacketTimeout();	                      //检测数据包延时
    void setBiasWaitTime(float _waitTm);          //设置偏置等待时间
};
}

#endif
