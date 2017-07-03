#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "PortHandler.h"
#include "RobotisDef.h"

#define LATENCY_TIMER (4) //msec (USB latency timer)
#define TX_TIME_PER_BYTE_FACTOR	(12.0) //每byte发送时间因子
// 发送指令包完成，至开始返回状态包时，中间有一个反馈包相应时间
#define RESPONSE_TIME   (3.5) //反馈包的响应时间

using namespace ROBOTIS;
/****************************************
函数原型
    PortHandler(int portIndex, int baudrate)
参数意义：
    portIndex:ttyUSBxx中的xx
    baudrate:发送数据的波特率
返回值：
    无返回值
函数意义：
    初始化变量，设置串口路径和波特率
*****************************************/
PortHandler::PortHandler(int portIndex, int baudrate)
{
    socket_fd_ = -1;
    baudrate_ = baudrate;
    packet_start_time_ = 0.0;
    packet_timeout_ = 0.0;
    tx_time_per_byte = 0.0;
    biasWaitTime = 5.0f;
    if(portIndex == 0)
        sprintf(deviceName, "/dev/ttyUSB%d", portIndex);
    else
        sprintf(deviceName, "/dev/ttyS%d", portIndex);
}

/****************************************
函数意义：
    初始化变量，设置串口路径和波特率
参数意义：
    portName：port路径
    baudrate:波特率
返回值：
    返回值
****************************************/
PortHandler::PortHandler(char portName[], int baudrate)
{
    socket_fd_ = -1;
    baudrate_ = baudrate;
    packet_start_time_ = 0.0;
    packet_timeout_ = 0.0;
    tx_time_per_byte = 0.0;
    biasWaitTime = 5.0f;
    strcpy(deviceName, portName);
}

/****************************************
函数原型
    OpenPort()
返回值：
    如打开成功，返回true
    如打开失败，返回false
函数意义：
    设置串口的硬件参数，并打开串口设备
*****************************************/
bool PortHandler::OpenPort()
{
    struct termios newtio;
    struct serial_struct serinfo;
    int _baud;

    //清理数据
    memset(&newtio, 0, sizeof(newtio));
    ClosePort();

    //获得波特率
    _baud = GetCFlagBaud(baudrate_);
    if(_baud == -1) {
        printf("%s: %d: PortHandler::OpenPort:_baud = %d\n", __FILE__, __LINE__, _baud);
        return false;
    }
    //printf("PortHandler::OpenPort:baudrate_= %d;baud = %d\n",baudrate_,  _baud);

    //打开串口
    /* 打开串口设备：
     * O_RDWR：表示对串口可以进行读写控制，
     * O_NOCTTY：表示打开的是终端设备.就是不把这个设备,作为控制终端。
     如果是控制终端，键盘上的Ctrl+C会产生终止信号
     * O_NONBLOCK: 表示非阻塞模式。O_NDELAY表示阻塞模式。
     阻塞的定义：对于read，当串口输入缓冲区没有数据时，read函数就会阻塞在那里;
     当串口数据有数据时，read都到需要的字节后，返回读到的字节数。对于write函数，
     串口输出缓冲区满，或剩下的空间小于要写入的字节数，write将阻塞，一直到串口
     输出缓冲区大于要写入的字节数，执行写入操作，返回写入的字节数。
     非阻塞的含义：对于read，当缓冲区没有数据的时候，read函数立即返回，返回值为0
     对于write函数，当串口输出缓冲区满或剩下的空间小于要写入的字节数，则write还是
     将进行写操作，写入当前串口允许写入的字节数，返回实际写入的字节数。
    */
    if((socket_fd_ = open(deviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
        printf("%s: %d: device open error:%s\n", __FILE__, __LINE__, deviceName);
        return false;
    }
    //else
    //    printf("PortHandler::OpenPort:socket_fd_ = %d success\n", socket_fd_);


    newtio.c_cflag = _baud|CS8|CLOCAL|CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    /*设置波特率*/
    //cfsetispeed(&newtio, _baud);
    //cfsetospeed(&newtio, _baud);

    /*设置数据位-8位*/
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    /*设置奇偶位-无奇偶校验*/
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag &= ~INPCK;

    /*设置停止位-1位停止位*/
    newtio.c_cflag &= ~CSTOPB;

    /*设置流控-无*/
    newtio.c_iflag &= ~(ICRNL | IXON | IXOFF);

    //clean the buffer and activate the settings for port
    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);
    tx_time_per_byte = (1000.0 / (double)baudrate_) * TX_TIME_PER_BYTE_FACTOR;

    if(ioctl(socket_fd_, TIOCGSERIAL, &serinfo) < 0) {
        printf("PortHandler::OpenPort:Cannot get serial info\n");
        return false;
    }
    //else
    //    printf("PortHandler::OpenPort:ioctl success\n");
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = (serinfo.baud_base + (baudrate_ / 2)) / baudrate_;
    int closest_speed = serinfo.baud_base / serinfo.custom_divisor;

	//检查串口设置是否正确
    if(closest_speed < baudrate_ * 98 / 100 || closest_speed > baudrate_ * 102 / 100) {
        printf("PortHandler::OpenPort:Cannot set speed to %d, closest is %d \n", baudrate_, closest_speed);
        return false;
    }
    if(ioctl(socket_fd_, TIOCSSERIAL, &serinfo) < 0) {
        printf("PortHandler::OpenPort:TIOCSSERIAL failed!\n");
        return false;
    }
    //else
    //    printf("PortHandler::OpenPort:TIOCSSERIAL success\n");
    //tx_time_per_byte = (1000.0 / (double)baudrate_) * TX_TIME_PER_BYTE_FACTOR;

    ClosePort();
    memset(&newtio, 0, sizeof(newtio));

    if((socket_fd_ = open(deviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
        printf("PortHandler::OpenPort:device open error: %s\n", deviceName);
        return false;
    }
    //else
    //    printf("PortHandler::OpenPort:device open success again:socket_fd_ = %d\n", socket_fd_);
    newtio.c_cflag = _baud|CS8|CLOCAL|CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    /*设置波特率*/
    //cfsetispeed(&newtio, _baud);
    //cfsetospeed(&newtio, _baud);

    /*设置数据位-8位*/
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    /*设置奇偶位-无奇偶校验*/
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag &= ~INPCK;

    /*设置停止位-1位停止位*/
    newtio.c_cflag &= ~CSTOPB;

    /*设置流控-无*/
    newtio.c_iflag &= ~(ICRNL | IXON | IXOFF);

    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);

    return true;
}

/****************************************
函数原型
    ClosePort()
参数意义：
    无
返回值：
    无
函数意义：
    关闭串口
*****************************************/
void PortHandler::ClosePort()
{
    if(socket_fd_ != -1)
        close(socket_fd_);
    socket_fd_ = -1;
}

//清空串口
void PortHandler::ClearPort()
{
    tcflush(socket_fd_, TCIFLUSH);
    tcflush(socket_fd_, TCOFLUSH);
    //tcflush(socket_fd_, TCIOFLUSH);
}

//清空输入串口
void PortHandler::ClearReceivePort()
{
    tcflush(socket_fd_, TCIFLUSH);
}

//清空输出串口
void PortHandler::clearSendPort()
{
    tcflush(socket_fd_, TCIOFLUSH);
}

//读取硬件设备中存储的字节数，并返回这个字节数
int PortHandler::GetBytesAvailable()
{
    int _bytes_available;
    ioctl(socket_fd_, FIONREAD, &_bytes_available);
    return _bytes_available;
}

/****************************************
函数原型
    ReadPort(UINT8_T *packet, int length)
参数意义：
    packet:存储要读取length的字节的空间
    length:要读取字节的数量
返回值：
    返回实际读取的字节数量，如果返回-1；表示读取失败
函数意义：
    读取数据
*****************************************/
int PortHandler::ReadPort(UINT8_T *packet, int length)
{
    memset(packet, 0, length);
    return read(socket_fd_, packet, length);
}

/****************************************
函数原型
    WritePort(UINT8_T *packet, int length)
参数意义：
    packet:存储要发送字节的空间
    length:要发送字节的数量
返回值：
    返回实际发送的字节数量，如果返回-1；表示发送失败
函数意义：
    发送数据
*****************************************/
int PortHandler::WritePort(UINT8_T *packet, int length)
{
    return write(socket_fd_, packet, length);
}

//设置开始时间，和超时时间
void PortHandler::SetPacketTimeout(UINT16_T packet_length)
{
    //计算延迟时间(ms) = 数据发送时间 + 硬件延迟时间 + 反馈包时间 + 偏置时间
    packet_timeout_ = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + tx_time_per_byte * RESPONSE_TIME + biasWaitTime;
    packet_start_time_  = GetCurrentTime(); //获取当前时间
}

//设置开始时间和超时时间
void PortHandler::SetPacketTimeout(double msec)
{
    packet_start_time_ = GetCurrentTime();
    packet_timeout_ = msec;
}

/****************************************
函数原型
    IsPacketTimeout
参数意义：
    无
返回值：
    如果发送数据超时，返回true;
    发送数据未超时，返回false
函数意义：
    检测发送数据包是否超时
*****************************************/
bool PortHandler::IsPacketTimeout()
{
    if(GetTimeSinceStart() > packet_timeout_) {
        //printf("%s: %d: packet_timeout_(%lf)\n", __FILE__, __LINE__, packet_timeout_);
        packet_timeout_ = 0;
        return true;
    }
    return false;
}

//获取当前的时间(msec)
double PortHandler::GetCurrentTime()
{
	struct timespec _tv;
	clock_gettime( CLOCK_REALTIME, &_tv);
    //以ms为单位
	return ((double)_tv.tv_sec * 1000.0 + (double)_tv.tv_nsec * 0.001 * 0.001);
}

//计算从SetPacketTimeout()到现在经历的时间，并返回；
//调用此函数之前必须先调用SetPacketTimeout()
double PortHandler::GetTimeSinceStart()
{
    double _time;

    _time = GetCurrentTime() - packet_start_time_;
    if(_time < 0.0)
        packet_start_time_ = GetCurrentTime();

    return _time;
}

//把波特率的数字形式转化为BXXXXX的波特率形式
int PortHandler::GetCFlagBaud(int baudrate)
{
    switch(baudrate)
    {
    case 1200:
    	return B1200;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}

//设置偏置等待时间
void PortHandler::setBiasWaitTime(float _waitTm)
{
    if(_waitTm <= 5.0f)
        biasWaitTime = 5.0f;
    else if(_waitTm >= 50.0f)
        biasWaitTime = 50.0f;
    else
        biasWaitTime = _waitTm;
}
