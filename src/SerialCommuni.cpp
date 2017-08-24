#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "RobotisDef.h"
#include "SerialCommuni.h"

using namespace ROBOTIS;

#define SerialBaseFre (115200) //base frequance

SerialCommuni::SerialCommuni(char _deviceName[], UINT32_T _baudrate)
{
    //端口初始化
    serial_fd = -1;

    //初始化设备名称
    deviceName = new char[30];
    memset(deviceName, 0, 30);
    sprintf(deviceName,"%s", _deviceName);

    //初始化波特率
    baudrate = _baudrate;
}

SerialCommuni::~SerialCommuni()
{
    delete[] deviceName;
    deviceName = NULL;
}

//计算波特率
//函数意义：
//  把波特率转化为Bxxxx形式
//参数的意义：
//  baudNum:表示波特率
//返回值：
//  返回波特率
UINT32_T SerialCommuni::getCFlagBaud(UINT32_T baudNum)
{
    switch (baudNum) {
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
        default:
            return 0;
    }
}

//打开通信端口
//函数意义：
//  已知端口号和波特率倍率，打开端口
//参数：
//返回值：
//  如果打开成功，返回true
//  如果带开失败，返回false
bool SerialCommuni::openCommuniPort()
{
    struct termios newtio;
    struct serial_struct serinfo;
    int baud, Bbaud;

    memset(&newtio, 0, sizeof(newtio));
    closeCommuniPort();

    //compute SerialBaudRate
    Bbaud = (int)getCFlagBaud(baudrate);
    if(!Bbaud) {
        perror("SerialCommuni::openCommuniPort:no Bbaud");
        return false;
    }

    //open serial device
    if((serial_fd = open(deviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
        perror("SerialCommuni::openCommuniPort:open failed\n");
        return false;
    }

    //set serial info
    newtio.c_cflag = Bbaud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(serial_fd, TCIFLUSH); //refresh the received data, but don't read
    tcsetattr(serial_fd, TCSANOW, &newtio); //set the serial parameters and take effect immediately

    //get the tty line information
    if(ioctl(serial_fd, TIOCGSERIAL, &serinfo) < 0) {
        printf("SerialCommuni::openCommuniPort:Cannot get serial info\n");
        return false;
    }
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    //set the tty line information
    if(ioctl(serial_fd, TIOCSSERIAL, &serinfo) < 0) {
        printf("SerialCommuni::openCommuniPort:Cannot set serial info\n");
        return false;
    }

    //close serial port
    closeCommuniPort();

    //open the serial again
    if((serial_fd = open(deviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
        printf("SerialCommuni::openCommuniPort:device open error %s again\n", deviceName);
        return false;
    }

    //Set a serial port propertie
    newtio.c_cflag = Bbaud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(serial_fd, TCIOFLUSH);
    tcsetattr(serial_fd, TCSANOW, &newtio);

    return true;
}

//函数意义：
//  已知数据包指针和数据包最大长度，接收数据
//参数：
//  pPacket:数据包指针
//  packetLen:数据包长度
//返回值：
//  返回实际接收到的数据长度
UINT32_T SerialCommuni::receiveMessage(UINT8_T *pPacket, UINT32_T packetLen)
{
    int readNum = 0;
    memset(pPacket, 0, packetLen);
    readNum = read(serial_fd, pPacket, packetLen);
    return readNum;
}

//函数意义：
//  已知数据包指针和数据包长度，发送数据包
//参数意义：
//  pPacket:存储数据的数据包指针
//  packetLen:数据包中数据的长度
//返回值：
//  返回实际发送的数据长度
UINT32_T SerialCommuni::sendMessage(UINT8_T *pPacket, UINT32_T packetLen)
{
    tcflush(serial_fd, TCOFLUSH);
    return write(serial_fd, pPacket, packetLen);
}

void SerialCommuni::clearCommuniPort()
{
    //清空管道
    tcflush(serial_fd, TCIOFLUSH);
}

//function:
//  close the communication port
//parameters:
//  none
//the returned value
//  none
void SerialCommuni::closeCommuniPort()
{
    if(serial_fd != -1)
        close(serial_fd);
    serial_fd = -1;
}
