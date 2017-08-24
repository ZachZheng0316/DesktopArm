#include "SerialCommuni.h"
#include "RealTimePro.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace ROBOTIS;

#define PROLENGTH   (12)

RealTimePro::RealTimePro(char *deviceName, int baudNum)
{
    x = 0; y = 0;
    Num = 0;
    //申请存储协议数据的空间
    pPacket = new UINT8_T[PROLENGTH+1];
    memset(pPacket, 0, PROLENGTH+1);
    //申请串口通信对象
    pCommuni = new SerialCommuni(deviceName, baudNum);
}

RealTimePro::~RealTimePro()
{
    delete[] pPacket;
    delete[] pCommuni;
    pPacket = NULL;
    pCommuni = NULL;
}

bool RealTimePro::openProSuccess()
{
    return pCommuni->openCommuniPort();
}

//成功读取协议数据
bool RealTimePro::readProSuccess()
{
    UINT8_T _checksum = 0;
    UINT32_T _real_length = 0;
    UINT32_T _data_length = PROLENGTH;
    bool ret;

    //清空管道
    pCommuni->clearCommuniPort();
    while (true) {
        _real_length += pCommuni->receiveMessage((UINT8_T*)&pPacket[_real_length], _data_length - _real_length);

        //如果实际数据长度>=给定的数据长度
        if(_real_length >= _data_length) {
            //找到数据头
            UINT8_T _idx = 0;
            for(_idx = 0; _idx < (_real_length - 1); _idx++) {
                if((pPacket[_idx] == 0xFF) && (pPacket[_idx+1] == 0xFF))
                    break;
            }

            //如果数据头在数据包的开头位置
            if(0 == _idx) {
                //检查checksum是否正确
                for(int i = 2; i < PROLENGTH - 1; i++)
                    _checksum += pPacket[i];
                _checksum = ~_checksum;
                if(_checksum == pPacket[PROLENGTH-1]) {
                    ret = true;
                    break;
                }
                else {
                    ret = false;
                    break;
                }
            }
            else { //数据头不在数据包的开头位置
                //移除非必须数据
                for(UINT8_T _s = 0; _s < _real_length - _idx; _s++)
                    pPacket[_s] = pPacket[_idx + _s];
                _real_length -= _idx;
            }
        }
        else { //实际数据长度 <= 给定的数据长度
            //如果数据超时，则接收失败
            //否则，继续接受
            //printf("\rreceiving data ...");
        }
    }
    //显示接受到数据
    /*
    printf("\n");
    if(ret)
        printf("Right :");
    else
        printf("Wrong :");
    for(int _i = 0; _i < PROLENGTH; _i++)
        printf("%x ", pPacket[_i]);

    printf("Num:%d\n", ++Num);
    fflush(stdout);
    */
    return ret;
}

//成功分析出协议数据
bool RealTimePro::anaProSuccess(UINT8_T* packet)
{
    UINT8_T _flag;

    //获取运动标志
    _flag = packet[2];
    if(_flag == 0x01)
        flag = 'l';
    if(_flag == 0x02)
        flag = 's';
    if((_flag != 0x01) && (_flag != 0x02))
        return false;

    //获取x,y坐标
    for(int i = 0; i < 4; i++) {
        x += ((int)packet[3+i])*pow(256, 3-i);
        y += ((int)packet[7+i])*pow(256, 3-i);
    }
    //printf("%c %d %d\n", flag, x, y);
    if(x >= 656)
        x = 656;
    if(y >= 656)
        y = 656;

    return true;
}

//关闭协议端口
void RealTimePro::closeProSucess()
{
    pCommuni->closeCommuniPort();
}

//保存指令包
void RealTimePro::savePacket(FILE* fp)
{
    if(anaProSuccess(pPacket)) {
        fprintf(fp, "%c,%d,%d\n", flag, x, y);
        fflush(fp);
    }
    x = 0; y = 0;
    /*
    fwrite(pPacket, sizeof(UINT8_T), 12, fp);
    fflush(fp);
    */
}

int RealTimePro::returnX()
{
    int _x = x;
    if(_x > 656)
        _x = 656;
    x = 0;
    return _x;
}

int RealTimePro::returnY()
{
    int _y = y;
    if(_y > 656)
        _y = 656;
    y = 0;
    return _y;
}

char RealTimePro::returnFlag()
{
    return flag;
}
