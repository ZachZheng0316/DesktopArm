#include "PortHandler.h"
#include "LeapControlPro.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace ROBOTIS;

#define PROLENGTH3D   (16)

LeapControlPro::LeapControlPro(int deviceIndex, int baudNum)
{
    xyz3D[0] = 0;
    xyz3D[1] = 0;
    xyz3D[2] = 0;

    bengOpen = false;

    //申请存储协议数据的空间
    pPacket = new UINT8_T[PROLENGTH3D+1];
    memset(pPacket, 0, PROLENGTH3D+1);

    //申请串口通信对象
    pPort = new PortHandler(deviceIndex, baudNum);
}

LeapControlPro::~LeapControlPro()
{
    delete[] pPacket;
    delete[] pPort;
    pPacket = NULL;
    pPort = NULL;
}

bool LeapControlPro::openProSuccess()
{
    return pPort->OpenPort();
}

//成功读取协议数据
bool LeapControlPro::readProSuccess()
{
    UINT8_T _checksum = 0;
    UINT32_T _real_length = 0;
    UINT32_T _data_length = PROLENGTH3D;
    bool ret;

    //清空管道
    pPort->ClearPort();

    while (true) {
        _real_length += pPort->ReadPort((UINT8_T*)&pPacket[_real_length], _data_length - _real_length);

        /*显示接收到的数据*/
        for(int i = 0; i < _real_length; i++)
            printf("%x ", pPacket[i]);

        /*如果实际数据长度>=给定的数据长度*/
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
                for(int i = 2; i < PROLENGTH3D - 1; i++)
                    _checksum += pPacket[i];
                _checksum = ~_checksum;
                if(_checksum == pPacket[PROLENGTH3D-1]) {
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

    }

    return ret;
}

//成功分析出协议数据
void LeapControlPro::anaProSuccess()
{
    char valueX[4], valueY[4], valueZ[4];

    xyz3D[0] = 0x00000000;
    xyz3D[1] = 0x00000000;
    xyz3D[2] = 0x00000000;

    if(0x01 == pPacket[2])
        bengOpen = 1;
    if(0x03 == pPacket[2])
        bengOpen = 3;
    if(0x02 == pPacket[2])
        bengOpen = 2;
    if(0x04 == pPacket[2])
        bengOpen = 4;

    //获取xyz坐标
    for(int i = 0; i < 4; i++) {
        valueX[i] = pPacket[3+i];
        valueY[i] = pPacket[7+i];
        valueZ[i] = pPacket[11+i];
    }

    /*显示valueXYZ*/
    printf("%s: %d\n", __FILE__, __LINE__);
    for(int i = 0; i < 4; i++)
        printf("%x ", valueX[i]);
    printf("\n");
    for(int i = 0; i < 4; i++)
        printf("%x ", valueY[i]);
    printf("\n");
    for(int i = 0; i < 4; i++)
        printf("%x ", valueZ[i]);
    printf("\n");fflush(stdout);

    //计算xyz的坐标
    for(int j = 0; j < 4; j++) {
        xyz3D[0] = (int)((xyz3D[0] << 8) + (int)valueX[j]);
        xyz3D[1] = (int)((xyz3D[1] << 8) + (int)valueY[j]);
        xyz3D[2] = (int)((xyz3D[2] << 8) + (int)valueZ[j]);
    }

    printf("%s: %d: XYZ(%d, %d, %d)\n", __FILE__, __LINE__, xyz3D[0], xyz3D[1], xyz3D[2]);

    /*空间映射*/
    /*X坐标*/
    if(xyz3D[0] <= -400)
        xyz3D[0] = -400;
    if(xyz3D[0] >= 400)
        xyz3D[0] = 400;
    /*Y坐标*/
    if (xyz3D[1] <= 0)
        xyz3D[1] = 0;
    if(xyz3D[1] >= 400)
        xyz3D[1] = 400;
    /*Z坐标*/
    if(xyz3D[2] <= -400)
        xyz3D[2] = -400;
    if(xyz3D[2] >= 400)
        xyz3D[2] = 400;
}

//关闭协议端口
void LeapControlPro::closeProSucess()
{
    pPort->ClosePort();
}

//保存指令包
void LeapControlPro::savePacket(FILE* fp)
{
    anaProSuccess();
    fprintf(fp, "%d,%lf,%lf,%lf\n", bengOpen, (double)xyz3D[0], (double)xyz3D[1], (double)xyz3D[2]);
    fflush(fp);
    printf("%s: %d: (X,Y,Z)(%d, %d, %d)-->(-X, Z, Y)\n", __FILE__, __LINE__, xyz3D[0], xyz3D[1], xyz3D[2]);

    /*设置数据为0*/
    for(int i = 0; i < 3; i++)
        xyz3D[i] = 0;
}

void LeapControlPro::returnXYZ3D(int value[], char _bengOpen)
{
    _bengOpen = pPacket[2];

    for(int i = 0; i < 3; i++) {
        value[i] = xyz3D[i];
        xyz3D[i] = 0;
    }
}
