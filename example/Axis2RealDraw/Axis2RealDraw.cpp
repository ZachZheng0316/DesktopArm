#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include "MoveControl.h"
#include "RobotisDef.h"
#include "LeapControlPro.h"
#include "SerialCommuni.h"
#include "ScaraAlgorithm.h"
#include "Dmath.h"
#include "Hardware.h"

using namespace ROBOTIS;

#define PATH         ("./PATH.dat")
#define EFFECTIVEGAP (1.0)

long saveLabel;     //指向当前存储数据的位置
long excuteLabel;   //指向执行数据的位置
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
FILE* fpPath = NULL;
LeapControlPro* pPro = NULL;
MoveControl* pMove = NULL;
double iniPos[3] = {0.0, 202.0, 20.0};   //初始位置
double F = 500.00;         //位置更新频率
double SpeedRatu = 0.5;   //速度倍率
double ReStartRatu = 100.0;//重新起始倍率
UINT8_T Packet[13];       //存储信息的全局变量

/*原坐标转化为要开始工作的坐标*/
void coorTrans(double sourxyz[], double destxyz[]);

void moveToInitialPos(); /*运动到起始位置*/

void* saveDataThread(void* arg);    //存储数据线程
void* excuteDataThread(void* arg);  //执行数据线程

int main(void)
{
    int ret1, ret2;
    pthread_t t_save, t_excute;

    pMove = new MoveControl(0x12);

    //以读写的方式打开文件
    fpPath = fopen(PATH, "w+");
    if(!fpPath) {
        printf("%s: %d: open file failed\n", __FILE__, __LINE__);
        exit(1);
    }
    saveLabel = ftell(fpPath);
    excuteLabel = ftell(fpPath);

    //申请协议对象
    pPro = new LeapControlPro(5, 115200);

    //打开协议对象
    if(!pPro->openProSuccess()){
        printf("%s: %d: pPro->openProSuccess() failed\n", __FILE__, __LINE__);
        exit(1);
    }
    else
        printf("%s: %d: pPro->openProSuccess() success\n", __FILE__, __LINE__);

    //动作初始化
    if(!pMove->moveInitial(0, 9600)) {
        printf("error: %s: %d: pMove->moveInitial failed\n", __FILE__, __LINE__);
        exit(1);
    }
    else
        printf("%s: %d: pMove->moveInitial success\n", __FILE__, __LINE__);

    moveToInitialPos(); /*运动到初始位置*/
    open_port();        /*打开气泵*/
    close_beng();       /*关闭气泵*/

    //创建线程
    ret1 = pthread_create(&t_save, NULL, saveDataThread, NULL);
    ret2 = pthread_create(&t_excute, NULL, excuteDataThread, NULL);
    if((0 != ret1) || (0 != ret2)) {
        printf("Create thread failed:ret1");
        exit(1);
    }

    //pthread_join(t_excute, NULL);
    pthread_join(t_save, NULL);
    pthread_mutex_destroy(&mutex);

    return 0;
}

/*运动到起始位置*/
void moveToInitialPos()
{
    int externK[3] = {5, 5, 5};

    pMove->pointToPointMove(iniPos, 1.50);  /*以1.50s的时间运动到目标位置*/
    pMove->waitMoveStopWithExtern(externK); /*等待运动停止*/
}

/*
原坐标转化为要开始工作的坐标
X[-400, +400] --> X[+200, -200]
Y[0, 400] --> Z[0, 200]
Z[-400, 400] --> Y[-150, 250]
*/
void coorTrans(double sourxyz[], double destxyz[])
{
    destxyz[0]=sourxyz[0] * (-0.5);               //x的坐标
    destxyz[1]=(sourxyz[2]+100) * 0.5 + iniPos[1];//y的坐标
    destxyz[2]=sourxyz[1]*0.5 - 35.0;             //z坐标
}

//存储数据的线程
void* saveDataThread(void* arg)
{
    saveLabel = 0;

    printf("Entry saveDataThread\n");
    while(true) {
        //读取数据
        if(pPro->readProSuccess()) {
            printf("\n"); fflush(stdout);
            //printf("main::saveDataThread:pPro->readProSuccess success\n");
            //把数据写入文件
            //写入文件中
            pthread_mutex_lock(&mutex);  //加锁
            //定位文件写入位置
            fseek(fpPath, saveLabel, SEEK_SET);
            //写入数据
            pPro->savePacket(fpPath);
            //更新文件位置标签
            saveLabel = ftell(fpPath);
            //printf("saveLabel:%ld\n", saveLabel);
            pthread_mutex_unlock(&mutex);//解锁
        }
        //else
        //    printf("main::saveDataThread:pPro->readProSuccess failed\n");
    }
}

//读取数据的线程
void* excuteDataThread(void* arg)
{
    int num = 0, result, pathNum = 0;
    int id[3] = {1, 2, 3}, value[3] = {1023, 1023, 1023};
    double xyz[3], goalXYZ[3], prexyz[3], nextxyz[3], diff;
    bool isFirst = true, bengIsOpen = false;
    int instFlag = 0, preFlagStatue = 0;

    goalXYZ[0] = 0.0; goalXYZ[1] = 0.0; goalXYZ[2] = -300.00;
    prexyz[0] = 0.00; prexyz[1] = 0.00; prexyz[2] = 0.00;
    diff = 0.00;

    printf("Entry excuteDataThread\n");

    /*获取当前坐标*/
    prexyz[0] = iniPos[0];
    prexyz[1] = iniPos[1];
    prexyz[2] = iniPos[2];

    while(true) {
        diff = 0.0;
        memset(Packet, 0, 13);
        //提取信息
        pthread_mutex_lock(&mutex);   //加锁
        //比较文件位置
        if(saveLabel > excuteLabel) {
            //定位
            fseek(fpPath, excuteLabel, SEEK_SET);
            //读取数据
            //fread(Packet, sizeof(UINT8_T), 12, fpPath);
            result = fscanf(fpPath, "%d,%lf,%lf,%lf\n",&instFlag, &xyz[0], &xyz[1], &xyz[2]);
            excuteLabel = ftell(fpPath);
            //printf("excuteLabel:%ld\n", excuteLabel);
            pthread_mutex_unlock(&mutex); //解锁

            /*如果读取信息正确*/
            if(4 == result) {
                coorTrans(xyz, nextxyz);    /*把接收到的数据转换为目标数据*/

                diff += pow(fabs(nextxyz[0] - prexyz[0]), 2);
                diff += pow(fabs(nextxyz[1] - prexyz[1]), 2);
                diff += pow(fabs(nextxyz[2] - prexyz[2]), 2);
                diff = sqrt(diff);

                printf("%s: %d:type(%d) nextxyz(%lf, %lf, %lf) diff(%lf)\n", __FILE__, __LINE__,instFlag, nextxyz[0], nextxyz[1], nextxyz[2], diff);

                /*第一个目标点*/
                if(isFirst) {
                    if(2 == instFlag) {
                        open_beng();
                        bengIsOpen = true;
                    }
                    else if(3 == instFlag) {
                        close_beng();
                        bengIsOpen = false;
                    }
                    else if(4 == instFlag) {
                        bengIsOpen = false;
                        nextxyz[0] = iniPos[0];
                        nextxyz[1] = iniPos[1];
                        nextxyz[2] = iniPos[2];
                    }
                    else{}
                    pMove->pointToPointMove(nextxyz, 1.0);
                    pMove->waitMoveStop();
                    pMove->setServoWordProperty(3, id, Moving_Speed, value);/*设置速度属性*/
                    isFirst = false;
                    preFlagStatue = instFlag;
                }
                else{
                    if(preFlagStatue != instFlag) {
                        if(2 == instFlag) {
                            open_beng();
                            bengIsOpen = true;
                            pMove->pointToPointMove(nextxyz, 0.0);
                        }
                        else if(3 == instFlag) {
                            close_beng();
                            bengIsOpen = false;
                            pMove->pointToPointMove(nextxyz, 0.0);
                        }
                        else if((4 == instFlag) &&(!bengIsOpen)) {
                            nextxyz[0] = iniPos[0];
                            nextxyz[1] = iniPos[1];
                            nextxyz[2] = iniPos[2];
                            pMove->pointToPointMove(nextxyz, 1.0);
                            pMove->waitMoveStop();
                            pMove->setServoWordProperty(3, id, Moving_Speed, value);/*设置速度属性*/
                        }
                        else{}
                        preFlagStatue = instFlag;
                    }
                    else{ /*获得的指令与上一条指令相同*/
                        if((diff > EFFECTIVEGAP * SpeedUnit) &&(4 != instFlag)){
                            pMove->pointToPointMove(nextxyz, 0.0);
                        }
                    }

                    //更新坐标
                    prexyz[0] = nextxyz[0];
                    prexyz[1] = nextxyz[1];
                    prexyz[2] = nextxyz[2];

                    //延迟时间
                    delay_ms(1000.0/F);
                }
            }
        }
        else{
            pthread_mutex_unlock(&mutex);
            //延迟一段时间
        }
    }
}
