#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include "MoveControl.h"
#include "RobotisDef.h"
#include "RealTimePro.h"
#include "SerialCommuni.h"
#include "ScaraAlgorithm.h"
#include "Dmath.h"

using namespace ROBOTIS;

#define PATH         ("./PATH.dat")
#define PXIEL_PER_MM (656.00/200.00)  //像素/长度
#define EFFECTIVEGAP (200.00/656.00)  //长度/像素

long saveLabel;     //指向当前存储数据的位置
long excuteLabel;   //指向执行数据的位置
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
FILE* fpPath = NULL;
RealTimePro* pPro = NULL;
MoveControl* pMove = NULL;
double F = 50.00;         //位置更新频率50Hz
double SpeedRatu = 1.0;   //速度倍率
double ReStartRatu = 100.0;//重新起始倍率
UINT8_T Packet[13];       //存储信息的全局变量

//原坐标转化为要开始工作的坐标
void coorTrans(double sourX, double sourY, double& destX, double& destY);
//设置笔的高度
//heightFlag = 1时为抬起
//heightFlag = 0时为下降
void set_pen_height(int heightFlag);

void* saveDataThread(void* arg);    //存储数据线程
void* excuteDataThread(void* arg);  //执行数据线程

int main(void)
{
    int ret1, ret2;
    pthread_t t_save, t_excute;
    pMove = new MoveControl(0x30);

    //以读写的方式打开文件
    fpPath = fopen(PATH, "w+");
    if(!fpPath) {
        perror("open file failed");
        exit(1);
    }
    saveLabel = ftell(fpPath);
    excuteLabel = ftell(fpPath);

    //申请协议对象
    pPro = new RealTimePro("/dev/ttyS5", 115200);

    //打开协议对象
    if(!pPro->openProSuccess()){
        printf("%s: %d: pPro->openProSuccess() failed\n", __FILE__, __LINE__);
        exit(1);
    }

    //动作初始化
    if(!pMove->moveInitial(0, 1000000)) {
        printf("error: %s: %d: pMove->moveInitial failed\n", __FILE__, __LINE__);
        exit(1);
    }

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

//原坐标转化为要开始工作的坐标
void coorTrans(double sourX, double sourY, double& destX, double& destY)
{
    destX = 80.0 - sourX;
    destY = sourY + 145.0;
}

//设置笔的高度
//heightFlag = 1时为抬起
//heightFlag = 0时为下降
void set_pen_height(int heightFlag)
{
    delay_ms(1);
    if(1 == heightFlag) {
        pMove->set_pen_height(5, Goal_Position, 11279); //[position speed] --> 0x2c0f
        delay_ms(1.0);
        pMove->set_pen_height(5, Goal_Position, 11279);
        delay_ms(1.0);
        pMove->set_pen_height(5, Goal_Position, 11279);
    }
    else {
        pMove->set_pen_height(5, Goal_Position, 44815); //x0AF0f
        delay_ms(1.0);
        pMove->set_pen_height(5, Goal_Position, 44815);
        delay_ms(1.0);
        pMove->set_pen_height(5, Goal_Position, 44815);
    }
}

//存储数据的线程
void* saveDataThread(void* arg)
{
    int x, y;
    char flag;
    double prexy[2], nextxy[2], diff = 0.0;
    double effectiveSpace = 0.0;

    saveLabel = 0;
    prexy[0] = 0.00; prexy[1] = 0.00;

    printf("Entry saveDataThread\n");
    while(true) {
        //读取数据
        if(pPro->readProSuccess()) {
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
    int num = 0, x, y, result, pathNum = 0;
    int id[2] = {1, 2}, value[2] = {1023, 1023};
    double goalXYZ[3];
    double prexy[2], nextxy[2], diff;
    char flag;
    bool ReStartPath = true;

    goalXYZ[0] = 0.0; goalXYZ[1] = 0.0; goalXYZ[2] = -300.00;
    prexy[0] = 0.00; prexy[1] = 0.00; diff = 0.00;

    printf("Entry excuteDataThread\n");
    set_pen_height(1); delay_ms(1000); //高度初始化

    //设置加速度属性
    value[0] = 254; value[1] = 254;
    pMove->setServoProperty(2, id, Goal_Acceleration, value);
    //设置速度属性
    value[0] = 1023; value[1] = 1023;
    pMove->setServoProperty(2, id, Moving_Speed, value);
    //设置初始位置
    x = 328; y = 0;
    prexy[0] = (double)x / PXIEL_PER_MM;
    prexy[1] = (double)y / PXIEL_PER_MM;
    coorTrans(prexy[0], prexy[1], goalXYZ[0], goalXYZ[1]);
    pMove->pointToPointMove(goalXYZ);
    pMove->waitMoveStop();

    while(true) {
        delay_start();
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
            result = fscanf(fpPath, "%c,%d,%d\n", &flag, &x, &y);
            //更新定位
            excuteLabel = ftell(fpPath);
            //printf("excuteLabel:%ld\n", excuteLabel);
            pthread_mutex_unlock(&mutex); //解锁

            //解析信息
            if(3 == result/*pPro->anaProSuccess(Packet)*/) {
                //x = pPro->returnX();
                //y = pPro->returnY();
                //flag = pPro->returnFlag();

                //计算目标位置
                nextxy[0] = (double)x / PXIEL_PER_MM;
                nextxy[1] = (double)y / PXIEL_PER_MM;
                //printf("%lf %lf\n", nextxy[0], nextxy[1]);
                diff += pow(fabs(nextxy[0] - prexy[0]), 2);
                diff += pow(fabs(nextxy[1] - prexy[1]), 2);
                diff = sqrt(diff);

                if('l' == flag) {
                    //只执行有效距离
                    if(diff > EFFECTIVEGAP * SpeedRatu) {
                        //把触屏上的坐标转化为scara工作范围内的坐标
                        coorTrans(nextxy[0], nextxy[1], goalXYZ[0], goalXYZ[1]);
                        //printf("goalXYZ(%lf, %lf)\n", goalXYZ[0], goalXYZ[1]);
                        //开始绘制一段新路径
                        /*
                        if(ReStartPath == true) {
                            //抬起笔
                            //set_pen_height(1); delay_ms(100.0);
                            pMove->pointToPointMove(goalXYZ, 1);
                            pMove->waitMoveStop();

                            //设置速度属性
                            value[0] = 1023; value[1] = 1023;
                            pMove->setServoProperty(2, id, Moving_Speed, value);

                            //降下笔
                            //set_pen_height(0);delay_ms(2000.0);
                            //printf("%s: %d: XY(%lf, %lf) ReStartPath(%d) pathNum: %d\n", __FILE__, __LINE__, goalXYZ[0], goalXYZ[1], ReStartPath, ++pathNum);

                            ReStartPath = false;
                        }
                        else{
                            //继续运动
                            pMove->pointToPointMove(goalXYZ);
                            //延迟时间
                            //delay_ms(1000/F);
                        }

                        //更新数据
                        prexy[0] = nextxy[0];
                        prexy[1] = nextxy[1];
                        */
                        pMove->pointToPointMove(goalXYZ, 1);
                        pMove->waitMoveStop();

                        //运动到初始点
                        x = 328; y = 0;
                        prexy[0] = (double)x / PXIEL_PER_MM;
                        prexy[1] = (double)y / PXIEL_PER_MM;
                        coorTrans(prexy[0], prexy[1], goalXYZ[0], goalXYZ[1]);
                        pMove->pointToPointMove(goalXYZ, 1);
                        pMove->waitMoveStop();
                    }
                }
                else {
                    //抬起笔
                    /*
                    pMove->waitMoveStop();
                    set_pen_height(1);
                    delay_ms(500.0);
                    */

                    //等待运动停止
                    //pMove->waitMoveStop();
                    //restart move to center
                    /*
                    x = 328; y = 328;
                    prexy[0] = (double)x / PXIEL_PER_MM;
                    prexy[1] = (double)y / PXIEL_PER_MM;
                    coorTrans(prexy[0], prexy[1], goalXYZ[0], goalXYZ[1]);
                    pMove->pointToPointMove(goalXYZ);
                    */
                    //pMove->waitMoveStop();

                    //更新数据
                    /*
                    prexy[0] = nextxy[0];
                    prexy[1] = nextxy[1];
                    */
                    ReStartPath = true;
                }
            }
            //printf("%s: %d: delay %lf ms\n", __FILE__, __LINE__, delay_end());
        }
        else{
            pthread_mutex_unlock(&mutex);
            //延迟一段时间
        }
    }
}
