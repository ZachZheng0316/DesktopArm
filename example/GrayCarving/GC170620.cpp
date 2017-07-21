#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include "Dmath.h"
#include "RobotisDef.h"
#include "XQtorH7.h"
#include "DeltaAlgorithm.h"
#include "DataParser.h"
#include "Hardware.h"

using namespace std;
using namespace ROBOTIS;

/***************************
DataPreUpper and DataPreDown
表示数据的精细程度：1代表数据
最精细；50代表数据最粗糙
****************************/
#define DataPreUpper    (50)
#define DataPreDown     (1)
#define LASER           (16)    //激光头端口号
#define LASER_OPEN      ('0')   //打开激光头
#define LASER_CLOSE     ('1')   //关闭激光头
#define RANGK           (4096.0)  //多级刻度范围
#define RANGANGLE       (360.0)   //角度范围
#define MAXSPEED        (360.0)   //舵机角速度360度/s
#define ACCUNIT         (4.5114787) //每个加速度刻度表示加速度度数

char SERVODEVICE[] = "/dev/ttyUSB0"; //舵机串口号
int SERVOBAUD = 115200;
double Height[3] = {-280.0, -290.0, -300.0};
int PosDataPre = 1;
/* 位置数据精细度，越小越精细
 * 范围[1-50]:[DataPreDown, DataPreUpper]
*/
char PicPath[] = "r0_5mm.txt"; //存储图片原始数据
char Tra_k[] = "trak.txt"; //存储舵机轨迹刻度的文件
char hardSizeFile[] = "hardSizeFile.txt";
int UpdatePosFre = 125.0; //位置更新频率125Hz
DeltaAlgorithm *pDeltaAlgo = NULL;
XQtorH7 *pXQH7 = NULL;
DataParser *pDParser = NULL;

//主要函数
bool initial_sys();
bool set_posture(float _xyz[]);
bool get_traK_file(char *picPath);
bool excute_traK_file();

// 功能函数
bool moveWithMaxSpeed(float srcXYZ[], float destXYZ[]);

int main()
{
    //系统初始化
    if(!initial_sys()) {
        printf("%s: %d: initial_sys failed\n", __FILE__, __LINE__);
        return 0;
    }

    //获取轨迹刻度文件
    /*
    if(!get_traK_file(PicPath)) {
        printf("%s: %d: function get_traK_file failed\n", __FILE__, __LINE__);
        return 0;
    }

    //延迟1s
    delay_ms(1000.0);
    printf("%s: %d: start to draw...\n", __FILE__, __LINE__);
    getchar();

    //计算绘制时间
    delay_start();

    //执行轨迹刻度文件
    if(!excute_traK_file()) {
        printf("%s: %d: excute_traK_file failed\n", __FILE__, __LINE__);
        return 0;
    }
    else
        printf("%s: %d: hello\n", __FILE__, __LINE__);

    //计算结束时间
    double delayTm = delay_end();
    printf("%s: %d: the delayTm(%lf)\n", __FILE__, __LINE__, delayTm);
    */


    moveWithMaxSpeed(srcXYZ, destXYZ1);
    delay_ms(1000.0);
    printf("%s: %d: start to draw...\n", __FILE__, __LINE__);
    getchar();
    while(true) {
        moveWithMaxSpeed(destXYZ1, destXYZ2);
        moveWithMaxSpeed(destXYZ2, destXYZ3);
        moveWithMaxSpeed(destXYZ3, destXYZ1);
        fprintf(fp, "%d\n", ++i);
        fflush(fp);
    }

    //关闭激光头
    Hardware_close_port();
    fclose(fp);

    return 1;
}

//主要函数
bool initial_sys()
{
    //延迟100ms
    delay_ms(100.0);

    //申请H7控制类
    pXQH7 = new XQtorH7(SERVODEVICE, SERVOBAUD, 3);
    if(!pXQH7->SuccessOpenDevice()) {
        printf("%s: %d: XQtorH7 Open Device failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: XQtorH7 Open Device Success\n", __FILE__, __LINE__);

    //申请Delta算法类
    pDeltaAlgo = new DeltaAlgorithm();
    if(!pDeltaAlgo->read_hardwareData(hardSizeFile)) {
        printf("%s: %d: read hardware size failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: read hardware size success\n", __FILE__, __LINE__);

    //申请数据解析类
    pDParser = new DataParser(0x20, Height, (void*)pDeltaAlgo);
    printf("%s: %d: apply for DataParser success\n", __FILE__, __LINE__);
    //设置舵机属性
    /*0、设置舵机Enable为失能，addr:24
     *1、设置安全保护状态为0,addr:29
     *2、设置力矩Enable使能, addr:24
     *3、设置PID：[64,32,30]
     *4、设置加速度为250,    addr:73
     *5、设置速度为1023,     addr:32
     *6、设置目标位置,       addr:30
    */
    bool result1, result2, result3;
    result1 = pXQH7->set_one_servo_bytes(1, 3, 24, 0x00);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 24, 0x00);
    result3 = pXQH7->set_one_servo_bytes(1, 1, 24, 0x00);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) Enable statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) Unenable \n", __FILE__, __LINE__);
    /*
    result1 = pXQH7->set_one_servo_bytes(1, 1, 29, 0x00);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 29, 0x00);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 29, 0x00);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) protect statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) protect statue\n", __FILE__, __LINE__);
    */
    result1 = pXQH7->set_one_servo_bytes(1, 1, 24, 0x01);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 24, 0x01);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 24, 0x01);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) Enable statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) Enable\n", __FILE__, __LINE__);
    result1 = pXQH7->set_one_servo_bytes(1, 1, 28, 64);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 28, 64);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 28, 64);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) P_PID failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) PID_P success\n", __FILE__, __LINE__);
    result1 = pXQH7->set_one_servo_bytes(1, 1, 27, 12);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 27, 12);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 27, 12);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) I_PID failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) PID_I success\n", __FILE__, __LINE__);
    result1 = pXQH7->set_one_servo_bytes(1, 1, 26, 10);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 26, 10);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 26, 10);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) P_PID failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) PID_D success\n", __FILE__, __LINE__);
    result1 = pXQH7->set_one_servo_bytes(1, 1, 73, 50);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 73, 50);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 73, 50);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) Acc statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) Acc success\n", __FILE__, __LINE__);
    result1 = pXQH7->set_one_servo_bytes(2, 1, 32, 1023);
    result2 = pXQH7->set_one_servo_bytes(2, 2, 32, 1023);
    result3 = pXQH7->set_one_servo_bytes(2, 3, 32, 1023);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) speedK statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) speedK success\n", __FILE__, __LINE__);

    //打开激光设备端口
    if(Hardware_open_port()) {
        //关闭激光头
        Hardware_write_gpio(LASER, LASER_CLOSE);
    }
    else {
        printf("%s: %d: open gpio port failed\n", __FILE__, __LINE__);
        return 0;
    }

    //设置初始位置
    float _xyz[3] = {0.0, 0.0, -330.0};
    _xyz[2] = Height[2];
    if(!set_posture(_xyz)) {
        printf("%s: %d: set posture failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set posture success\n", __FILE__, __LINE__);

    return true;
}

bool set_posture(float _xyz[])
{
    int goalK[3] = {0, 0, 0};
    int id[3] = {1, 2, 3}, result;
    pDParser->trans_coor_to_k(_xyz, goalK);
    printf("%s: %d: _xyz(%f, %f, %f) goalK(%d, %d, %d)\n", __FILE__, __LINE__, _xyz[0], _xyz[1], _xyz[2], goalK[0], goalK[1], goalK[2]);

    result = pXQH7->sync_set_many_servo(2, id, 30, goalK);
    if(result) {
        printf("%s: %d: set id(1,2,3) xyz(%f, %f, %f) goalK(%d, %d, %d) failed\n", __FILE__, __LINE__, _xyz[0], _xyz[1], _xyz[2], goalK[0], goalK[1], goalK[2]);
        return false;
    }
    else
        printf("%s: %d: set id(1, 2, 3) xyz(%f, %f, %f) goalK(%d, %d, %d)\n", __FILE__, __LINE__, _xyz[0], _xyz[1], _xyz[2], goalK[0], goalK[1], goalK[2]);

    return true;
}

bool get_traK_file(char *picPath)
{
    return pDParser->trans_ink_to_goalK(picPath);
}

bool excute_traK_file()
{
    FILE *fp = NULL;
    char flag;
    int posK[3], id[3] = {1, 2, 3}, exten[3] = {5, 5, 5};
    int result1, result2;
    int _stepIndex=PosDataPre;
    bool laserOpen = false, laserClose = true;

    fp = fopen(Tra_k, "r");
    if(!fp) {
        printf("%s: %d: open Tra_k failed\n", __FILE__, __LINE__);
        return false;
    }
    while(!feof(fp)) {
        if(EOF == fscanf(fp, "%c (%d %d %d)\n", &flag, &posK[0], &posK[1], &posK[2])) {
            printf("%s: %d: read tra_k failed\n", __FILE__, __LINE__);
            return false;
        }
        if('m' == flag) {
            if(laserOpen) {
                laserOpen = false;
                laserClose = true;
                Hardware_write_gpio(LASER, LASER_CLOSE);
            }

            //如果遇到'm'标志坐标，设置目标位置，并等待运动停止
            result1 = pXQH7->sync_set_many_servo(2, id, 30, posK);
            printf("%s: %d: set goalK(%d, %d, %d)\n", __FILE__, __LINE__, posK[0], posK[1], posK[2]);
            result2 = pXQH7->wait_for_many_servo_exten(exten);
            if(result1 || result2) {
                printf("%s: %d: error result1(%d) or result2(%d)\n", __FILE__, __LINE__, result1, result2);
                return false;
            }
            _stepIndex = 1;
            delay_ms(1000.0); //延迟50ms
        }
        else{
            if(laserClose) {
                laserClose = false;
                laserOpen = true;
                Hardware_write_gpio(LASER, LASER_OPEN);
            }

            //如果遇到'l'标志的数据，通过_stepIndex
            //控制运动的精度和速度
            //if(_stepIndex >= PosDataPre) {
                result1 = pXQH7->sync_set_many_servo(2, id, 30, posK);
                //result2 = pXQH7->wait_for_many_servo_exten(exten);
                if(result1 /*|| result2*/) {
                    printf("%s: %d: error result1(%d)\n", __FILE__, __LINE__, result1);
                    return false;
                }
                //printf("%s: %d: posK(%d, %d, %d)\n", __FILE__, __LINE__, posK[0], posK[1], posK[2]);
                //_stepIndex = 1;
            //}
            //else
            //    _stepIndex += 1;
            delay_ms((double)(1000.0 / UpdatePosFre));
        }
        /* 延迟1ms
         * 如果发送的指令无应答包，指令之间应相差100us
         * 此条指令无应答包，指令之间相差1000us
        */
        //delay_ms(1.0);
    }

    fclose(fp);

    return true;
}

/****************************************************
函数意义：
    机械臂以最快的速度从坐标srcXYZ[],运动到destXYZ[]
参数意义：
    srcXYZ:起始坐标
    destXYZ:终止坐标
返回值：
    执行成功，返回true;
    执行失败，返回false;
*****************************************************/
bool moveWithMaxSpeed(float srcXYZ[], float destXYZ[])
{
    int srcGoalK[3], destGoalK[3], exten[3] = {10, 10, 10}, exten1[3] = {35, 35, 35};
    int diffK[3];
    double angleDiff[3], minTm;

    float srcXYZ[3] = {0.0, 0.0, -280.0};
    float destXYZ1[3] = {0.0, 50.0, -300.0};
    float destXYZ2[3] = {-50.0, -50.0, -300.0};
    float destXYZ3[3] = {50.0, -50.0, -300.0};
    FILE *fp = NULL;
    int i = 0;
    fp = fopen("num.txt", "w");

    //从初始位置运动到起始位置


    //转换坐标到目标刻度
    pDParser->trans_coor_to_k(srcXYZ, srcGoalK);
    pDParser->trans_coor_to_k(destXYZ, destGoalK);

    //运动到起始作坐标
    set_posture(srcXYZ);
    pXQH7->wait_for_many_servo_exten(exten);

    //运动到目标位置
    set_posture(destXYZ);
    pXQH7->wait_for_many_servo_exten(exten1);
    Hardware_write_gpio(LASER, LASER_OPEN);

    //下降
    destXYZ[2] -= 20.0;
    set_posture(destXYZ);
    pXQH7->wait_for_many_servo_exten(exten);

    //上升
    destXYZ[2] += 20.0;
    set_posture(destXYZ);
    pXQH7->wait_for_many_servo_exten(exten1);
    Hardware_write_gpio(LASER, LASER_CLOSE);

    //计算实际运动时间

    //返回运算结果
    return true;
}
