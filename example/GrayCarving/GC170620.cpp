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

using namespace std;
using namespace ROBOTIS;

/***************************
DataPreUpper and DataPreDown
表示数据的精细程度：1代表数据
最精细；50代表数据最粗糙
****************************/
#define DataPreUpper    (50)
#define DataPreDown     (1)

char SERVODEVICE[] = "/dev/ttyUSB0"; //舵机串口号
int SERVOBAUD = 115200;
double Height[3] = {-330.0, -340.0, -350.0};
int PosDataPre = 1;
/* 位置数据精细度，越小越精细
 * 范围[1-50]:[DataPreDown, DataPreUpper]
*/
char PicPath[] = "picture.txt"; //存储图片原始数据
char Tra_k[] = "trak.txt"; //存储舵机轨迹刻度的文件
char hardSizeFile[] = "hardSizeFile.txt";
int UpdatePosFre = 500; //位置更新频率500Hz
DeltaAlgorithm *pDeltaAlgo = NULL;
XQtorH7 *pXQH7 = NULL;
DataParser *pDParser = NULL;

//主要函数
bool initial_sys();
bool set_posture(float _xyz[]);
bool get_traK_file(char *picPath);
bool excute_traK_file();

int main()
{
    //系统初始化
    if(!initial_sys()) {
        printf("%s: %d: initial_sys failed\n", __FILE__, __LINE__);
        return 0;
    }

    //获取轨迹刻度文件
    if(!get_traK_file(PicPath)) {
        printf("%s: %d: function get_traK_file failed\n", __FILE__, __LINE__);
        return 0;
    }

    //延迟1s
    delay_ms(1000.0);

    //执行轨迹刻度文件
    if(!excute_traK_file()) {
        printf("%s: %d: excute_traK_file failed\n", __FILE__, __LINE__);
        return 0;
    }
    else
        printf("%s: %d: hello\n", __FILE__, __LINE__);

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
    result1 = pXQH7->set_one_servo_bytes(1, 1, 29, 0x00);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 29, 0x00);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 29, 0x00);
    if(result1 || result2 || result3) {
        printf("%s: %d: set id(1,2,3) protect statue failed\n", __FILE__, __LINE__);
        return false;
    }
    else
        printf("%s: %d: set id(1,2,3) protect statue\n", __FILE__, __LINE__);
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
    result1 = pXQH7->set_one_servo_bytes(1, 1, 73, 10);
    result2 = pXQH7->set_one_servo_bytes(1, 2, 73, 10);
    result3 = pXQH7->set_one_servo_bytes(1, 3, 73, 10);
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

    //设置初始位置
    float _xyz[3] = {0.0, 0.0, -330.0};
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
            //如果遇到'm'标志坐标，设置目标位置，并等待运动停止
            result1 = pXQH7->sync_set_many_servo(2, id, 30, posK);
            printf("%s: %d: set goalK(%d, %d, %d)\n", __FILE__, __LINE__, posK[0], posK[1], posK[2]);
            result2 = pXQH7->wait_for_many_servo_exten(exten);
            if(result1 || result2) {
                printf("%s: %d: error result1(%d) or result2(%d)\n", __FILE__, __LINE__, result1, result2);
                return false;
            }
            _stepIndex = 1;
            delay_ms(50.0); //延迟50ms
        }
        else{
            //如果遇到'l'标志的数据，通过_stepIndex
            //控制运动的精度和速度
            if(_stepIndex >= PosDataPre) {
                result1 = pXQH7->sync_set_many_servo(2, id, 30, posK);
                //result2 = pXQH7->wait_for_many_servo_exten(exten);
                if(result1 /*|| result2*/) {
                    printf("%s: %d: error result1(%d)\n", __FILE__, __LINE__, result1);
                    return false;
                }
                //printf("%s: %d: posK(%d, %d, %d)\n", __FILE__, __LINE__, posK[0], posK[1], posK[2]);
                _stepIndex = 1;
            }
            else
                _stepIndex += 1;
            delay_ms((double)(1000.0 / UpdatePosFre));
        }
        /* 延迟1ms
         * 如果发送的指令无应答包，指令之间应相差100us
         * 此条指令无应答包，指令之间相差1000us
        */
        delay_ms(1.0);
    }

    fclose(fp);

    return true;
}