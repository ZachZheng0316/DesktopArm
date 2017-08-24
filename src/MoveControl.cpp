#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "AXISAlgorithm.h"
#include "RobotisDef.h"
#include "MoveControl.h"
#include "MX28AT.h"
#include "Dmath.h"

using namespace std;
using namespace ROBOTIS;

//机械臂初始位置
double AXISInitialPos[3] = {0.00, 162.00, 70.00};
double DeltaInitialPos[3] = {0.00, 0.00, -300.00};
double ScaraInitiaPos[3] = {0.00, 165.00, 0.00};

//函数意义
//  1. 初始化变量参数
//  2. 根据armId设置初始位置坐标
//  3. 根据armId设置算法指针
MoveControl::MoveControl(UINT8_T armId)
{
    UINT8_T armType = armId & 0xf0;
    UINT8_T _armId = (int)(armId & 0x0f);
    char sizePath[30];

    ArmId = armId;  /*设置机械臂的种类*/

    /*如果是四轴机械臂*/
    if(0x10 == armType) {
        ServoNum = 3;
        /*更新坐标值*/
        for(int i = 0; i < 3; i++) {
            InitialPos[i] = AXISInitialPos[i];
            CurrentXYZ[i] = AXISInitialPos[i];
        }
        pAXIS = new AXISAlgorithm();
        //读取4轴硬件尺寸
        sprintf(sizePath, "config/AXISNO.%d", _armId);
        if(pAXIS->read_hardwareData(sizePath))
            printf("%s: %d: read_hardwareData axis success\n", __FILE__, __LINE__);
        else
            printf("%s: %d: read_hardwareData axis failed\n", __FILE__, __LINE__);
    }
    else if(0x20 == armType) { /*如果是Delta机械臂*/
        ServoNum = 3;
        //delta
        for(int i = 0; i < ServoNum; i++) {
            InitialPos[i] = DeltaInitialPos[i];
            CurrentXYZ[i] = DeltaInitialPos[i];
            CurrentK[i] = 0;
        }
        pDelta = new DeltaAlgorithm();
        //读取delta硬件尺寸
        sprintf(sizePath, "./size");
        if(pDelta->read_hardwareData(sizePath))
            printf("%s: %d: MoveControl::MoveControl:read hardwareData success\n", __FILE__, __LINE__);
        else
            printf("%s: %d: MoveControl::MoveControl:read hardwareData failed\n", __FILE__, __LINE__);
    }
    else if(0x30 == armType) { /*如果是Scara机械臂*/
        ServoNum = 2;
        //scara
        pScaraAlgo = new ScaraAlgorithm();
    }
    else if(0x40 == armType) {
        ServoNum = 10;
    }

    /*申请关键性数据存储空间*/
    PGAIN = new int[ServoNum];
    IGAIN = new int[ServoNum];
    DGAIN = new int[ServoNum];
    SPEEDK = new int[ServoNum];
    ACCK = new int[ServoNum];
    ENABLEK = new int[ServoNum];
    CurrentK = new int[ServoNum];
    for (int i = 0; i < ServoNum; i++) {
        PGAIN[i] = 64;
        IGAIN[i] = 32;
        DGAIN[i] = 30;
        SPEEDK[i] = 1023;
        ACCK[i] = 10;
        ENABLEK[i] = 1;
    }
}

MoveControl::~MoveControl()
{
    delete PGAIN;
    delete IGAIN;
    delete DGAIN;
    delete SPEEDK;
    delete ACCK;
    delete ENABLEK;
    delete CurrentK;
}

//动作初始化：打开动作设备、运动到初始位置
bool MoveControl::moveInitial(char deviceName[], int baud, int _servoNum)
{
    int servoK[_servoNum];

    /*申请动作控制类*/
    pMX28AT = new MX28AT(deviceName, baud, _servoNum);

    /*如果是scara*/
    if(0x30 == (ArmId & 0xf0)) {
        if(pMX28AT->SuccessOpenDevice()) {
            printf("%s: %d: MX28AT isSuccessInit is OK\n", __FILE__, __LINE__);
            int id[2] = {1, 2}, value[2];
            value[0] = 64; value[1] = 64;
            pMX28AT->set_many_servo_byte(2, id, P_Gain, value);
            value[0] = 32; value[1] = 32;
            pMX28AT->set_many_servo_byte(2, id, I_Gain, value);
            value[0] = 30; value[1] = 30;
            pMX28AT->set_many_servo_byte(2, id, D_Gain, value);
            value[0] = 30; value[0] = 30;
            pMX28AT->set_many_servo_byte(2, id, Goal_Acceleration, value);
            value[0] = 1; value[1] = 1;
            pMX28AT->set_many_servo_byte(2, id, Torque_Enable, value);

            /*运动到目标位置*/
            pointToPointMove(ScaraInitiaPos, 1.5);

            return true;
        }
        else {
            printf("%s: %d: MX28AT isSuccessInit is failed\n", __FILE__, __LINE__);
            return false;
        }
    }
    else if(0x10 == (ArmId & 0xf0)) {
    /*如果是四轴*/
        if(pMX28AT->SuccessOpenDevice()) {
            printf("%s: %d: MX28AT isSuccessInit is OK\n", __FILE__, __LINE__);
            int id[3] = {1, 2, 3};
            pMX28AT->set_many_servo_byte(ServoNum, id, P_Gain, PGAIN);
            pMX28AT->set_many_servo_byte(ServoNum, id, I_Gain, IGAIN);
            pMX28AT->set_many_servo_byte(ServoNum, id, D_Gain, DGAIN);
            pMX28AT->set_many_servo_byte(ServoNum, id, Goal_Acceleration,ACCK);
            pMX28AT->set_many_servo_byte(ServoNum, id, Torque_Enable, ENABLEK);

            /*运动到目标位置*/
            pointToPointMove(AXISInitialPos, 1.5);

            return true;
        }
        else {
            printf("%s: %d: MX28AT isSuccessInit is failed\n", __FILE__, __LINE__);
            return false;
        }
    }

    return true;
}

/*
控制舵机byte属性
当num = 1时，设置单个舵机属性；
当num > 1时，设置多个舵机属性；
*/
void MoveControl::setServoByteProperty(int num, int id[], int addr, int value[])
{
    if(1 == num) {
        pMX28AT->set_one_servo_byte(id[0], addr, value[0]);
    }
    else if(1 <= num) {
        pMX28AT->set_many_servo_byte(num, id, addr, value);
    }
    else{
        printf("%s: %d: the num of servos is less than 1\n", __FILE__, __LINE__);
    }
}

/*
设置舵机word属性
当num = 1时，设置单个舵机属性；
当num > 1时，设置多个舵机属性；
*/
void MoveControl::setServoWordProperty(int num, int id[], int addr, int value[])
{
    /*舵机数量为1个*/
    if(1 == num) {
        pMX28AT->set_one_servo_word(id[0], addr, value[0]);
    }
    else if(num >= 1) {
        pMX28AT->set_many_servo_word(num, id, addr, value);
    }
    else{
        printf("%s: %d: the num of servos is less than 1\n", __FILE__, __LINE__);
    }
}

//根据百分比计算速度
//函数意义：
//  1. 根据给定的百分比，计算给定的速度
//  2. 当百分比为50.0时，速度为给定的速度
//  3. 当百分比为[50.0, 100.0]时，速度倍率为[1.0, 3.0]
//  4. 当百分比为[0.0, 50.0]时，速度倍率为[1.0/3.0, 1.0]
//参数意义：
//  percent：速度百分比为[0.0, 100.0]
//  SPEK：要改变的速度
//返回值：
//  属于本地操作，返回值由SPEK体现。
void MoveControl::calSpeKWithPer(double percent, int SPEK[])
{
    double maxMultiple = 1023.0, _multiple;
    int goalK[ServoNum], _curK[ServoNum];

    //计算最大的可扩大倍数
    for(int i = 0; i <ServoNum; i++) {
        _multiple = 1023.0 / (double)(SPEK[i]+1.0);
        if(maxMultiple >= _multiple)
            maxMultiple = _multiple;
    }
    //计算倍率
    if((percent >= 0.0) && (percent < 50.0)) {
        _multiple = 0.300 + (0.700 * percent)/50.0;
    }
    else if((percent >= 50.0) && (percent <= 100.0)) {
        percent = percent - 50.0;
        _multiple = 1.00 + (2.00 * percent)/50.0;
        if(_multiple >= maxMultiple)
            _multiple = maxMultiple;
    }

    //计算速度刻度:使速度在[10, 1023]之间
    for(int i = 0; i < ServoNum; i++) {
        SPEK[i] = (int)((double)SPEK[i] * _multiple + 0.50);
        SPEK[i] = (int)(10.5 + (1013.0 * SPEK[i])/1023.0);
    }
}

/*获取当前刻度*/
void MoveControl::getCurrentK(int curK[])
{
    int id[ServoNum];

    /*设置机械臂ID*/
    if(0x30==(ArmId&0xf0)) {
        id[0] = 1; id[1] = 2;
    }
    else if (0x10 == (ArmId&0xf0)) {
        id[0] = 1; id[1] = 2; id[2] = 3;
    }
    else{}

    /*获取当前刻度*/
    for(int i = 0; i < ServoNum; i++) {
        CurrentK[i] = pMX28AT->get_one_servo_word(id[i], Present_Position);
        curK[i] = CurrentK[i];
    }

}

//获得当前坐标
void MoveControl::getCurrentXYZ(double curxyz[])
{
    int curK[ServoNum];

    //读取当前刻度
    getCurrentK(curK);

    if(0x10 == (ArmId&0xf0)) {
        //当前刻度转化为坐标
        pAXIS->cal_servoK_to_xyz(curK, CurrentXYZ);
        for(int i = 0; i < ServoNum; i++)
            curxyz[i] = CurrentXYZ[i];
    }
}

//控制scara的高度
void MoveControl::set_pen_height(int id, int address, int value)
{
    pMX28AT->set_pen_height(id, address, value);
}

/*
通过给定的当前刻度、目标刻度以及时间，
到达目标位置时的速度刻度
*/
void MoveControl::calPoint2PointMove(int CurK[], int GoalK[], double tm, int SpeK[])
{
    int currentK[ServoNum], diffK[ServoNum];
    int maxDiff = 0, minDiff = 4096;
    double tempTm, spe[ServoNum];

    /*获取刻度距离之差*/
    for(int i = 0; i < ServoNum; i++) {
        diffK[i] = abs(GoalK[i] - CurK[i]) + 1;
        if(diffK[i] > maxDiff)
            maxDiff = diffK[i];
        if(diffK[i] < minDiff)
            minDiff = diffK[i];
    }

    /*计算最短运动时间*/
    tempTm = maxDiff / GOALKBMAXSPEK;
    if(tm < tempTm) {
        tm = tempTm;
        printf("%s: %d: the tm is less than the least time\n", __FILE__, __LINE__);
    }

    /*计算最长运动时间*/
    tempTm = minDiff / GOALKBMINSPEK;
    if(tm > tempTm) {
        tm = tempTm;
        printf("%s: %d: the tm is more than the longest time\n", __FILE__, __LINE__);
    }

    /*计算运动速度*/
    for(int i = 0; i < ServoNum; i++) {
        spe[i] = (diffK[i] * PositionUnit) / tm;
        SpeK[i] = spe[i] / ANGLEBMINSPEK;
    }
}

/*
通过设置时间运动到目标位置
当tm == 0.0时，按照现有的速度运动到目标位置
当tm > 0.0时，按照指定的时间运动到目标位置
*/
void MoveControl::pointToPointMove(double goalXYZ[], double tm)
{
    int id[ServoNum], value[ServoNum], currentK[ServoNum];
    double tempTm = 0.0;
    int _SpeK[ServoNum];

    /*如果四轴*/
    if(0x10 == (ArmId&0xf0)) {
        id[0] = 1; id[1] = 2; id[2] = 3;

        pAXIS->cal_xyz_to_servoK(goalXYZ, value);/*计算目标刻度*/

        if(0.0 == tm) {
            pMX28AT->set_many_servo_word(ServoNum, id, Goal_Position, value);
        }
        else if(0.0 <= tm) {
            getCurrentK(currentK);  /*获取当前刻度*/
            calPoint2PointMove(currentK, value, tm, _SpeK);

            /*设置目标位置和目标速度*/
            pMX28AT->set_many_servo_word(ServoNum, id, Moving_Speed, _SpeK);
            pMX28AT->set_many_servo_word(ServoNum, id, Goal_Position, value);
        }
        else{
            printf("%s: %d: the tm is less than 0.0\n", __FILE__, __LINE__);
        }
    }
    else if(0x30 == (ArmId&0xf0)) {
        id[0] = 1; id[1] = 2;

        pScaraAlgo->cal_xyz_to_servoK(goalXYZ, value);  /*计算目标刻度*/

        if(0.0 == tm) {
            pMX28AT->set_many_servo_word(ServoNum, id, Goal_Position, value);
        }
        else if(0.0 <= tm) {
            getCurrentK(currentK);  /*获取当前刻度*/
            calPoint2PointMove(currentK, value, tm, _SpeK);

            /*设置目标位置和目标速度*/
            pMX28AT->set_many_servo_word(ServoNum, id, Moving_Speed, _SpeK);
            pMX28AT->set_many_servo_word(ServoNum, id, Goal_Position, value);
        }
        else{
            printf("%s: %d: the tm is less than 0.0\n", __FILE__, __LINE__);
        }
    }
    else{
        printf("%s: %d: other armtype\n", __FILE__, __LINE__);
    }
}

//通过设置速度运动到目标位置
void MoveControl::pointTpPointMove(double goalXYZ[], int SpeK[])
{
    int id[ServoNum], value[ServoNum];

    /*如果机械臂是AXIS*/
    if(0x10 == (ArmId&0xf0)) {
        id[0] = 1; id[1] = 2; id[2] = 3;
        pAXIS->cal_xyz_to_servoK(goalXYZ, value);/*计算目标刻度*/

        /*设置目标位置和目标速度*/
        pMX28AT->set_many_servo_word(ServoNum, id, Moving_Speed, SpeK);
        pMX28AT->set_many_servo_word(ServoNum, id, Goal_Position, value);
    }
}

//点对点运动
//函数意义
//  1.输入目标位置，当当前位置运动到目标位置
//参数意义：
//  goalXYZ：目标位置坐标
//  spePercent：最大速度比例，0.0：代表默认速度；返回为[0.0, 100.0]
//函数返回值：
//  函数无返回值
void MoveControl::pointToPointMove(double goalXYZ[], int spePercent)
{
    /*
    int* goalK = new int[3], *_curK = new int[3];
    int* speK = new int[3];

    //获取当前刻度
    getCurrentK(_curK);
    //计算目标刻度
    if(0x20 == (ArmId&0xf0)) {
        pDelta->cal_xyz_to_servoK(goalXYZ, goalK);
        //计算速度
        for(int i = 0; i < ServoNum; i++) {
            //计算当前刻度与目标刻度的差值
            speK[i] = abs(goalK[i] - _curK[i])/2; //两舵机的刻度之差最大不会超过2048
            //把速度从[0, 1023]映射到[10, 1023]中
            speK[i] = (int)(10.5 + 1013.0 *(double)speK[i] / 1023.00);
        }

        //计算速度
        calSpeKWithPer(spePercent, speK);
        //设置速度
        pMX28AT->set_many_servo_word(ServoNum, Moving_Speed, speK);
        //运动到目标位置
        pMX28AT->set_many_servo_word(Goal_Position, goalK);
    }
*/
}

//直线运动
void MoveControl::lineMove()
{
}

//矩形运动
void MoveControl::rectangleMove()
{
}

//圆运动
void MoveControl::circleMove()
{
}

//路径运动
void MoveControl::pathMove()
{
}

//等待运动结束
void MoveControl::waitMoveStop()
{
    //如果机械臂是scara
    if(0x30 == (ArmId&0xf0)) {
        int num = 2, id[2] = {1, 2};
        pMX28AT->wait_for_many_servo(num, id);
    }
}

//等待运动结束
void MoveControl::waitMoveStopWithExtern(int externK[])
{
    //如果是scara
    if(0x30 == (ArmId&0xf0)) {
        int num = 2, id[2] = {1, 2};
        pMX28AT->wait_for_many_servo_exten(num, id, externK);
    }
}

//判断机械臂是否停止
bool MoveControl::IsMoving()
{
    //如果机械臂是scara
    if(0x30 == (ArmId&0xf0)) {
        for(int i = 1; i <= 2; i++)
            if(pMX28AT->IsMoving(i))
                return true;
        return false;
    }
}
