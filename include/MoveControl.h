#ifndef MOVECONTROLHEADER__H__
#define MOVECONTROLHEADER__H__
#include "MX28AT.h"
#include "AXISAlgorithm.h"
#include "DeltaAlgorithm.h"
#include "ScaraAlgorithm.h"

namespace ROBOTIS{
class MoveControl{
private:
    int ServoNum;           //舵机个数

    int *PGAIN;             //P
    int *IGAIN;             //I
    int *DGAIN;             //D
    int *SPEEDK;            //速度
    int *ACCK;              //加速度
    int *ENABLEK;           //刚度
    double InitialPos[3];   //初始位置坐标
    double CurrentXYZ[3];   //当前坐标
    int *CurrentK;          //获取当前刻度

    MX28AT *pMX28AT;                //舵机控制指针

    AXISAlgorithm *pAXIS;           //指向4轴算法指针
    DeltaAlgorithm *pDelta;         //指向Delta的算法指针
    ScaraAlgorithm* pScaraAlgo;     //指向Scara的算法指针

public:
    MoveControl(UINT8_T armId);
    ~MoveControl();

    UINT8_T ArmId;

    bool moveInitial(char deviceName[], int baud, int _servoNum); //动作初始化：打开动作设备、运动到初始位置

    void calSpeKWithPer(double percent, int SPEK[]); //根据百分比计算速度

    void getCurrentK(int curK[]);                    //获取当前刻度
    void getCurrentXYZ(double curxyz[]);             //获得当前坐标

    void set_pen_height(int id, int address, int value);            //控制scara的高度

    void setServoByteProperty(int num, int id[], int addr, int value[]);//控制舵机属性
    void setServoWordProperty(int num, int id[], int addr, int value[]);//控制舵机属性

    void calPoint2PointMove(int CurK[], int GoalK[], double tm, int SpeK[]);
    void pointToPointMove(double goalXYZ[], double tm);           //通过设置时间运动到目标位置
    void pointTpPointMove(double goalXYZ[], int SpeK[]);       //通过设置速度运动到目标位置
    void pointToPointMove(double goalXYZ[], int spePercent);//通过设置速度百分比运动目标位置
    void lineMove();        //直线运动
    void rectangleMove();   //矩形运动
    void circleMove();      //圆运动
    void pathMove();        //路径运动
    void waitMoveStop();    //等待运动结束
    void waitMoveStopWithExtern(int externK[]);//等待运动结束

    bool IsMoving(); //判断机械臂是否停止
};
}

#endif
