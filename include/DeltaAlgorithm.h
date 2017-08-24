#ifndef DELTAALGORITHMHEADER__H__
#define DELTAALGORITHMHEADER__H__

#include "RobotisDef.h"

namespace ROBOTIS{
class DeltaAlgorithm{
private:

public:
    DeltaAlgorithm();

    bool read_hardwareData(char *hardwarePath); //读取尺寸文件

    //工具函数
    double radianSpeedFromRPM(double rpm);//把rpm转化为舵机弧速度
    double rpmFromRadianSpeed(double rad);//把舵机弧速度转化为rpm
    double angleFromPositionK(UINT8_T id, int positionK);//已知舵机当前刻度转换为关节当前角度
    int positionKFromAngle(UINT8_T arm_id, double angle);//已知关节当前角度转换为舵机当前刻度
    int speedKFromRPM(double rpm);//已知运动rpm转换为速度刻度
    double rpmFromSpeedK(int speedK);//已知速度刻度转换为rpm
    int kFromRadianSpeed(int arm_id, double radianSpeed);       //把关节弧速度转化为对应舵机速度寄存器中的刻度

    //运动学函数
    //由舵机刻度求得执行端坐标
    void cal_servoK_to_xyz(int servoK[], double xyz[]);
    //由执行端坐标求得舵机刻度
    void cal_xyz_to_servoK(double xyz[], int servoK[]);
    //速度反解：已知执行端坐标和速度，求解舵机速度刻度
    void cal_Vxyz_to_VK(double xyz[], float Vxyz[], int VK[]);
};
}

#endif
