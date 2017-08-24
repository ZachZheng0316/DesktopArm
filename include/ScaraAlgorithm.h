#ifndef SCARAALGORITHMHEADER__H__
#define SCARAALGORITHMHEADER__H__

#include "RobotisDef.h"

namespace ROBOTIS{
class ScaraAlgorithm{
private:
    double J;
    double G;
    double H;

    double B[2];
    double C[2];
    double XY[2];
    double ANGLE[2];

    //校准参数
    double IDAngle;
    int ID1POSK;
    int ID2POSK;
    double REDUCTION_RATE;

public:
    ScaraAlgorithm();

    int positionKFromAngle(UINT8_T arm_id, double angle);//已知关节当前角度转换为舵机当前刻度

    void cal_xyz_to_servoK(double* xy, int* goalK); //已知坐标求舵机刻度
    void cal_servoK_to_xyz(int* goalK, double* xy); //已知舵机刻度求坐标

};
}

#endif
