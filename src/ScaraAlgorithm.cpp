#include "ScaraAlgorithm.h"
#include "Dmath.h"
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace ROBOTIS;

ScaraAlgorithm::ScaraAlgorithm()
{
    J = 90.00; G = 180.00; H = 235.0;
    B[0] = -J/2.0; B[1] = 0.0;
    C[0] = J/2.0; C[1] = 0.0;

    XY[0] = 0.0; XY[1] = 0.0;
    ANGLE[0] = 0.0; ANGLE[1] = 0.0;

    IDAngle = 0.00;
    ID1POSK = 3493; ID2POSK = 741;
    REDUCTION_RATE = 0.500;
}

//已知关节当前角度转换为舵机当前刻度
int ScaraAlgorithm::positionKFromAngle(UINT8_T arm_id, double angle)
{
    double diffAngle;
    int posK;

    //计算出舵机角度差
    diffAngle = (angle - IDAngle) / REDUCTION_RATE;

    //计算舵机刻度
    if(0x01 == arm_id)
        posK = (int)(ID1POSK*1.0 - diffAngle/PositionUnit + 0.35);
    else
        posK = (int)(ID2POSK*1.0 + diffAngle/PositionUnit + 0.35);

    return posK;
}

//已知坐标求舵机刻度
void ScaraAlgorithm::cal_xyz_to_servoK(double* xy, int* goalK)
{
    double k, l;
    double EBF, DCF;
    double GBF, HCF;

    //设置此时目标
    XY[0] = xy[0];
    XY[1] = xy[1];

    //计算k, l
    k = disPoint(XY, B);
    l = disPoint(XY, C);

    //计算角度EBF，DCF
    EBF = cal_triangle_angle(H, G, k);
    DCF = cal_triangle_angle(H, G, l);

    //计算GBF，HCF
    GBF = (XY[0] - B[0]) / (XY[1] - B[1]);
    HCF = (XY[0] - C[0]) / (XY[1] - C[1]);
    GBF = atan(GBF);
    HCF = atan(HCF);
    GBF = AngleFromPI(GBF);
    HCF = AngleFromPI(HCF);

    //关节角度
    ANGLE[0] = EBF - GBF;
    ANGLE[1] = DCF + HCF;

    //printf("%s: %d:angle(%lf, %lf)\n", __FILE__, __LINE__, ANGLE[0], ANGLE[1]);

    //由关节角度求舵机刻度
    goalK[0] = positionKFromAngle(0x01, ANGLE[0]);
    goalK[1] = positionKFromAngle(0x02, ANGLE[1]);
}

//已知舵机刻度求坐标
void ScaraAlgorithm::cal_servoK_to_xyz(int* goalK, double* xy)
{

}
