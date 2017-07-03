#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include "Dmath.h"
#include "DeltaAlgorithm.h"

using namespace std;
using namespace ROBOTIS;

#define ServoNum (3)

//尺寸信息
double RO;
double LB;
double LA;
double R_1O;
double H;

//校准刻度信息
double ID1Angle; //90.00
double ID2Angle; //90.00
double ID3Angle; //90.00
int ID1POSK;     //773
int ID2POSK;     //870
int ID3POSK;     //801

//减速比
double Rate1;   //1:2.4 = 0.4166666
double Rate2;   //1:2.4 = 0.4166666
double Rate3;   //1:2.4 = 0.4166666

DeltaAlgorithm::DeltaAlgorithm()
{
    RO = 0.0;
    LB = 0.0;
    LA = 0.0;
    R_1O = 0.0;
    H = 0.0;
}

//读取尺寸文件
bool DeltaAlgorithm::read_hardwareData(char *hardwarePath)
{
    FILE *fpFile = NULL;

    fpFile = fopen(hardwarePath, "r");
    if(fpFile){
        if(3 != fscanf(fpFile, "RO,R_1O,H,%lf,%lf,%lf\n", &RO, &R_1O, &H)) {
            printf("%s: %d:read RO R_1O H failed\n", __FILE__, __LINE__);
            return false;
        }
        printf("%s: %d: RO,R_1O,H,%lf,%lf,%lf\n", __FILE__, __LINE__, RO, R_1O, H);

        if(2 != fscanf(fpFile, "LA,LB,%lf,%lf\n", &LA, &LB)) {
            printf("%s: %d:read LB LA failed\n", __FILE__, __LINE__);
            return false;
        }
        printf("%s: %d: LB,LA,%lf,%lf\n", __FILE__, __LINE__, LB, LA);

        if(3 != fscanf(fpFile, "ID1Angle,ID2Angle,ID3Angle,%lf,%lf,%lf\n", &ID1Angle, &ID2Angle, &ID3Angle)) {
            printf("%s: %d:read ID1Angle ID1Angle ID1Angle failed\n", __FILE__, __LINE__);
            return false;
        }
        printf("%s: %d: ID1Angle,ID2Angle,ID3Angle,%lf,%lf,%lf\n", __FILE__, __LINE__, ID1Angle, ID2Angle, ID3Angle);

        if(3 != fscanf(fpFile, "ID1POSK,ID2POSK,ID3POSK,%d,%d,%d\n", &ID1POSK, &ID2POSK, &ID3POSK)) {
            printf("%s: %d:read ID1Angle ID2Angle ID3Angle failed\n",  __FILE__, __LINE__);
            return false;
        }
        printf("%s: %d:ID1POSK,ID2POSK,ID3POSK,%d,%d,%d\n", __FILE__, __LINE__, ID1POSK, ID2POSK, ID3POSK);

        if(3 != fscanf(fpFile, "Rate1,Rate2,Rate3,%lf,%lf,%lf\n", &Rate1, &Rate2, &Rate3)) {
            printf("%s: %d:read Rate1 Rate2 Rate3 failed\n", __FILE__, __LINE__);
            return false;
        }
        printf("%s: %d:Rate1,Rate2,Rate3,%lf,%lf,%lf\n", __FILE__, __LINE__, Rate1, Rate2, Rate3);

        return true;
    }
    else {
        printf("%s: %d:open hardwareData file %s failed\n", __FILE__, __LINE__, hardwarePath);
        return false;
    }
    fclose(fpFile);
}

//工具函数
//把rpm转化为舵机弧速度
double DeltaAlgorithm::radianSpeedFromRPM(double rpm)
{
    double radianSpeed;
    radianSpeed = rpm * 2 * PI / 60.0;
    return radianSpeed;
}

//把舵机弧速度转化为rpm
double DeltaAlgorithm::rpmFromRadianSpeed(double rad)
{
    double rpm;
    rpm = rad * 60.0 / (2 * PI);
    return rpm;
}

//已知舵机当前刻度转换为关节当前角度
double DeltaAlgorithm::angleFromPositionK(UINT8_T id, int positionK)
{
    int IDPOSK;
    double temp, IDAngleY;

    if (0x01 == id)
        IDPOSK = ID1POSK;
    if (0x02 == id)
        IDPOSK = ID2POSK;
    if (0x03 == id)
        IDPOSK = ID3POSK;

    temp = (double)(positionK - IDPOSK);
    temp *= PositionUnit;
    temp *= Rate1;              //Rate1 == Rate2 == Rate3
    IDAngleY = ID1Angle - temp; //ID1Angle == ID2Angle == ID3Angle

    return IDAngleY;
}

//已知关节当前角度转换为舵机当前刻度
int DeltaAlgorithm::positionKFromAngle(UINT8_T arm_id, double angle)
{
    int IDPOSKY, IDPOSK;
    double temp, rate;

    if(0x01 == arm_id) {
        IDPOSK = ID1POSK;
        temp = ID1Angle - angle;
    }
    if(0x02 == arm_id) {
        IDPOSK = ID2POSK;
        temp = ID2Angle - angle;
    }
    if(0x03 == arm_id) {
        IDPOSK = ID3POSK;
        temp = ID3Angle - angle;
    }

    temp = temp / Rate1;           //Rate1 == Rate2 == Rate3
    temp = temp / PositionUnit;
    IDPOSKY = (int)(IDPOSK + temp + 0.30);

    return IDPOSKY;
}

//已知运动rpm转换为速度刻度
int DeltaAlgorithm::speedKFromRPM(double rpm)
{
    double speedK;

    speedK = (int)(rpm / SpeedUnit + 0.4);
    if(speedK <= 1) speedK = 1;
    if(speedK >= 1023) speedK = 1023;

    return (int)speedK;
}

//已知速度刻度转换为rpm
double DeltaAlgorithm::rpmFromSpeedK(int speedK)
{
    double rpm;

    rpm = speedK * SpeedUnit;
    if(rpm <= 0.0) rpm = 0.0;
    if(rpm >= SpeedRange) rpm = SpeedRange;

    return rpm;
}

//把关节弧速度转化为对应舵机速度寄存器中的刻度
int DeltaAlgorithm::kFromRadianSpeed(int arm_id, double radianSpeed)
{
    int kSpeed;
    double vec = 0.0, rate;

    if(1 == arm_id)
        rate = Rate1;
    if(2 == arm_id)
        rate = Rate1;
    if(3 == arm_id)
        rate = Rate1;

    vec = fabs(radianSpeed / rate);
    vec = rpmFromRadianSpeed(vec);
    kSpeed = (int)(vec / SpeedUnit + 0.35);
    if(kSpeed >= 1023) kSpeed = 1023;
    if(kSpeed <= 1) kSpeed = 1;

    return kSpeed;
}

//运动学函数
//由舵机刻度求得执行端坐标
void DeltaAlgorithm::cal_servoK_to_xyz(int servoK[], double xyz[])
{
}

//由执行端坐标求得舵机刻度
void DeltaAlgorithm::cal_xyz_to_servoK(double xyz[], int servoK[])
{
    double radian[3], angle[3], K, U, V;
    double KUVtemp, result1, result2;
    double temp;

    //获得第1个系数组
	KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp + (RO-R_1O)*(sqrt(3.0)*xyz[0] + xyz[1]);
    temp = KUVtemp / LB;
    K = temp + 2.0 * xyz[2]; //计算K值

    temp = 2.0 * (RO-R_1O);
    temp = temp - sqrt(3.0) * xyz[0];
    temp = temp - xyz[1];
    U = -2.0 * temp; //计算U值

    temp = KUVtemp / LB;
    V = temp - 2.0 * xyz[2]; //计算V值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

    radian[0] = atan(result2);
    radian[0] = 2.0 * radian[0];

    //获得第2个系数组
    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O)*(sqrt(3.0)*xyz[0] - xyz[1]);
    temp = KUVtemp / LB;
    K = temp + 2.0 * xyz[2]; //计算K值

    temp = 2.0 * (RO-R_1O);
    temp = temp + sqrt(3.0) * xyz[0];
    temp = temp - xyz[1];
    U = -2.0 * temp; //计算U值

    temp = KUVtemp / LB;
    V = temp - 2.0 * xyz[2]; //计算V值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

    radian[1] = atan(result2);
    radian[1] = 2.0 * radian[1];

    //获得第3个系数组
    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O) * (2.0 * xyz[1]);
    temp = KUVtemp / (2 * LB);
    K = temp + xyz[2]; //计算K值

    temp = RO-R_1O;
    temp = temp + xyz[1];
    U = -2.0 * temp; //计算U值

    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O) * (2.0 * xyz[1]);
    temp = KUVtemp / (2 * LB);
    V = temp - xyz[2]; //计算K值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

    radian[2] = atan(result2);
    radian[2] = 2.0 * radian[2];

    //把关节弧度转化为关节角度
    angle[0] = AngleFromPI(radian[0]);
    angle[1] = AngleFromPI(radian[1]);
    angle[2] = AngleFromPI(radian[2]);
    //printf("%s: %d: radian(%lf, %lf, %lf) angle(%lf, %lf, %lf)\n", __FILE__, __LINE__, radian[0], radian[1], radian[2], angle[0], angle[1], angle[2]);

    //把关节角度转化为舵机刻度
    servoK[0] = positionKFromAngle(0x01, angle[0]);
    servoK[1] = positionKFromAngle(0x02, angle[1]);
    servoK[2] = positionKFromAngle(0x03, angle[2]);
    //printf("%s: %d: servoK(%d, %d, %d)\n", __FILE__, __LINE__, servoK[0], servoK[1], servoK[2]);
}

//速度反解：已知执行端坐标和速度，求解舵机速度刻度
void DeltaAlgorithm::cal_Vxyz_to_VK(double xyz[], float Vxyz[], int VK[])
{
}
