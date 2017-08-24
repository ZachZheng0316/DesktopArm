#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include "Dmath.h"
#include "AXISAlgorithm.h"

using namespace std;
using namespace ROBOTIS;

#define ServoNum (3)    //舵机的个数

AXISAlgorithm::AXISAlgorithm()
{
    //初始化尺寸信息

    //初始化校准刻度

    //初始化减速比
}

//读取尺寸文件
bool AXISAlgorithm::read_hardwareData(char *hardwarePath)
{
    FILE *fpFile = NULL;

    fpFile = fopen(hardwarePath, "r");
    if(fpFile){
        if(3 != fscanf(fpFile, "LA,LB,LC,%lf,%lf,%lf\n", &LA, &LB, &LC)) {
            printf("%s: %d: read LA LB LC failed\n", __FILE__, __LINE__);
            return false;
        }
        //printf("AXISAlgorithm::read_hardwareData:LA,LB,LC,%lf,%lf,%lf\n", LA, LB, LC);

        if(3 != fscanf(fpFile, "D0,D1,D2,%lf,%lf,%lf\n", &D0, &D1, &D2)) {
            printf("%s: %d: read D0 D1 D2 failed\n", __FILE__, __LINE__);
            return false;
        }
        //printf("AXISAlgorithm::read_hardwareData:D0,D1,D2,%lf,%lf,%lf\n", D0, D1, D2);

        if(3 != fscanf(fpFile, "ID1Angle,ID2Angle,ID3Angle,%lf,%lf,%lf\n", &ID1Angle, &ID2Angle, &ID3Angle)) {
            printf("%s: %d: read ID1Angle ID1Angle ID1Angle failed\n", __FILE__, __LINE__);
            return false;
        }
        //printf("AXISAlgorithm::read_hardwareData:ID1Angle,ID2Angle,ID3Angle,%lf,%lf,%lf\n", ID1Angle, ID2Angle, ID3Angle);

        if(3 != fscanf(fpFile, "ID1POSK,ID2POSK,ID3POSK,%d,%d,%d\n", &ID1POSK, &ID2POSK, &ID3POSK)) {
            printf("%s: %d: read ID1Angle ID2Angle ID3Angle failed\n", __FILE__, __LINE__);
            return false;
        }
        //printf("AXISAlgorithm::read_hardwareData:ID1POSK,ID2POSK,ID3POSK,%d,%d,%d\n", ID1POSK, ID2POSK, ID3POSK);

        if(3 != fscanf(fpFile, "Rate1,Rate2,Rate3,%lf,%lf,%lf\n", &Rate1, &Rate2, &Rate3)) {
            printf("%s: %d: read Rate1 Rate2 Rate3 failed\n", __FILE__, __LINE__);
            return false;
        }
        //printf("AXISAlgorithm::read_hardwareData:Rate1,Rate2,Rate3,%lf,%lf,%lf\n", Rate1, Rate2, Rate3);

        return true;
    }
    else {
        printf("%s: %d: open hardwareData file %s failed\n", __FILE__, __LINE__, hardwarePath);
        return false;
    }
    fclose(fpFile);
}

//工具函数
//把rpm转化为舵机弧速度
double AXISAlgorithm::radianSpeedFromRPM(double rpm)
{
    double radianSpeed;
    radianSpeed = rpm * 2 * PI / 60.0;
    return radianSpeed;
}

//把舵机弧速度转化为rpm
double AXISAlgorithm::rpmFromRadianSpeed(double rad)
{
    double rpm;
    rpm = rad * 60.0 / (2 * PI);
    return rpm;
}

//已知舵机当前刻度转换为关节当前角度
double AXISAlgorithm::angleFromPositionK(UINT8_T id, int positionK)
{
    double IDAngleY = 60.0, temp = 0.0;

    if(1 == id) {
        temp = (double)(positionK - ID1POSK);
        temp = temp * PositionUnit;
        temp = temp * Rate1;
        IDAngleY = ID1Angle - temp;
    }
    else if(2 == id) {
        temp = (double)(positionK - ID2POSK);
        temp = temp * PositionUnit;
        temp = temp * Rate2;
        IDAngleY = ID2Angle + temp;
    }
    else if(3 == id) {
        temp = (double)(positionK - ID3POSK);
        temp = temp * PositionUnit;
        temp = temp * Rate3;
        IDAngleY = ID3Angle + temp;
    }
    else
        IDAngleY = -1.0;
    return IDAngleY;
}

//已知关节当前角度转换为舵机当前刻度
int AXISAlgorithm::positionKFromAngle(UINT8_T arm_id, double angle)
{
    int IDPOSKY, IDPOSK;
    double temp, rate;

    if(1 == arm_id) {
        IDPOSK = ID1POSK;
		rate = Rate1;
		temp = ID1Angle - angle;
    }
    else if(2 == arm_id){
        IDPOSK = ID2POSK;
		rate = Rate2;
		temp = angle - ID2Angle;
    }
    else if(3 == arm_id){
        IDPOSK = ID3POSK;
		rate = Rate3;
		temp = angle - ID3Angle;
    }
    else {
        IDPOSKY = -1;
		return IDPOSKY;
    }
    temp = temp / rate;
	temp = temp / PositionUnit;
	IDPOSKY = (int)(IDPOSK + temp + 0.35);

	return IDPOSKY;
}

//已知运动rpm转换为速度刻度
int AXISAlgorithm::speedKFromRPM(double rpm)
{
    double speedK;
	speedK = (int)(rpm / SpeedUnit + 0.4);
	if(speedK <= 1) speedK = 1;
	if(speedK >= 1023) speedK = 1023;
	return (int)speedK;
}

//已知速度刻度转换为rpm
double AXISAlgorithm::rpmFromSpeedK(int speedK)
{
    double rpm;
	rpm = speedK * SpeedUnit;
	if(rpm <= 0.0) rpm = 0.0;
	if(rpm >= SpeedRange) rpm = SpeedRange;
	return rpm;
}

//把关节弧速度转化为对应舵机速度寄存器中的刻度
int AXISAlgorithm::kFromRadianSpeed(int arm_id, double radianSpeed)
{
    int kSpeed;
	double vec = 0.0, rate;

	if(1 == arm_id)
		rate = Rate1;
	else if(2 == arm_id)
		rate = Rate2;
	else if(3 == arm_id)
		rate = Rate3;
	else
		return -1;

	vec = fabs(radianSpeed / rate);         //速度绝对值化
	vec = rpmFromRadianSpeed(vec);		    //把速度转化为rpm
	kSpeed = (int)(abs(vec) / SpeedUnit + 0.35);	//把rpm转化为舵机速度寄存器刻度
	if(kSpeed >= 1023) kSpeed = 1023;
	if(kSpeed <= 1) kSpeed = 1;
	return kSpeed;//返回速度刻度值
}

//运动学函数
//由舵机刻度求得执行端坐标
void AXISAlgorithm::cal_servoK_to_xyz(int servoK[], double xyz[])
{
    int i;
    double angle[ServoNum], o[2], B[2], A[2], C[2], D[2];

    //舵机刻度转化为关节角度
    for(i = 0; i < ServoNum; i++)
        angle[i] = angleFromPositionK((unsigned char)(i + 1), servoK[i]);

    //关节角度转化为执行端坐标
    //在绝对坐标系中计算D点的坐标
    //点o的的坐标
    o[0] = D0; o[1] = LA;
    //计算B的坐标
    B[0] = o[0] + LB * cos(PIFromAngle(angle[1]));
    B[1] = o[1] + LB * sin(PIFromAngle(angle[1]));
    //计算A的坐标
    A[0] = B[0] + LC * cos(PIFromAngle(angle[2]));
    A[1] = B[1] - LC * sin(PIFromAngle(angle[2]));
    //计算C的坐标
    C[0] = A[0] + D1;
    C[1] = A[1];
    //计算D的坐标
    D[0] = C[0];
    D[1] = C[1] - D2;

    //D点在O系中的坐标
    xyz[0] = D[0] * sin(PIFromAngle(angle[0]));
    xyz[1] = D[0] * cos(PIFromAngle(angle[0]));
    xyz[2] = D[1];
}

//由执行端坐标求得舵机刻度
void AXISAlgorithm::cal_xyz_to_servoK(double xyz[], int servoK[])
{
    int i;
    double tan_alpha, alpha, temp;
    double angle[ServoNum], Oxyzabs[3];
    double L_do, mDY, mDZ, mAY, mAZ;
    double AxyzO[3], LAO, beta1, cos_beta1, deltaBeta;
    double tan_deltaBeta, cos_alpha, temp1, beta, gamma2;

    //执行端坐标转化为关节角度
    //点D在o系中的坐标转化为点A在o系中的坐标
    //计算中轴关节偏离O线的角度
    tan_alpha = xyz[0] / xyz[1];
    alpha = atan(tan_alpha);

    //计算O在绝对坐标系中的坐标
    Oxyzabs[0] = D0 * sin(alpha);
    Oxyzabs[1] = D0 * cos(alpha);
    Oxyzabs[2] = LA;
    //printf("O(%f, %f)\n", D0, LA);

    //计算目标点D与点O的距离
    temp = pow(xyz[0] - Oxyzabs[0], 2);
    temp = temp + pow(xyz[1] - Oxyzabs[1], 2);
    temp = temp + pow(xyz[2] - Oxyzabs[2], 2);
    L_do = sqrt(temp);

    //计算点D在坐标系O中的坐标值
    mDZ = xyz[2] - LA;
    temp = pow(L_do, 2) - pow(mDZ, 2);
    mDY = sqrt(temp);
    //printf("D(%f %f)\n", mDY, mDZ);

    //计算点A在坐标O中的坐标值
    mAY = mDY - D1;
    mAZ = mDZ + D2;
    //printf("A(%f %f)\n", mAY, mAZ);

    //返回点A在坐标系O中的坐标值
    AxyzO[0] = alpha;
    AxyzO[1] = mAY;
    AxyzO[2] = mAZ;

    //计算LAO的距离
    temp = pow(AxyzO[1], 2) + pow(AxyzO[2], 2);
    LAO = sqrt(temp);

    //计算beta值
    temp = pow(LB, 2) + pow(LAO, 2) - pow(LC, 2);
    temp1 = 2 * LB * LAO;
    cos_beta1 = temp / temp1;
    beta1 = acos(cos_beta1);

    tan_deltaBeta = AxyzO[2] / AxyzO[1];
    deltaBeta = atan(tan_deltaBeta);

    beta = beta1 + deltaBeta;

    //计算gamma2
    temp = pow(LB, 2) + pow(LC, 2) - pow(LAO, 2);
    temp1 = 2 * LC * LB;
    cos_alpha = temp / temp1;
    alpha = acos(cos_alpha);

    gamma2 = PI - beta - alpha;

    //获得关节弧度
    angle[0] = AxyzO[0];
    angle[1] = beta;
    angle[2] = gamma2;

    //关节角度转化为舵机刻度
    for(i = 0; i < ServoNum; i++) {
        //关节弧度转化为关节角度
        angle[i] = AngleFromPI(angle[i]);
        //关节角度转化为舵机刻度
        servoK[i] = positionKFromAngle(i + 1, angle[i]);
    }
}

//速度反解：已知执行端坐标和速度，求解舵机速度刻度
void AXISAlgorithm::cal_Vxyz_to_VK(double xyz[], float Vxyz[], int VK[])
{
    int i;
    double tan_alpha[2], alpha[2], temp, temp1, temp2;
    double cos_beta1, beta1, tan_deltaBeta, deltaBeta, beta, cos_alpha2, alpha2, gamma2;
    double OxyzAbs[2][3], DxyzAbs[2][3], DxyzO[2][3], AxyzO[2][3], VAO[3];
    double LAO, jointCurRad[3], jointRadSpe[3];
    double m1, m2, n1, n2;

    //计算点A在O系中的速度
    //计算点D在绝对值坐标系中的坐标
     for(i = 0; i < 3; i++) {
        DxyzAbs[0][i] = xyz[i];
        DxyzAbs[1][i] = xyz[i] + Vxyz[i];
     }
    //计算中轴关节偏离O线的角度
    for(i = 0; i < 2; i++) {
        tan_alpha[i] = DxyzAbs[i][0] / DxyzAbs[i][1];
        alpha[i] = atan(tan_alpha[i]);
    }
    //计算O在绝对坐标系中的坐标
    for(i = 0; i < 2; i++) {
        OxyzAbs[i][0] = D0 * sin(alpha[i]);
        OxyzAbs[i][1] = D0 * cos(alpha[i]);
        OxyzAbs[i][2] = LA;
    }

    //计算点D在坐标系O中的坐标值
    for(i = 0; i < 2; i++) {
        DxyzO[i][0] = DxyzAbs[i][0] - OxyzAbs[i][0];
        DxyzO[i][1] = DxyzAbs[i][1] - OxyzAbs[i][1];
        DxyzO[i][2] = DxyzAbs[i][2] - OxyzAbs[i][2];
    }
    //printf("D(%f %f)\n", mDY, mDZ);

    //计算点A在坐标O中的坐标值
    for(i = 0; i < 2; i++) {
        AxyzO[i][0] = DxyzO[i][0];
        AxyzO[i][1] = DxyzO[i][1] - D1;
        AxyzO[i][2] = DxyzO[i][2] + D2;
    }

    //计算LAO的值
    temp = pow(AxyzO[0][1], 2) + pow(AxyzO[0][2], 2);
    LAO = sqrt(temp);

    //计算beta值
    temp = pow(LB, 2) + pow(LAO, 2) - pow(LC, 2);
    temp1 = 2 * LB * LAO;
    cos_beta1 = temp / temp1;
    beta1 = acos(cos_beta1);

    tan_deltaBeta = AxyzO[0][2] / AxyzO[0][1];
    deltaBeta = atan(tan_deltaBeta);

    beta = beta1 + deltaBeta;

    //计算gamma2的值
    temp = pow(LB, 2) + pow(LC, 2) - pow(LAO, 2);
    temp1 = 2 * LC * LB;
    cos_alpha2 = temp / temp1;
    alpha2 = acos(cos_alpha2);

    gamma2 = PI - beta - alpha2;

    //获得关节弧度
    jointCurRad[0] = alpha[0];
    jointCurRad[1] = beta;
    jointCurRad[2] = gamma2;
    //printf("\njointCurRad(%f %f %f)\n", jointCurRad[0], jointCurRad[1], jointCurRad[2]);

    //计算点A在O系中的速度
    for(i = 0; i < 3; i++)
        VAO[i] = AxyzO[1][i] - AxyzO[0][i];
    //printf("VAO(%f %f %f)\n", VAO[0], VAO[1], VAO[2]);

    //求出关节弧速度
    //计算临时参量
    m1 = LB * sin(jointCurRad[1]);
	n1 = LC * sin(jointCurRad[2]);
	m2 = LB * cos(jointCurRad[1]);
	n2 = LC * cos(jointCurRad[2]);
    //计算关节弧速度
    temp1 = n1 * VAO[2] - n2 * VAO[1];
    temp2 = m1 * n2 + m2 * n1;
    jointRadSpe[1] = (fabs)(temp1 / temp2);

    temp1 = -(m2 * VAO[1] + m1 * VAO[2]);
    temp2 = m2 * n1 + m1 * n2;
    jointRadSpe[2] = (fabs)(temp1 / temp2);

    jointRadSpe[0] = (fabs)(alpha[1] - alpha[0]);

    //由关节弧速度求出关节速度刻度值
    for(i = 0; i < ServoNum; i++)
        VK[i] = kFromRadianSpeed(i + 1, jointRadSpe[i]);

    //printf("jointRadSpe(%f %f %f)\n", jointRadSpe[0], jointRadSpe[1], jointRadSpe[2]);
}
