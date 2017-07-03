#ifndef DMATHHEADER__H__
#define DMATHHEADER__H__

//MX28AT舵机属性值(12V)
#define AngleRange	  (double)(360.0000000)  //舵机运动的角度范围
#define PositionRange (double)(4095.000000)  //舵机运动的刻度范围
#define PositionUnit  (double)(0.087912088)  //舵机每个刻度对应0.088°
#define SpeedRange    (double)(55.00000000)  //速度最大为55rpm
#define SpeedUnit     (double)(0.053763440)  //每个刻度对应0.054rpm
#define SpeedKRange	  (double)(1023.000000)  //舵机速度刻度范围
#define GOALKBMAXSPEK (double)(3754.266212)  //最大速度时，每秒运转多少个目标刻度
#define GOALKBMINSPEK (double)(3.669358449)  //最小速度时，每秒运转多少个目标刻度
#define ANGLEBMINSPEK (double)(0.322580641)  //当是最小速度时，每秒多少度

//常量
#define PI (double)(3.141592653) //常数PI的值

//换算函数
float AngleFromPI(float pi);             //弧度转角度
float PIFromAngle(float alpha);          //角度转弧度

double disPoint(int num, double p1[], double p2[]);//求两点之间的距离

double cal_triangle_angle(float a, float b, float c); //计算三角形a对边的角度

void delay_ms(double msec); //毫秒延迟函数
void delay_start();         //函数延迟开始
double delay_end();         //函数延迟结束

#endif
