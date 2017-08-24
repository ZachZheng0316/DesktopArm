#include <time.h>
#include <sys/time.h>
#include "Dmath.h"
#include <math.h>
#include <stdio.h>

//弧度转角度
float AngleFromPI(float pi)
{
    float alpha;
    alpha = pi * 180.0 / PI;
    return alpha;
}

//角度转弧度
float PIFromAngle(float alpha)
{
    float pi;
    pi = alpha * PI / 180.0;
    return pi;
}

//求两点之间的距离
double disPoint(int num, double p1[], double p2[])
{
    double sum = 0.0, dis;
    //printf("%s: %d: num(%d)\n", __FILE__, __LINE__, sizeof(p1)/sizeof(p1[0]));
    for(int i = 0; i < num; i++) {
        dis = p1[i] - p2[i];
        sum += pow(dis, 2);
    }

    return sqrt(sum);
}

//计算三角形a对边的角度
double cal_triangle_angle(float a, float b, float c)
{
    float cos_angle, angle;
	float temp1, temp2;

	temp1 = pow(b, 2);
	temp1 += pow(c, 2);
	temp1 -= pow(a, 2);
	temp2 = 2 * b * c;
	cos_angle = temp1 / temp2;

	angle = acos(cos_angle);

	//把弧度转化为角度
	angle = AngleFromPI(angle);

	return angle;
}

//微秒延迟函数
void delay_ms(double msec)
{
    int i = 2000;
    double diff;
    struct timespec tsStart, tsEnd;

    clock_gettime(CLOCK_MONOTONIC, &tsStart);
    do{
        while(i--); i = 2000;
        clock_gettime(CLOCK_MONOTONIC, &tsEnd);
        diff = (double)((tsEnd.tv_sec - tsStart.tv_sec) * 1000 + (double)(tsEnd.tv_nsec - tsStart.tv_nsec) * 0.001 * 0.001);
    }while(diff <= msec);
}


//微妙延迟函数
void delay_us(double usec)
{
    struct timeval start, end;
    unsigned long diff;

    gettimeofday(&start, NULL);
    do{
        gettimeofday(&end, NULL);
        diff = 1000000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
    }while(diff <= usec);
}

//函数延迟开始
struct timespec TSStart;
void delay_start()
{
    clock_gettime(CLOCK_MONOTONIC, &TSStart);
}

//函数延迟结束:以毫秒为单位
double delay_end()
{
    int i = 2000;
    double diff = 0;
    struct timespec TSEnd;
    while(i--);
    clock_gettime(CLOCK_MONOTONIC, &TSEnd);
    diff = (double)((TSEnd.tv_sec - TSStart.tv_sec) * 1000 + (double)(TSEnd.tv_nsec - TSStart.tv_nsec) * 0.001 * 0.001);

    return diff;
}
