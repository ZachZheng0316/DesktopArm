#include <stdio.h>
#include <stdlib.h>
#include "Dmath.h"
#include "RobotisDef.h"
#include "DataParser.h"

using namespace std;
using namespace ROBOTIS;

char coorfile[] = "source.txt";  //由inkscape转换而来的坐标文件
char tra_coor[] = "tracoor.txt"; //获得详细的轨迹坐标
char tra_k[] = "trak.txt";       //轨迹坐标对应的刻度坐标
UINT8_T ArmType;  //机械臂的种类
double HEIGHT[3]; //三级高度:HEIGHT[0, 1, 2]=[高，中，低]
double SIZE;      //工作范围SIZE*SIZE

DataParser::DataParser(UINT8_T _armType, double _height[], void* _pAlgorithm)
{
    for(int i = 0; i < 3; i++)
        HEIGHT[i] = _height[i];

    if(0x20 == (_armType&0xf0)) {
        ArmType = DELTAID;
        pAlgorithm = (DeltaAlgorithm*)_pAlgorithm;
        SIZE = 180.0;
    }
    else
        ArmType = 0x00;
}

DataParser::~DataParser()
{
}

/**************************************************
函数意义：
    把坐标转化为刻度值
参数意义：
    xyz:坐标值
    k  :刻度值
返回值:
    无返回值
**************************************************/
void DataParser::trans_coor_to_k(float *xyz, int *k)
{
    double _xyz[3];
    int servoK[3];
    _xyz[0] = (double)xyz[0];
    _xyz[1] = (double)xyz[1];
    _xyz[2] = (double)xyz[2];
    pAlgorithm->cal_xyz_to_servoK(_xyz, servoK);
    k[0] = servoK[0];
    k[1] = servoK[1];
    k[2] = servoK[2];
}

/************************************************
函数意义：
    把inkscape的数据进行坐标转换，使其能在项目中使用。
    inkscape中的数据主要由'm'、's'标志的信息组成。
    'm':表示一段路径的起始点，数据是二维坐标。
    's':表示一段路径的过程点，数据是二维坐标。
    起始点为[0.0, 0.0, HEIGHT1]
    路径转化的过程如下：
    当遇到'm'标志时，把标志数据分拆为以下三种数据：
        1、保存当前位置，高度为HEIGHT1;
        2、保存pd位置，高度为HEIGHT1;
        3、保存pd位置，高度为HEIGHT2(落笔的高度);
    当遇到's'标志时，直接保存's'标志的数据。
参数意义：
    picPath:要绘制的图片的路径
返回值：
    如果执行成功，返回true;
    如果执行失败，返回false;
************************************************/
bool DataParser::trans_ink_to_coor(char *picPath)
{
    char flag, prevFlag = 'm'; //preFlag指向前一个数据
    double ps[2], pd[3], cur[3] = {0.0, 0.0, 0.0};
    FILE *fp_source = NULL, *fp_result = NULL;

    fp_source = fopen(picPath, "r");
    fp_result = fopen(coorfile, "w");

    pd[2] = HEIGHT[2];
    while(!feof(fp_source)) {
        if(EOF == fscanf(fp_source, "%c (%lf %lf)\n", &flag, &ps[0], &ps[1]))
            return false;
        coor_translate(ps, pd);
        //printf("%s: %d: flag(%c) ps(%lf, %lf)\n", __FILE__, __LINE__, flag, ps[0], ps[1]);
        //printf("%s: %d: flag(%c) pd(%lf, %lf, %lf)\n", __FILE__, __LINE__, flag, pd[0], pd[1], pd[2]);

        if('m' == flag) {
            if('l' == prevFlag) {
                cur[2] = HEIGHT[2];
                fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, cur[0], cur[1], cur[2]);
            }

            //调整高度, 高度为HEIGHT1
            cur[2] = HEIGHT[1];
            fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, cur[0], cur[1], cur[2]);

            //运动到起始点，高度为HEIGHT1
            pd[2] = HEIGHT[1];
            fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, pd[0], pd[1], pd[2]);

            //开始运动，高度为HEIGHT2
            pd[2] = HEIGHT[2];
            fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, pd[0], pd[1], pd[2]);
        }
        else
            fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, pd[0], pd[1], pd[2]);

        //更新数据
        prevFlag = flag;
        cur[0] = pd[0];
        cur[1] = pd[1];
    }

    //回到圆点
    flag = 'm'; cur[2] = HEIGHT[1];
    fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, cur[0], cur[1], cur[2]);
    flag = 'm'; cur[0] = 0.0; cur[1] = 0.0;
    fprintf(fp_result, "%c (%lf %lf %lf)\n", flag, cur[0], cur[1], cur[2]);

    fclose(fp_source);
    fclose(fp_result);

    return true;
}

/**********************************************
函数意义：
    把粗率的轨迹转化为精确的轨迹。
    以0.5mm为最基本的单位
参数意义：
    无参数
返回值：
    如果执行成功，返回true;
    如果执行失败，返回false;
**********************************************/
bool DataParser::trans_rough_to_pre()
{
    double ps[3], pd[3], px[3];
    double dis, unit = 0.1, startStep = 0.0; //线性函数控制变量
    char flag[2];
    FILE *fp1 = NULL, *fp2 = NULL;

    fp1 = fopen(coorfile, "r");
    fp2 = fopen(tra_coor, "w");
    if(!fp1 || !fp2) {
        printf("%s: %d: open file %s or %s failed\n", __FILE__, __LINE__, coorfile, tra_coor);
        return false;
    }

    //搜索到以'm'开头的起始点
    while(1) {
        if(EOF==fscanf(fp1, "%c (%lf %lf %lf)\n", &flag[0], &ps[0], &ps[1], &ps[2]))
            return false;
        if('m' == flag[0])
            break;
    }
    fprintf(fp2, "%c (%lf %lf %lf)\n", flag[0], ps[0], ps[1], ps[2]);
    //printf("%s: %d: %c (%lf %lf %lf)\n", __FILE__, __LINE__, flag[0], ps[0], ps[1], ps[2]);
    while(!feof(fp1)) {
        if(EOF == fscanf(fp1, "%c (%lf %lf %lf)\n", &flag[1], &pd[0], &pd[1], &pd[2]))
            return false;
        //如果搜索到'm'标志的数据,直接保存
        if('m' == flag[1]) {
            //if('l' == flag[0]) {
            //    fprintf(fp2, "%c (%lf %lf %lf)\n", flag[0], ps[0], ps[1], ps[2]);
            //    printf("%s: %d: %c (%lf %lf %lf)\n", __FILE__, __LINE__, flag[0], ps[0], ps[1], ps[2]);
            //}
            fprintf(fp2, "%c (%lf %lf %lf)\n", flag[1], pd[0], pd[1], pd[2]);
            //更新数据
            flag[0] = flag[1];
            ps[0] = pd[0];
            ps[1] = pd[1];
            ps[2] = pd[2];
        }
        else if('l' == flag[1]){
            //搜索到以'l'为标志的数据
            dis = disPoint(3, ps, pd);
            if(dis > unit) { //如果dis大于0.5
                //float unitNum = dis/unit;
                //float unitStep = 1.0/unitNum;
                float unitStep = unit / dis;
                if('m' == flag[0])
                    startStep = unitStep;
                if('l' == flag[0])
                    startStep = 0.0;
                int index = 0;
                for(float t = startStep; t < 1.0; t += unitStep) {
                    px[0] = t * (pd[0] - ps[0]) + ps[0];
                    px[1] = t * (pd[1] - ps[1]) + ps[1];
                    px[2] = t * (pd[2] - ps[2]) + ps[2];
                    if((0 == startStep) &&(0 == index)) {
                        fprintf(fp2, "%c (%lf %lf %lf)\n", flag[1], px[0], px[1], px[2]);
                        index = 1;
                    }
                    else
                        fprintf(fp2, "%c (%lf %lf %lf)\n", flag[1], px[0], px[1], px[2]);
                        //fprintf(fp2, "X (%lf %lf %lf) %lf\n", px[0], px[1], px[2], dis);
                    //printf("%s: %d: %c (%lf %lf %lf)\n", __FILE__, __LINE__, flag[1], px[0], px[1], px[2]);
                }
                //更新数据
                flag[0] = flag[1];
            }
            else{
                //如果dis小于0.5
                fprintf(fp2, "%c (%lf %lf %lf) %lf\n", flag[1], pd[0], pd[1], pd[2], dis);
                //更新数据
                flag[0] = 'm';
            }
            ps[0] = pd[0];
            ps[1] = pd[1];
            ps[2] = pd[2];
        }
    }
    fprintf(fp2, "%c (%lf %lf %lf)\n", flag[1], pd[0], pd[1], pd[2]);

    fclose(fp1);
    fclose(fp2);

    return true;
}

/*********************************************
函数意义：
    把来自inkscape的数据转化为舵机可以运动的刻度数据
参数意义：
    picPath:要绘制的图片的路径。
返回值：
    如果执行成功，返回true;
    如果执行失败，返回false;
*********************************************/
bool DataParser::trans_ink_to_goalK(char* picPath)
{
    FILE *fp1 = NULL, *fp2 = NULL;
    float xyz[3], _stepIndex = 1;
    int posK[3];
    char flag;

    fp2 = fopen(tra_k, "w");
    if(!fp2){
        printf("%s: %d: fopen %s failed\n", __FILE__, __LINE__, tra_k);
        return false;
    }

    //把inkscape数据转化为粗率的轨迹数据
    if(trans_ink_to_coor(picPath)) {
        //把粗率的轨迹数据转化为精确的轨迹数据
        if(trans_rough_to_pre()) {
            fp1 = fopen(tra_coor, "r");
            if(!fp1) {
                printf("%s: %d: fopen %s failed\n", __FILE__, __LINE__, tra_coor);
                return false;
            }
            while(!feof(fp1)) {
                //把精确的轨迹数据转化为运动的刻度数据
                if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag, &xyz[0], &xyz[1], &xyz[2])) {
                    printf("%s: %d: read %s failed\n", __FILE__, __LINE__, coorfile);
                    return false;
                }
                trans_coor_to_k(xyz, posK); //把坐标转化为刻度
                fprintf(fp2, "%c (%d %d %d)\n", flag, posK[0], posK[1], posK[2]);
            }
        }
    }

    //关闭文件
    fclose(fp1);
    fclose(fp2);

    return true;
}

/*********************************************
函数意义：
    把inkscape的基准坐标转化为机械臂的工作坐标
**********************************************/
void DataParser::coor_translate(double ps[], double pd[])
{
    if(0x20 == (ArmType&0xf0)) {
        //delta项目
        pd[0] = ps[0] - 0.5 * SIZE;
        pd[1] = ps[1] - 0.5 * SIZE;
    }
}
