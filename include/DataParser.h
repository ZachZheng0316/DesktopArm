#ifndef DATAPARSERHEADER__H__
#define DATAPARSERHEADER__H__

#include "DeltaAlgorithm.h"

namespace ROBOTIS{
class DataParser{
private:
    DeltaAlgorithm *pAlgorithm;

    bool trans_rough_to_pre(); //把粗率的轨迹转化为精确的轨迹
    bool trans_ink_to_coor(char picPath[]); //把inkscape的数据进行坐标
    void coor_translate(double ps[], double pd[]); //把inkscape的基准坐标转化为机械臂的工作坐标转换
public:
    DataParser(UINT8_T _armType, double _height[], void* _pAlgorithm);
    ~DataParser();
    void trans_coor_to_k(float *xyz, int *k);
    bool trans_ink_to_goalK(char *picPath);
};
}

#endif
