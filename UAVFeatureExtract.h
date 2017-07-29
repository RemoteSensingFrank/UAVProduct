//
// Created by wuwei on 17-7-26.
//

#ifndef UAVPRODUCT_UAVFEATUREEXTRACT_H
#define UAVPRODUCT_UAVFEATUREEXTRACT_H

#include "common.h"

//特征点解算的基类
//解析特征点
//读取特征点
class UAVFeatureExtract
{
public:
    //解析特征点并将其保存到文件中
    virtual bool UAVFeatsExtract() = 0;
    //读取特征点
    virtual bool UAVFeatsLoad() = 0;
};

/**
 * 解析SIFT特征点算子
 */
class UAVFeatsSIFT:public UAVFeatureExtract
{
public:
    bool UAVFeatsExtract();
    bool UAVFeatsLoad();
};

/**
 * 解析SIFT算子通过GPU加速
 */
class UAVFeatsSIFTGpu:public UAVFeatureExtract
{
public:
    bool UAVFeatsExtract();
    bool UAVFeatsLoad();
};

#endif
