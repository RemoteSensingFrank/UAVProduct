//
// Created by wuwei on 17-7-26.
//

#ifndef UAVPRODUCT_UAVFEATUREEXTRACT_H
#define UAVPRODUCT_UAVFEATUREEXTRACT_H

#include "UAVCommon.h"
#include "SiftGPU/SiftGPU.h"

//特征点解算的基类
//解析特征点
//读取特征点
class UAVFeatureExtract
{
public:
    //解析特征点并将其保存到文件中
    virtual bool UAVFeatsExtract() = 0;
    //读取特征点并进行匹配
    virtual bool UAVMatchesExtract() = 0;
    //匹配范围的列表
    virtual bool UAVMatchesList(int neighbor_count) ;
};

/**
 * 解析SIFT特征点算子
 * 特征点进行匹配
 */
class UAVFeatsSIFT:public UAVFeatureExtract
{
public:
    bool UAVFeatsExtract();
    bool UAVMatchesExtract();
};

/**
 * 解析SIFT算子通过GPU加速
 * 通过GPU加速进行匹配
 */
class UAVFeatsSIFTGpu:public UAVFeatureExtract
{
public:
    bool UAVFeatsExtract();
    bool UAVMatchesExtract();
};

#endif
