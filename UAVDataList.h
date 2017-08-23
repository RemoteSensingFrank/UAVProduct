//
// Created by wuwei on 17-7-26.
//

#ifndef UAVPRODUCT_UAVDATALIST_H
#define UAVPRODUCT_UAVDATALIST_H

#include "UAVCommon.h"
#include "UAVXYZToLatLonWGS84.h"
#include <string>
#include <fstream>

/***
 * 数据整理的操作，一般来说给定数据，然后对于数据进行整理是必要的操作
 */
class UAVPosRead{
public:
    virtual pair<Vec3f,Vec3f> ReadPOS(fstream &ifs);
};

class UAVDataList {
public:
    /***
     * 根据全局的变量对输入文件夹进行检查以及创建输出文件夹，以及SFM_Data
     * @return 返回所有待处理影像的总大小MB
     */
    float UAVList_CreateSFMList();

    /***
     * 根据POS数据初步获取飞行点的位置和大致范围
     * @return 返回预计计算的拼接后影像大小的估计
     */
    float UAVList_CreateImageRange(double dGroundSize);


};


#endif //UAVPRODUCT_UAVDATALIST_H
