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
 * TODO:数据整理的操作，一般来说给定数据，然后对于数据进行整理是必要的操作
 * TODO:3.地理范围的简单查看，以及估算分辨率
 * TODO:4.估算在此分辨率下进行拼接所需要的内存大小
 */
class UAVPosRead{
public:
    virtual pair<Vec3,Vec3> ReadPOS(fstream *ifs);
};

class UAVDataList {
public:
    /***
     * 根据全局的变量对输入文件夹进行检查以及创建输出文件夹，以及SFM_Data
     * @return 返回所有待处理影像的总大小MB
     */
    float UAVList_CreateSFMList();


};


#endif //UAVPRODUCT_UAVDATALIST_H
