//
// Created by wuwei on 17-9-15.
//
#pragma once
#ifndef UAVPRODUCT_UAVGEOPROC_H
#define UAVPRODUCT_UAVGEOPROC_H


//点云数据简单处理，转换为DSM数据，内插，几何变换
//TODO:接口还需要实现
class UAVGeoPointsCloud{
public:
    /*
      点云数据转换为DSM数据，输入ply格式的点云数据，输出DSM数据
      参数：输入点云数据路径，DSM输出路径，中心经纬度以及地面分辨率
    */
    void UAVGeoProcPointsCloud_ToDSM(string pathPly,string pathDsm,double dL,double dB,double dGround);

    /*
      点云数据滤波后输出为DSM
      参数：输入点云数据路径，输出DSM路径，中心经纬度，分辨率
    */
    void UAVGeoProcPointsCloud_ToDSMFilter(string pathPly,string pathDsm,double dL,double dB,double dGround);
}

#endif
