//
// Created by wuwei on 17-3-26.
//
#pragma once
#ifndef UAVPRODUCT_UAVGEOPROC_H
#define UAVPRODUCT_UAVGEOPROC_H

#include "gdal_priv.h"
#include "gdalwarper.h"
#include "gdal_alg_priv.h"
#include "gdal_alg.h"
#include "ogrsf_frmts.h"

#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <fstream>

#include<vector>
#include<string>

using namespace std;
#include <string>

using namespace std;

class UAVGeoProc {
public:
    /***
     * 根据影像控制点进行几何校正
     * @param pathSFM ：SFM数据路径
     * @param pathDstDir ：几何校正结果输出路径
     * @param dL ：中心经度
     * @param dB ：中心纬度
     */
    void UAVGeoProc_GeoProc(string pathSFM,string pathDstDir,double dL,double dB);

    /***
     * 将地心直角坐标系转换为UTM坐标系
     * @param gcps ：控制点（输入-输出）
     * @param num ：控制点个数
     * @param dL ：中心经度
     * @param dB ：中心纬度
     */
    void UAVGetProc_GeoCoordiTrans(double* gcps,int num,double dL,double dB);

    /***
     * 通过控制点进行几何校正
     * @param image ：影像路径
     * @param gcps ：控制点
     * @param gcpNum ：控制点数
     * @param dGroundSize ：地面分辨率
     * @param dL ：中心经度
     * @param dB ：中心纬度
     * @param geoImage ：校正后的影像
     */
    void UAVGeoProc_GeoCorrection(string image,double* gcps,int gcpNum,double dGroundSize,double dL,double dB,string geoImage);

    /***
     * 成像区域点的输出
     * @param pathSFM ：SFM文件路径
     * @param cameraInfo ：输出相机的位置
     * @param focalpix ：焦距（按像素记）
     * @param scale ：缩放尺寸
     */
    void UAVGeoProc_GetCameraInfo(string pathSFM,string cameraInfo,float focalpix,float scale);
};

#endif //UAVPRODUCT_UAVGEOPROC_H
