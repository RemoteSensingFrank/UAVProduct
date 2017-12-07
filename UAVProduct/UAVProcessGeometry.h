//
// Created by wuwei on 17-11-6.
//

#ifndef UAVPRODUCT_UAVPROCESSGEOMETRY_H
#define UAVPRODUCT_UAVPROCESSGEOMETRY_H
#include "UAVInterface.h"
#include "openMVG/sfm/sfm.hpp"
#include "gdal_priv.h"

#define MINXBound -20037508
#define MINYBound -20037508
#define MAXXBound 20037508
#define MAXYBound 20037508.34
#define MAPUNITSIZE 256

const double Lods[20][3]={
        {0,156543.03392800014,591657527.591555},
        {1,78271.516963999937,295828763.79577702},
        {2,39135.758482000092,147914381.89788899},
        {3,19567.879240999919,73957190.948944002},
        {4,9783.9396204999593,36978595.474472001},
        {5,4891.9698102499797,18489297.737236001},
        {6,2445.9849051249898,9244648.8686180003},
        {7,1222.9924525624949,4622324.4343090001},
        {8,611.49622628138,2311162.217155},
        {9,305.748113140558,1155581.108577},
        {10,152.874056570411,577790.554289},
        {11,76.4370282850732,288895.277144},
        {12,38.2185141425366,144447.638572},
        {13,19.1092570712683,72223.819286},
        {14,9.55462853563415,36111.909643},
        {15,4.7773142679493699,18055.954822},
        {16,2.3886571339746849,9027.9774109999998},
        {17,1.1943285668550503,4513.9887049999998},
        {18,0.59716428355981721,2256.994353},
        {19,0.298582142,1128.4971765}};

typedef struct mapunit{
    int row;
    int col;
    int level;
    std::string unit_save;
    std::string unit_url;
}MAPUNIT;

typedef std::vector<MAPUNIT> MapCalculateUnits;

//只做接口
class UAVMapCalculate:public UAVProcessGeometry
{
public:
    //根据中心经纬度和影像范围解算切片块
    MapCalculateUnits UAVMapCalculateUnit(double centerUTMx,double centerUTMy,double width,double height,int level);

    //对于单个切片计算其角点坐标
    bool UAVMapUnitCorner(MAPUNIT unitMap,double &cornerx,double &cornery);

    // 将切片数据组装一下
    bool UAVMapUnitCombie(MapCalculateUnits units,std::string dest);

    //根据url解析得到影像坐标并拼装起来
    virtual bool UAVMapUnitURL(MapCalculateUnits &units)=0;
    //根据切片块组合为url并保存到文件中
    virtual bool UAVMapUnitData(MapCalculateUnits units)=0;
};

class UAVMapCalculateGoogle:public UAVMapCalculate{
public:
    UAVMapCalculateGoogle(){
        Google_URL="http://mt3.google.cn/vt/lyrs=s@110&hl=zh-CN&gl=cn&src=app&";
    }

    //直接根据全局变量得到影像对应的瓦片地图
    bool UAVMapGoogleRun(std::string sfm,std::string dMap);

    //根据url解析得到影像坐标并拼装起来
    virtual bool UAVMapUnitURL(MapCalculateUnits &units);

    //根据切片块组合为url并保存到文件中
    virtual bool UAVMapUnitData(MapCalculateUnits units);
private:
    std::string Google_URL;
};


class UAVProcessGeoCorrect:public UAVProcessGeometry{
public:
    //通过控制点进行校正
    UAVErr UAVGeoCorrectGcps(std::string pathImg, GDAL_GCP* gcps,int gcpNumber ,std::string pathGeo,
                                            double dGroundSize,double dL,double dB);

    //还是只计算五个点，而不是所有点都计算，所有点都计算存在两个问题：１．效率太低；２．不同的像素有不同的偏差
    UAVErr UAVGeoCorrectExterior(std::string pathImg, openMVG::Mat34 P, double avgHeight, std::string pathGeo,
                                            double dGroundSize,double dL,double dB);

    UAVErr UAVGeoCorrectDEM(std::string pathGeo, openMVG::Mat34 P, std::string pathDEM, std::string pathGeoAccur);

protected:
    /***
     * unsigned char* pDataSrc:输入影像数据
     * xMap：ｘ映射
     * yMap：ｙ映射
     * double dGroundSize：地面点分辨率
     * int xsrc，int ysrc：源影像大小
     * int xre,int yre　：采样后影像大小
     * unsigned char* pDataRe：采样后数据
     */
    virtual void UAVGeoProc_ImageResample(unsigned char *pDataSrc, float *xMap, float *yMap, double maxpt[], double minpt[],
                                  double dGroundSize, int xsrc, int ysrc, int xre, int yre, unsigned char *pDataRe);
};

/***
 * 密集点解算
 * Dense Points calculate
 */
class UAVProcessGeoDense:UAVProcessGeometry{
    friend class UAVProcessGeoBundler;
public:
    //设置坐标转换类型
    UAVErr UAVGeoDenseSetCoordinateType();

    //密集点云转换为DEM
    UAVErr UAVGeoDenseToDEM(std::string dense,std::string dem);

protected:
    UAVErr UAVGeoIndex();
    UAVErr UAVGeoDenseInterpolation();
};

/***
 * 光束法平差结果进行几何校正
 * Geo correction using bundler result
 */
class UAVProcessGeoBundler:UAVProcessGeoCorrect{
public:
    //对SFM结果进行几何校正
    UAVErr UAVGeoCorrectSFM(std::string sfm,std::string dGeo,CoordiListType typeCoordi,double dGroundSize);

    //通过密集匹配点进行几何校正
    UAVErr UAVGeoCorrectDense(std::string sfm,std::string dGeo,std::string pDense);
};


class UAVProcessGCPs:UAVProcessGeometry{
public:
    virtual openMVG::sfm::Landmarks UAVGeoGCPImport(std::string fgcp,CoordiListType coordiTpIn,CoordiListType coordiTpOut)=0;
};


#endif //UAVPRODUCT_UAVPROCESSGEOMETRY_H
