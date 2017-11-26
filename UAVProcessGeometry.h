//
// Created by wuwei on 17-11-6.
//

#ifndef UAVPRODUCT_UAVPROCESSGEOMETRY_H
#define UAVPRODUCT_UAVPROCESSGEOMETRY_H
#include "UAVInterface.h"
#include "gdal_priv.h"

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
    UAVErr UAVGeoCorrectGcps(std::string pathImg, GDAL_GCP* gcps,int gcpNumber ,std::string pathGeo);

    //通过外参进行校正
    UAVErr UAVGeoCorrectExterior(std::string pathImg, openMVG::Mat34 P, double avgHeight, std::string pathGeo);

    //DEM几何精校正
    UAVErr UAVGeoCorrectDEM(std::string pathGeo, openMVG::Mat34 P, std::string pathDEM, std::string pathGeoAccur);
};

class UAVProcessGeoBundler:UAVProcessGeoCorrect{
public:
    //对SFM结果进行几何校正
    UAVErr UAVGeoCorrectSFM(std::string sfm,std::string dGeo);

    //通过密集匹配点进行几何校正
    UAVErr UAVGeoCorrectDense(std::string sfm,std::string dGeo,std::string pDense);
};

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

#endif //UAVPRODUCT_UAVPROCESSGEOMETRY_H
