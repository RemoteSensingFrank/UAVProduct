//
// Created by wuwei on 17-7-26.
//

#ifndef UAVPRODUCT_UAVXYZTOLATLONWGS84_H
#define UAVPRODUCT_UAVXYZTOLATLONWGS84_H


#include"numeric/numeric.h"

// WGS84 Ellipsoid
static const double WGS84_A = 6378137.0;      // major axis
static const double WGS84_B = 6356752.314245; // minor axis
static const double WGS84_E = 0.0818191908;   // first eccentricity

using namespace openMVG;
class UAVXYZToLatLonWGS84 {

public:
    Vec3 BLHToXYZ(double lat,
                  double lon,
                  double alt);

    Vec3 XYZToBLH(double x,
                  double y,
                  double z);

    //trans from lat&lon&altitude to XYZ coordinate in WGS84
    Vec3 LatLonToXYZ(double lat,
                     double lon,
                     double alt);

    //trans from lat&lon&altitude to UTM coordinate in WGS84
    Vec3 LatLonToUTM(double lat,
                     double lon,
                     double alt,
                     double a=WGS84_A,
                     double b=WGS84_B);

    //trans from XYZ to lat&lon&altitude coordinate in WGS84
    Vec3 XYZToLatLon(double x,
                     double y,
                     double z);

    //web Mector
    Vec3 UTMToLatLonWMT(double x,
                        double y,
                        double z);

    Vec3 LatLonToUTMWMT(double lat,
                        double lon,
                        double alt);

    //GCJ01
    bool OutOfChina(double lat, double lon);

    void WGSLatLonToGCJ(double wgLon,
                        double wgLat,
                        double &mgLon,
                        double &mgLat);
};

//为什么要增加这个，主要是因为在后期的计算过程中需要根据高程进行校正在相对坐标系下更好处理
class POSProc{
public:
    long POSProc_POSTrans(Vec3 centerllat,Vec3 curllat,Vec3 rot,Vec3 &eo);

    long POSProc_POSTrans(Vec3 centerllat,Vec3 curllat,Vec3 &eo);


};

#endif //UAVPRODUCT_UAVXYZTOLATLONWGS84_H
