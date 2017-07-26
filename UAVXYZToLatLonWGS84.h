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
};


#endif //UAVPRODUCT_UAVXYZTOLATLONWGS84_H
