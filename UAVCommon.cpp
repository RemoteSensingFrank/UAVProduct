//
// Created by wuwei on 17-8-8.
//
#include "UAVCommon.h"

GLOBAL_INFO _info_;

void toTile(int zoom,double Lng,double lat,int &x,int &y) {
    double n = pow(2, zoom);
    double tileX = ((Lng + 180) / 360) * n;
    double tileY = (1 - (log(tan(lat*M_PI/180) + (1 / cos(lat*M_PI/180))) / M_PI)) / 2 * n;
    x=tileX;
    y=tileY;
}

void toLnglat(int zoom,double &lng,double &lat,int x,int y) {
    double n = pow(2, zoom);
    lng =x / n * 360.0 - 180.0;
    lat = atan(sinh(M_PI * (1 - 2 * y / n)));
    lat = lat * 180.0 / M_PI;
}