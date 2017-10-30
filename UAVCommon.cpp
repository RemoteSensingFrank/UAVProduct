//
// Created by wuwei on 17-8-8.
//
#include "UAVCommon.h"
#include "UAVDataList.h"
#include "UAVBundle.h"
#include "UAVFeatureExtract.h"
#include "UAVDenseProcess.h"
#include "UAVGeoProc.h"
#include <math.h>

GLOBAL_INFO _info_;

void GLOBAL_INFO::_g_run(string Type_P,string Type_B,double dL,double dB) {
    UAVDataList   _datalist_;
    UAVFeatureExtract *_sift_features_;

    if(!_datalist_.UAVList_CreateSFMList())
        return ;
    if(!_datalist_.UAVList_CreateImageRange(0.5))
    {
        printf("无法计算地理范围!\n");
    }
    //CPU解算
    if(Type_P=="CPU")
    {
        _sift_features_=new UAVFeatsSIFT();
        if(!_sift_features_->UAVFeatsExtract())
        {
            printf("解算特征点失败!");
            return;
        }
        if(_g_Has_Pos)
        {
            if(!_sift_features_->UAVMatchesList(8))
            {
                return ;
            }
        }
        else
        {
            if(!_sift_features_->UAVMatchesList(0))
            {
                return ;
            }
        }
        if(!_sift_features_->UAVMatchesExtract())
        {
            printf("特征点匹配失败!");
            return ;
        }
        UAVBundle _bundler_;
        if(Type_B=="Global")
        {
            if(!_bundler_.UAVBundleGlobal())
            {
                printf("光束法平差失败!");
                return ;
            }
        }
        if(Type_B=="Sequence")
        {
            if(!_bundler_.UAVBundleSequence())
            {
                printf("光束法平差失败!");
                return ;
            }
        }
    }
    if(Type_P=="GPU"){
        _sift_features_=new UAVFeatsSIFTGpu();
        if(!_sift_features_->UAVFeatsExtract())
        {
            printf("解算特征点失败!");
            return;
        }
        if(_g_Has_Pos)
        {
            if(!_sift_features_->UAVMatchesList(8))
            {
                return ;
            }
        }
        else {
            if (!_sift_features_->UAVMatchesList(0)) {
                return;
            }
        }
            if(!_sift_features_->UAVMatchesExtract())
        {
            printf("特征点匹配失败!");
            return ;
        }

        UAVBundle _bundler_;
        if(Type_B=="Global")
        {
            if(!_bundler_.UAVBundleGlobalGpu())
            {
                printf("光束法平差失败!");
                return ;
            }
        }
        if(Type_B=="Sequence")
        {
            if(!_bundler_.UAVBundleSequenceGpu())
            {
                printf("光束法平差失败!");
                return ;
            }
        }
    }

    //几何粗校正
    //UAVGeoProc _geo_;
    //_geo_.UAVGeoProc_GeoProc(0,dL,dB);

    //密集匹配
    UAVDenseProcess _dense_;
    _dense_.UAVDP_ExportMVS();
    _dense_.UAVDP_MVSProc();
}


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

