//
// Created by wuwei on 17-7-26.
//
//存放全局变量，虽然现在还没有想好有什么全局变量需要存放先随便写几个
#ifndef UAVPRODUCT_COMMON_H
#define UAVPRODUCT_COMMON_H

#include "UAVXYZToLatLonWGS84.h"
#include <string>
using namespace std;

#define  __LOG_FILE__ "log"
#define  __PRO_FILE__ "pro" //处理进度


typedef struct __GLOBAL__INFO_
{
    string  _g_image_dir_;
    string  _g_Pos_data;
    int     _g_Pos_bias;
    bool    _g_Has_Pos;

    string _g_auxiliary_dir;    //辅助数据文件夹
    string _g_match_dir_;
    string _g_feature_dir_;
    string _g_geocorrect_dir_;
    string _g_mosaic_path;
    string _g_point_cloud_dir;
    string _g_SFM_data;

    double _g_focal_x;
    double _g_focal_y;
    double _g_ppx;
    double _g_ppy;
    double _g_ccdsize;

} GLOBAL_INFO ;

//全局的文件夹变量
extern GLOBAL_INFO _info_;



#endif //UAVPRODUCT_COMMON_H
