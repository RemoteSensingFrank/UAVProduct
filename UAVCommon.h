//
// Created by wuwei on 17-8-8.
//

#ifndef UAVPRODUCT_UAVCOMMON_H
#define UAVPRODUCT_UAVCOMMON_H
#include <math.h>
#include <string>
using namespace std;

#define  __LOG_FILE__ "log"
#define  __PRO_FILE__ "pro" //处理进度

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
    string _g_Map_dir;

    double _g_focal_x;
    double _g_focal_y;
    double _g_ppx;
    double _g_ppy;
    double _g_ccdsize;

} GLOBAL_INFO ;

void toTile(int zoom,double Lng,double lat,int &x,int &y) ;

void toLnglat(int zoom,double &lng,double &lat,int x,int y) ;

//全局的文件夹变量
extern GLOBAL_INFO _info_;


#endif //UAVPRODUCT_UAVCOMMON_H
