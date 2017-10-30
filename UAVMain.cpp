#include<stdlib.h>
#include<stdio.h>
#include "UAVCommon.h"
#include "UAVGeoProc.h"
#include "UAVDenseProcess.h"


void InitialData()
{
    _info_._g_image_dir_="/home/wuwei/Data/UAVData/10/Img/";
    _info_._g_SFM_data="/home/wuwei/Data/UAVData/10/SFM_Data.json";
    _info_._g_match_dir_="/home/wuwei/Data/UAVData/10/Matches/";
    _info_._g_feature_dir_="/home/wuwei/Data/UAVData/10/Features/";
    _info_._g_point_cloud_dir="/home/wuwei/Data/UAVData/10/Points/";
    _info_._g_Pos_data = "";
    _info_._g_Pos_bias = 0;
    _info_._g_auxiliary_dir = "/home/wuwei/Data/UAVData/10/Auxiliary/";
    _info_._g_geocorrect_dir_="/home/wuwei/Data/UAVData/10/Geocorrect/";
    _info_._g_mosaic_path = "/home/wuwei/Data/UAVData/10/mosaic.tif";
    _info_._g_Map_dir = "/home/wuwei/Data/UAVData/10/Map/";
    _info_._g_focal_x = _info_._g_focal_y = 5616;
    _info_._g_ccdsize = 1;
    _info_._g_Has_Pos = false;
}

int main(int argc,char* argv[])
{
    InitialData();
    _info_._g_run("GPU","Sequence",0,0);

    //UAVICPExtract extract;
    //extract.UAVICPExtractMatchesEnvi("/home/wuwei/Data/UAVData/9/Img/0001.jpg","/home/wuwei/Data/UAVData/9/Img/0002.jpg","/home/wuwei/Data/UAVData/9/Img/1-2.pts",EXTRACT_GPU);

    //UAVDenseProcess dense;
    //dense.UAVDPCloud_ToDSM("/home/wuwei/Data/UAVData/1/Points/dense.ply","/home/wuwei/Data/UAVData/1/Points/dem.tif",124,41,0.5);

    //UAVGeoProc geo;
    //geo.UAVGeoProc_GeoProcDEM("/home/wuwei/Data/UAVData/1/Points/sfm_data.bin","/home/wuwei/Data/UAVData/1/Points/dem.tif","/home/wuwei/Data/UAVData/1/Geocorrect/",0.5,124,41);
    return 0;

}

