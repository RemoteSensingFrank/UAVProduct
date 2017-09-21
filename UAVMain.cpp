#include<stdlib.h>
#include<stdio.h>
#include "gtest/gtest.h"
#include "UAVCommon.h"


void InitialData5()
{
    _info_._g_image_dir_="/home/wuwei/Data/UAVData/9/Img/";
    _info_._g_SFM_data="/home/wuwei/Data/UAVData/9/SFM_Data.json";
    _info_._g_match_dir_="/home/wuwei/Data/UAVData/9/Matches/";
    _info_._g_feature_dir_="/home/wuwei/Data/UAVData/9/Features/";
    _info_._g_point_cloud_dir="/home/wuwei/Data/UAVData/9/Points/";
    _info_._g_Pos_data = "";
    _info_._g_Pos_bias = 0;
    _info_._g_auxiliary_dir = "/home/wuwei/Data/UAVData/9/Auxiliary/";
    _info_._g_geocorrect_dir_="/home/wuwei/Data/UAVData/9/Geocorrect/";
    _info_._g_mosaic_path = "/home/wuwei/Data/UAVData/9/mosaic.tif";
    _info_._g_Map_dir = "/home/wuwei/Data/UAVData/9/Map/";
    _info_._g_focal_x = _info_._g_focal_y = 3072;
    _info_._g_ccdsize = 0;
    _info_._g_Has_Pos = false;
}

int main(int argc,char* argv[])
{
    InitialData5();
    _info_._g_run("GPU","Sequence",0,0);

    //UAVICPExtract extract;
    //extract.UAVICPExtractMatchesEnvi("/home/wuwei/Data/UAVData/9/Img/0001.jpg","/home/wuwei/Data/UAVData/9/Img/0002.jpg","/home/wuwei/Data/UAVData/9/Img/1-2.pts",EXTRACT_GPU);

    return 0;
}
