#include<stdlib.h>
#include<stdio.h>
#include "gtest/gtest.h"
#include "UAVMapCalculate.h"
#include "UAVCommon.h"
#include "UAVDataList.h"

void InitialData5()
{
    _info_._g_image_dir_="/home/wuwei/Data/UAVData/1/Img/";
    _info_._g_SFM_data="/home/wuwei/Data/UAVData/1/SFM_Data.json";
    _info_._g_match_dir_="/home/wuwei/Data/UAVData/1/Matches/";
    _info_._g_feature_dir_="/home/wuwei/Data/UAVData/1/Features/";
    _info_._g_point_cloud_dir="/home/wuwei/Data/UAVData/1/Points/";
    _info_._g_Pos_data = "/home/wuwei/Data/UAVData/1/EO.txt";
    _info_._g_Pos_bias = 0;
    _info_._g_auxiliary_dir = "/home/wuwei/Data/UAVData/1/Auxiliary/";
    _info_._g_geocorrect_dir_="/home/wuwei/Data/UAVData/1/Geocorrect/";
    _info_._g_mosaic_path = "/home/wuwei/Data/UAVData/1/mosaic.tif";
    _info_._g_Map_dir = "/home/wuwei/Data/UAVData/1/Map/";
    _info_._g_focal_x = _info_._g_focal_y = -1;
    _info_._g_ccdsize = 35;
    _info_._g_Has_Pos = true;
}

int main(int argc,char* argv[])
{
	//测试环境的初始化
    //testing::GTEST_FLAG(output) = "xml:TestReport.xml";
	//testing::InitGoogleTest(&argc,argv);
    //return RUN_ALL_TESTS();
    InitialData5();
    UAVMapCalculateGoogle mapCalculte;
    UAVDataList dataList;
    dataList.UAVList_CreateSFMList();
    dataList.UAVList_CreateImageRange(0.5);
    mapCalculte.UAVMapGoogleRun();
}
