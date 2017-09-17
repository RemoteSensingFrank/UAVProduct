#include<stdlib.h>
#include<stdio.h>
//#include "gtest/gtest.h"
#include "UAVCommon.h"
#include"UAVDenseProcess.h"

void InitialData1()
{
	_info_._g_image_dir_="/home/wuwei/Data/UAVData/small_test/Img/";
	_info_._g_SFM_data="/home/wuwei/Data/UAVData/small_test/SFM_Data.json";
	_info_._g_match_dir_="/home/wuwei/Data/UAVData/small_test/Matches/";
	_info_._g_feature_dir_="/home/wuwei/Data/UAVData/small_test/Features/";
	_info_._g_point_cloud_dir="/home/wuwei/Data/UAVData/small_test/Points/";
	_info_._g_Pos_data = "";
	_info_._g_Pos_bias = 0;
	_info_._g_auxiliary_dir = "/home/wuwei/Data/UAVData/small_test/Auxiliary/";
	_info_._g_geocorrect_dir_="/home/wuwei/Data/UAVData/small_test/Geocorrect/";
	_info_._g_mosaic_path = "/home/wuwei/Data/UAVData/small_test/mosaic.tif";
	_info_._g_focal_x = _info_._g_focal_y = -1;
	_info_._g_ccdsize = 6.16;
	_info_._g_Has_Pos = true;
}


int main(int argc,char* argv[])
{
	//测试环境的初始化
	//InitialData1();

    //testing::GTEST_FLAG(output) = "xml:TestReport.xml";
	//testing::InitGoogleTest(&argc,argv);
    //return RUN_ALL_TEST();

	UAVDenseProcess pointcloud;
	pointcloud.UAVDPCloud_ToDSM("/home/wuwei/Data/UAVData/1/Points/dense.ply","/home/wuwei/Data/UAVData/1/Points/dem.tif",124,41,0.3);
	return 0;
}

