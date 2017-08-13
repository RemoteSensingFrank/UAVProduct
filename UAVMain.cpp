#include<stdlib.h>
#include<stdio.h>
#include<GL/glut.h>

#include "UAVDataList.h"
#include "UAVBundle.h"
#include "UAVFeatureExtract.h"


#include "gtest/gtest.h"
void InitialUAVInfo1UAV1()
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


    UAVDataList _datalist_;

    float file_size = _datalist_.UAVList_CreateSFMList();
	UAVFeatsSIFTGpu featGpu;
	//featGpu.UAVFeatsExtract();
	featGpu.UAVMatchesExtract();

	/*printf("process total image size:%lf\n",file_size);
    UAVFeatsSIFT featureSift;
    featureSift.UAVFeatsExtract();
    featureSift.UAVMatchesList(8);
    featureSift.UAVMatchesExtract();
    UAVBundle bundler;
    bundler.UAVBundleGlobal();
	*/
}



int main(int argc,char* argv[])
{
	//测试环境的初始化
	::testing::InitGoogleTest(&argc,argv);

    InitialUAVInfo1UAV1();

	//return RUN_ALL_TESTS();
}
