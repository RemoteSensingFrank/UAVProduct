//
// Created by wuwei on 17-8-18.
//
#include "UAVDataList.h"
#include "UAVBundle.h"
#include "UAVFeatureExtract.h"
#include "UAVDenseProcess.h"
#include "UAVGeoProc.h"
//影像处理全流程测试
#include "UAVCommon.h"
#include "gtest/gtest.h"
/*
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

void InitialData2()
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
    _info_._g_focal_x = _info_._g_focal_y = -1;
    _info_._g_ccdsize = 35;
    _info_._g_Has_Pos = true;
}


TEST(DataSet1GPUFlowTest,GPUTest)
{
    InitialData1();
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateSFMList());
    EXPECT_LE(0,_datalist_.UAVList_CreateImageRange(0.5));
    UAVFeatsSIFTGpu  _sift_features_gpu_;
    EXPECT_EQ(true,_sift_features_gpu_.UAVFeatsExtract());
    EXPECT_EQ(true,_sift_features_gpu_.UAVMatchesExtract());
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobalGpu());
    UAVGeoProc  geoProc;
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
    EXPECT_EQ(true,_bundler_.UAVBundleSequenceGpu());
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
}
TEST(DataSet1CPUFlowTest,CPUTest)
{
    InitialData1();
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateSFMList());
    EXPECT_LE(0,_datalist_.UAVList_CreateImageRange(0.5));
    UAVFeatsSIFT  _sift_features_;
    EXPECT_EQ(true,_sift_features_.UAVFeatsExtract());
    EXPECT_EQ(true,_sift_features_.UAVMatchesExtract());
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobal());
    UAVGeoProc  geoProc;
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
    EXPECT_EQ(true,_bundler_.UAVBundleSequence());
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
}


TEST(DataSet2GPUFlowTest,GPUTest)
{
    InitialData2();
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateSFMList());
    EXPECT_LE(0,_datalist_.UAVList_CreateImageRange(0.5));
    UAVFeatsSIFTGpu  _sift_features_gpu_;
    EXPECT_EQ(true,_sift_features_gpu_.UAVFeatsExtract());
    EXPECT_EQ(true,_sift_features_gpu_.UAVMatchesExtract());
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobalGpu());
    UAVGeoProc  geoProc;
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
    EXPECT_EQ(true,_bundler_.UAVBundleSequenceGpu());
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
}

TEST(DataSet2CPUFlowTest,CPUTest)
{
    InitialData2();
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateSFMList());
    EXPECT_LE(0,_datalist_.UAVList_CreateImageRange(0.5));
    UAVFeatsSIFT  _sift_features_;
    EXPECT_EQ(true,_sift_features_.UAVFeatsExtract());
    EXPECT_EQ(true,_sift_features_.UAVMatchesExtract());
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobal());
    UAVGeoProc  geoProc;
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
    EXPECT_EQ(true,_bundler_.UAVBundleSequence());
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
}
*/