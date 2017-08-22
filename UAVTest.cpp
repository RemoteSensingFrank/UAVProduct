//
// Created by wuwei on 17-8-18.
//

#include "UAVTest.h"
#include "UAVDataList.h"
#include "UAVBundle.h"
#include "UAVFeatureExtract.h"
#include "UAVDenseProcess.h"
#include "UAVGeoProc.h"
//影像处理全流程测试
#include "UAVCommon.h"
#include "gtest/gtest.h"

TEST(FlowTestGPUDataList,FlowTestGPUDataList)
{
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateSFMList());
}
TEST(FlowTestGPUDataXML,FlowTestGPUDataListXML)
{
    UAVDataList   _datalist_;
    EXPECT_LE(0,_datalist_.UAVList_CreateImageRange(0.5));
}


TEST(FlowTestGPUExtract,FlowTestGPUExtract)
{
    UAVFeatsSIFTGpu  _sift_features_gpu_;
    EXPECT_EQ(true,_sift_features_gpu_.UAVFeatsExtract());
}
TEST(FlowTestGPUMatch,FlowTestGPUMatch)
{
    UAVFeatsSIFTGpu  _sift_features_gpu_;
    EXPECT_EQ(true,_sift_features_gpu_.UAVMatchesExtract());
}
TEST(FlowTestGPUGlobalGpu,FlowTestGPUGlobalGpu)
{
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobalGpu());
}

TEST(FlowTestGPUSequenceGpu,FlowTestGPUSequenceGpu)
{
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleSequenceGpu());
}


TEST(FlowTestCPUExtract,FlowTestCPUExtract)
{
    UAVFeatsSIFT  _sift_features_;
    EXPECT_EQ(true,_sift_features_.UAVFeatsExtract());
}
TEST(FlowTestCPUMatch,FlowTestCPUMatch)
{
    UAVFeatsSIFT  _sift_features_;
    EXPECT_EQ(true,_sift_features_.UAVMatchesExtract());
}
TEST(FlowTestCPUGlobalCpu,FlowTestCPUGlobalCpu)
{
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleGlobal());
}
TEST(FlowTestCPUSequenceCpu,FlowTestCPUSequenceCpu)
{
    UAVBundle _bundler_;
    EXPECT_EQ(true,_bundler_.UAVBundleSequence());
}

TEST(FlowTestGeoCorrect,FlowTestGeoCorrect)
{
    UAVGeoProc  geoProc;
    EXPECT_EQ(true,geoProc.UAVGeoProc_GeoProc(1,6,46));
}