//
// Created by wuwei on 17-11-5.
//
#include "gtest/gtest.h"
#include "UAVPreProcess.h"

TEST(UAVUnitTest,UnitTestListPOS)
{
    UAVProcessList* list=new UAVProcessList();
    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();

    UAVCalibParams calibCameras;
    UndefinedCalibParam(calibCameras);
    UAVErr  err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/1/Img/","/home/wuwei/Data/UAVData/1/EO.txt",calibCameras,"/home/wuwei/Data/UAVData/1/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
    err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/small_test/Img/","",calibCameras,"/home/wuwei/Data/UAVData/small_test/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
}
TEST(UAVUnitTest,UnitTestListNoPOS)
{
    UAVProcessList* list=new UAVProcessList();
    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();

    UAVCalibParams calibCameras;
    UndefinedCalibParam(calibCameras);
    UAVErr  err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/1/Img/","",calibCameras,"/home/wuwei/Data/UAVData/1/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
    err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/10/Img/","",calibCameras,"/home/wuwei/Data/UAVData/10/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
    err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/8/Img/","",calibCameras,"/home/wuwei/Data/UAVData/8/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
}
TEST(UAVUnitTest,UnitTestListError)
{
    UAVProcessList* list=new UAVProcessList();
    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();

    UAVCalibParams calibCameras;
    UndefinedCalibParam(calibCameras);
    UAVErr  err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/1/NULL/","",calibCameras,"/home/wuwei/Data/UAVData/1/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,1);
    err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/1/Img/","",calibCameras,"/home/wuwei/Data/UAVData/NULL/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,3);
    err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/8/Img/","",calibCameras,"/home/wuwei/Data/UAVData/8/sfm.json",posSimple,CoordinateUTM);
    EXPECT_EQ(err,0);
}