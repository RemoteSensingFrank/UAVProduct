//
// Created by wuwei on 17-11-5.
//
#include "gtest/gtest.h"
#include "UAVPreProcess.h"
#include <memory>
#include <iostream>

//TODO:1．构造测试数据　
//TODO:2．将所有测试用例用测试数据进行重新编写
//TODO:3．测试用例对所有分支进行覆盖
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
    UAVErr  err=list->UAVProcessListGet("/home/wuwei/Data/UAVData/9/Img/","",calibCameras,"/home/wuwei/Data/UAVData/9/sfm.json",posSimple,CoordinateUTM);
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

TEST(UAVUnitTest,UnitTestMatch)
{
    std::string strList = "/home/wuwei/Data/UAVData/1/sfm.json";
    std::string strMatch="/home/wuwei/Data/UAVData/1/matches.txt";
    std::shared_ptr<UAVProcessMatches> match(new UAVProcessMatches());
    UAVErr err;
    err=match->UAVProcessMatchesList(strList,1,false,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,0,false,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,8,false,strMatch);
    EXPECT_EQ(0,err);
    strList="/home/wuwei/Data/UAVData/8/sfm.json";
    strMatch="/home/wuwei/Data/UAVData/8/matches.txt";
    err=match->UAVProcessMatchesList(strList,1,true,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,0,true,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,8,true,strMatch);
    EXPECT_EQ(0,err);
}

TEST(UAVUnitTest,UnitTestMatchError)
{
    std::string strList = "/home/wuwei/Data/UAVData/1/sfm.json";
    std::string strMatch="/home/wuwei/Data/UAVData/1/matches.txt";
    std::shared_ptr<UAVProcessMatches> match(new UAVProcessMatches());

    UAVErr err;
    err=match->UAVProcessMatchesList(strList,1,true,"/home/wuwei/Data/UAVData/1/test/matches.txt");
    EXPECT_EQ(4,err);
    err=match->UAVProcessMatchesList(strList,0,true,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,8,true,strMatch);
    EXPECT_EQ(0,err);
    strList="/home/wuwei/Data/UAVData/8/sfm.json";
    strMatch="/home/wuwei/Data/UAVData/8/matches.txt";
    err=match->UAVProcessMatchesList(strList,1,false,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,0,false,strMatch);
    EXPECT_EQ(0,err);
    err=match->UAVProcessMatchesList(strList,8,false,strMatch);
    EXPECT_EQ(0,err);

    err=match->UAVProcessMatchesList("",1,true,strMatch);
    EXPECT_EQ(1,err);
    err=match->UAVProcessMatchesList("",1,false,strMatch);
    EXPECT_EQ(1,err);
    err=match->UAVProcessMatchesList(strList,1,true,"");
    EXPECT_EQ(1,err);
    err=match->UAVProcessMatchesList(strList,1,false,"");
    EXPECT_EQ(1,err);
}

TEST(UAVUnitTest,UnitTestFeature)
{
    std::string sfm_data="/home/wuwei/Data/UAVData/1/sfm.json";
    std::string dFeature="/home/wuwei/Data/UAVData/1/feats/";
    std::string strMatchList="/home/wuwei/Data/UAVData/1/matches.txt";
    std::string strMatch="/home/wuwei/Data/UAVData/1/feats/matches.txt";

    UAVProcessFeatureSIFT *feats=new UAVProcessFeatureSIFT();
    UAVErr err=0;
    err=feats->UAVProcessFeatExtract(false);
    EXPECT_EQ(err,5);
    err=feats->UAVProcessFeatList(sfm_data,dFeature);
    EXPECT_EQ(0,err);
    //err=feats->UAVProcessFeatExtract(false);
    //EXPECT_EQ(0,err);
    err=feats->UAVProcessFeatExtract(true);
    EXPECT_EQ(0,err);

    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
    EXPECT_EQ(0,err);
}

TEST(UAVUniTest,UnitTestFeatureErr)
{
    std::string sfm_data="/home/wuwei/Data/UAVData/1/sfm.json";
    std::string dFeature="/home/wuwei/Data/UAVData/1/feats/";
    std::string strMatchList="/home/wuwei/Data/UAVData/1/matches.txt";
    std::string strMatch="/home/wuwei/Data/UAVData/1/feats/matches.txt";

    UAVProcessFeatureSIFT *feats=new UAVProcessFeatureSIFT();
    UAVErr err=0;
    err=feats->UAVProcessFeatExtract(true);
    EXPECT_EQ(err,1);

    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
    EXPECT_EQ(err,1);

}