//
// Created by wuwei on 17-11-5.
//
#include "gtest/gtest.h"
#include "UAVProcImplement.h"

TEST(UAVUnitTest,UnitTestList)
{
    UAVParam* param = new UAVParamList();
    ((UAVParamList*)param)->_sfm_data_in_="/home/wuwei/Data/UAVData/1/sfm_in.json";
    ((UAVParamList*)param)->_sfm_data_out_="/home/wuwei/Data/UAVData/1/sfm_out.json";
    ((UAVParamList*)param)->_gps_file_="/home/wuwei/Data/UAVData/1/EO.txt";
    ((UAVParamList*)param)->_path_imagedir_="/home/wuwei/Data/UAVData/1/Img/";
    ((UAVParamList*)param)->_calib_param_._flen_x_=-1;
    ((UAVParamList*)param)->_calib_param_._flen_y_=-1;
    ((UAVParamList*)param)->_calib_param_._ccd_size_=35;

    UAVProcess* proc = new UAVProcList();
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcParam(param),0);
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcProcess(NULL,NULL),0);

    EXPECT_EQ(((UAVProcList*)proc)->UAVProcParam(NULL),0);
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcProcess(NULL,NULL),1);

    ((UAVParamList*)param)->_sfm_data_in_="/home/wuwei/Data/UAVData/1/sfm_in.json";
    ((UAVParamList*)param)->_sfm_data_out_="/home/wuwei/Data/UAVData/1/sfm_out.json";
    ((UAVParamList*)param)->_gps_file_="/home/wuwei/Data/UAVData/1/EO.txt";
    ((UAVParamList*)param)->_path_imagedir_="/home/wuwei/Data/UAVData/1/Img/";
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcParam(param),0);
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcProcess(NULL,NULL),1);

    ((UAVParamList*)param)->_sfm_data_in_="/home/wuwei/Data/UAVData/1/sfm_in.json";
    ((UAVParamList*)param)->_sfm_data_out_="/home/wuwei/Data/UAVData/1/sfm_out.json";
    ((UAVParamList*)param)->_gps_file_="/home/wuwei/Data/UAVData/1/EO.txt";
    ((UAVParamList*)param)->_path_imagedir_="/home/wuwei/Data/UAVData/1/Img/";
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcParam(param),0);
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcProcess(NULL,NULL),0);

    ((UAVParamList*)param)->_sfm_data_in_="";
    ((UAVParamList*)param)->_sfm_data_out_="/home/wuwei/Data/UAVData/1/sfm_out.json";
    ((UAVParamList*)param)->_gps_file_="/home/wuwei/Data/UAVData/1/EO.txt";
    ((UAVParamList*)param)->_path_imagedir_="/home/wuwei/Data/UAVData/1/Img/";
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcParam(param),0);
    EXPECT_EQ(((UAVProcList*)proc)->UAVProcProcess(NULL,NULL),3);
}