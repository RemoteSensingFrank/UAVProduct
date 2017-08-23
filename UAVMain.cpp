#include<stdlib.h>
#include<stdio.h>
#include "gtest/gtest.h"
#include "UAVBundle.h"
#include "UAVCommon.h"
#include "UAVGeoProc.h"
#include "UAVDataList.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "UAVGeoMosaic.h"


int main(int argc,char* argv[])
{
	//测试环境的初始化
    testing::GTEST_FLAG(output) = "xml:TestReport.xml";
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
