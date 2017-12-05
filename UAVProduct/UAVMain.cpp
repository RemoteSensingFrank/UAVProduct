#include<stdlib.h>
#include<stdio.h>
//./UAVProcess#include "gtest/gtest.h"
#include "UAVPreProcess.h"
#include "UAVBundler.h"
#include "UAVDenseProcess.h"

int main(int argc,char* argv[])
{
    std::string image_dir="/home/yan/Desktop/odm_data_aukerman/images/";

    std::string sfm_data="/home/yan/Desktop/odm_data_aukerman/result/sfm.json";
    std::string dFeature="/home/yan/Desktop/odm_data_aukerman/result/";
    std::string strMatchList="/home/yan/Desktop/odm_data_aukerman/result/matche_list.txt";
    std::string strMatch="/home/yan/Desktop/odm_data_aukerman/result/matches.txt";
    std::string bunder_out = "/home/yan/Desktop/odm_data_aukerman/result/bunder.bin";
    // std::string trans_mvs = "/home/wuwei/Data/UAVData/1/mvs/trans.mvs";
    // std::string densemvs = "/home/wuwei/Data/UAVData/1/mvs/dense.ply";


    UAVErr err=0;

    UAVProcessList* list=new UAVProcessList();
    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();
    std::shared_ptr< UAVProcessFeatureSIFTGpu> feats=std::make_shared<UAVProcessFeatureSIFTGpu>();
    std::unique_ptr<UAVProcessMatches> match(new UAVProcessMatches());
    std::unique_ptr<UAVProcessBundle> bundler(new UAVProcessBundle());
    //std::unique_ptr<UAVDenseProcess> dense(new UAVDenseProcess());

    UAVCalibParams calibCameras;
    UndefinedCalibParam(calibCameras);
    err=list->UAVProcessListGet(image_dir,sfm_data,
                               calibCameras,PINHOLE_CAMERA_RADIAL3,true,
                                "",posSimple,CoordinateUTM);
    err=feats->UAVProcessFeatList(sfm_data,dFeature);
    err=feats->UAVProcessFeatExtract(true);
    err=match->UAVProcessMatchesList(sfm_data,0,false,strMatchList);
    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
    err=bundler->UAVProcessBundleGlobal(feats,strMatch,sfm_data,bunder_out);
    //err=bundler->UAVProcessBundleToMVS(bunder_out,trans_mvs);
    //err=dense->UAVDP_MVSProc(trans_mvs,densemvs);
    return err;
    //testing::InitGoogleTest(&argc, argv);
    //return RUN_ALL_TESTS();
}

