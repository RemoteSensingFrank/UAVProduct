#include<stdlib.h>
#include<stdio.h>
//./UAVProcess#include "gtest/gtest.h"
#include "UAVPreProcess.h"
#include "UAVBundler.h"
#include "UAVDenseProcess.h"

int main(int argc,char* argv[])
{
    std::string image_dir="/home/yan/Desktop/ImageDataset_SceauxCastle/images/";
    std::string sfm_data="/home/yan/Desktop/ImageDataset_SceauxCastle/result/sfm.json";
    std::string dFeature="/home/yan/Desktop/ImageDataset_SceauxCastle/result/";
    std::string strMatchList="/home/yan/Desktop/ImageDataset_SceauxCastle/result/matche_list.txt";
    std::string strMatch="/home/yan/Desktop/ImageDataset_SceauxCastle/result/matches.txt";
    std::string bunder_out = "/home/yan/Desktop/ImageDataset_SceauxCastle/result/bunder.bin";
    std::string trans_mvs = "/home/yan/Desktop/ImageDataset_SceauxCastle/result/mvs/trans.mvs";
    std::string densemvs = "/home/yan/Desktop/ImageDataset_SceauxCastle/result/mvs/dense.ply";


    UAVErr err=0;

    UAVProcessList* list=new UAVProcessList();
    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();
    std::shared_ptr< UAVProcessFeatureSIFT> feats=std::make_shared<UAVProcessFeatureSIFT>();
    std::unique_ptr<UAVProcessMatches> match(new UAVProcessMatches());
    std::unique_ptr<UAVProcessBundle> bundler(new UAVProcessBundle());
    std::unique_ptr<UAVDenseProcess> dense(new UAVDenseProcess());

    UAVCalibParams calibCameras;
    UndefinedCalibParam(calibCameras);
    err=list->UAVProcessListGet(image_dir,sfm_data,
                                calibCameras,PINHOLE_CAMERA_RADIAL3,true,
                                "",posSimple,CoordinateUTM);

    err=feats->UAVProcessFeatList(sfm_data,dFeature);
    err=feats->UAVProcessFeatExtract();
    err=match->UAVProcessMatchesList(sfm_data,0,false,strMatchList);
    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
    err=bundler->UAVProcessBundleGlobal(feats,strMatch,sfm_data,bunder_out);
    err=bundler->UAVProcessBundleToMVS(bunder_out,trans_mvs);
    err=dense->UAVDP_MVSProc(trans_mvs,densemvs);
    return err;
    //testing::InitGoogleTest(&argc, argv);
    //return RUN_ALL_TESTS();
}

