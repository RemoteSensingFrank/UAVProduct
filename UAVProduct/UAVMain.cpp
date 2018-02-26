#include<stdlib.h>
#include<stdio.h>
#include "UAVPreProcess.h"
#include "UAVBundler.h"
#include "UAVDenseProcess.h"
#include "UAVSeamFinder.h"

int sample_odm_data_aukerman() {
        std::string image_dir="/home/wuwei/Data/Raw/odm_data_aukerman/images/";
        std::string sfm_data="/home/wuwei/Data/Proc/odm_data_aukerman/sfm.json";
        std::string dFeature="/home/wuwei/Data/Proc/odm_data_aukerman/features/";
        std::string strMatchList="/home/wuwei/Data/Proc/odm_data_aukerman/matche_list.txt";
        std::string strMatch="/home/wuwei/Data/Proc/odm_data_aukerman/features/matches.txt";
        std::string bunder_out = "/home/wuwei/Data/Proc/odm_data_aukerman/bunder.bin";
        std::string trans_mvs = "/home/wuwei/Data/Proc/odm_data_aukerman/trans.mvs";
        std::string densemvs = "/home/wuwei/Data/Proc/odm_data_aukerman/dense.ply";


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
                                    "",posSimple,CoordinateXYZ);

        err=feats->UAVProcessFeatList(sfm_data,dFeature);
        err=feats->UAVProcessFeatExtract();
        printf("finished features extract\n");
        err=match->UAVProcessMatchesList(sfm_data,0,false,strMatchList);
        err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
        printf("finished features matches\n");
        err=bundler->UAVProcessBundleGlobal(feats,strMatch,sfm_data,bunder_out);
        printf("finished bundle\n");
        err=bundler->UAVProcessBundleToMVS(bunder_out,trans_mvs);
        err=dense->UAVDP_MVSProc(trans_mvs,densemvs);
        return err;
}

int main(int argc,char* argv[])
{
    UAVSeamVoronoiFinder seamFinder;
    seamFinder.UAVDistanceTransTest("/home/wuwei/Pictures/test.bmp","/home/wuwei/Pictures/dt.tif");
    //return sample_odm_data_aukerman();
    //testing::InitGoogleTest(&argc, argv);
    //return RUN_ALL_TESTS();
}

