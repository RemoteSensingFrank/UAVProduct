#include<stdlib.h>
#include<stdio.h>
//./UAVProcess#include "gtest/gtest.h"
#include "UAVPreProcess.h"
#include "UAVBundler.h"

int main(int argc,char* argv[])
{
    std::string sfm_data="/home/wuwei/Data/UAVData/1/sfm.json";
    std::string dFeature="/home/wuwei/Data/UAVData/1/feats/";
    std::string strMatchList="/home/wuwei/Data/UAVData/1/matches.txt";
    std::string strMatch="/home/wuwei/Data/UAVData/1/feats/matches.txt";
    std::string bunder_out = "/home/wuwei/Data/UAVData/1/bunder.bin";

    std::shared_ptr< UAVProcessFeatureSIFTGpu> feats=std::make_shared<UAVProcessFeatureSIFTGpu>();
    std::unique_ptr<UAVProcessMatches> match(new UAVProcessMatches());
    std::unique_ptr<UAVProcessBundle> bundler(new UAVProcessBundle());

    UAVErr err=0;
    err=feats->UAVProcessFeatList(sfm_data,dFeature);
    err=feats->UAVProcessFeatExtract(true);
    err=match->UAVProcessMatchesList(sfm_data,3,false,strMatchList);
    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
    err=bundler->UAVProcessBundleSquence(feats,strMatch,sfm_data,bunder_out);

    return err;
    //testing::InitGoogleTest(&argc, argv);
    //return RUN_ALL_TESTS();
}

