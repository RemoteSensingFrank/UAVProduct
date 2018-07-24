#include<stdlib.h>
#include<stdio.h>
#include <gdal.h>

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "UAVProcessGeometry.h"
#include "UAVMapCalculateTools.h"
#include "UAVPOSProcess.h"

//int sample_odm_data_aukerman() {
//        std::string image_dir="/home/wuwei/Data/Raw/odm_data_aukerman/images/";
//        std::string sfm_data="/home/wuwei/Data/Proc/odm_data_aukerman/sfm.json";
//        std::string dFeature="/home/wuwei/Data/Proc/odm_data_aukerman/features/";
//        std::string strMatchList="/home/wuwei/Data/Proc/odm_data_aukerman/matche_list.txt";
//        std::string strMatch="/home/wuwei/Data/Proc/odm_data_aukerman/features/matches.txt";
//        std::string bunder_out = "/home/wuwei/Data/Proc/odm_data_aukerman/bunder.bin";
//        std::string trans_mvs = "/home/wuwei/Data/Proc/odm_data_aukerman/trans.mvs";
//        std::string densemvs = "/home/wuwei/Data/Proc/odm_data_aukerman/dense.ply";
//        UAVErr err=0;
//        UAVProcessList* list=new UAVProcessList();
//        UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();
//        std::shared_ptr< UAVProcessFeatureSIFT> feats=std::make_shared<UAVProcessFeatureSIFT>();
//        std::unique_ptr<UAVProcessMatches> match(new UAVProcessMatches());
//        std::unique_ptr<UAVProcessBundle> bundler(new UAVProcessBundle());
//        std::unique_ptr<UAVDenseProcess> dense(new UAVDenseProcess());
//
//        UAVCalibParams calibCameras;
//        UndefinedCalibParam(calibCameras);
//        err=list->UAVProcessListGet(image_dir,sfm_data,
//                                    calibCameras,PINHOLE_CAMERA_RADIAL3,true,
//                                    "",posSimple,CoordinateXYZ);
//
//        err=feats->UAVProcessFeatList(sfm_data,dFeature);
//        err=feats->UAVProcessFeatExtract();
//        printf("finished features extract\n");
//        err=match->UAVProcessMatchesList(sfm_data,0,false,strMatchList);
//        err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
//        printf("finished features matches\n");
//        err=bundler->UAVProcessBundleGlobal(feats,strMatch,sfm_data,bunder_out);
//        printf("finished bundle\n");
//        err=bundler->UAVProcessBundleToMVS(bunder_out,trans_mvs);
//        err=dense->UAVDP_MVSProc(trans_mvs,densemvs);
//        return err;
//}
//
//int AgriCultural(){
//    std::string image_dir="/home/wuwei/Data/agriculture/Img/";
//    std::string sfm_data="/home/wuwei/Data/agriculture/sfm.json";
//    std::string dFeature="/home/wuwei/Data/agriculture/features/";
//    std::string strMatchList="/home/wuwei/Data/agriculture/matche_list.txt";
//    std::string strMatch="/home/wuwei/Data/agriculture/features/matches.txt";
//    std::string bunder_out = "/home/wuwei/Data/agriculture/bunder.bin";
//    std::string trans_mvs = "/home/wuwei/Data/agriculture/trans.mvs";
//    std::string densemvs = "/home/wuwei/Data/agriculture/dense.ply";
//
//    UAVErr err=0;
//    UAVProcessList* list=new UAVProcessList();
//    UAVProcessPOSSimple* posSimple = new UAVProcessPOSSimple();
//    std::shared_ptr< UAVProcessFeatureSIFT> feats=std::make_shared<UAVProcessFeatureSIFT>();
//    std::unique_ptr<UAVProcessMatches> match(new UAVProcessMatches());
//    std::unique_ptr<UAVProcessBundle> bundler(new UAVProcessBundle());
//    std::unique_ptr<UAVDenseProcess> dense(new UAVDenseProcess());
//
//    UAVCalibParams calibCameras;
//    UndefinedCalibParam(calibCameras);
//    err=list->UAVProcessListGet(image_dir,sfm_data,
//                                calibCameras,PINHOLE_CAMERA_RADIAL1,true,
//                                "",posSimple,CoordinateXYZ);
//
//    err=feats->UAVProcessFeatList(sfm_data,dFeature);
//    err=feats->UAVProcessFeatExtract();
//    printf("finished features extract\n");
//    err=match->UAVProcessMatchesList(sfm_data,0,false,strMatchList);
//    err=feats->UAVProcessMatchesExtract(strMatchList,strMatch);
//    printf("finished features matches\n");
//    err=bundler->UAVProcessBundleGlobal(feats,strMatch,sfm_data,bunder_out);
//    printf("finished bundle\n");
//    err=bundler->UAVProcessBundleToMVS(bunder_out,trans_mvs);
//    err=dense->UAVDP_MVSProc(trans_mvs,densemvs);
//    return err;
//}
//
//void correctDirectly(){
//        UAVPOSProcessExtract *posproc = new UAVPOSProcessExtract();
//        posproc->UAVPorcessPOSGet("/home/wuwei/Data/agriculture/Img/",false);
//        int num = posproc->posList.size();
//
//        Vec3f pA(0,0,0);
//        Vec3f pV(0,0,0);
//
//        UAVCalibParams calib;
//        calib._ppx_   = 2000;
//        calib._ppy_   = 1500;
//        calib._flen_x_= 4000;
//        calib._flen_y_= 3000;
//        calib._ccd_size_=1;
//
//        vector<openMVG::Mat34> vec_p;
//        posproc->UAVPOSProc_ExtractToP(vec_p,calib);
//        UAVProcessGeoCorrect geoCorrect;
//        std::vector<std::string> vec_image = stlplus::folder_files("/home/wuwei/Data/agriculture/Img/");
//        std::sort(vec_image.begin(), vec_image.end());
//        for(int i=0;i<vec_image.size();++i)
//        {
//            double dL = posproc->posList[i].dL;
//            double dB = posproc->posList[i].dB;
//            string images  = stlplus::create_filespec("/home/wuwei/Data/agriculture/Img/",vec_image[i]);
//            string pathGeo = stlplus::create_filespec("/home/wuwei/Data/agriculture/Geo/", stlplus::basename_part(vec_image[i]), "tif");
//            geoCorrect.UAVGeoCorrectExterior(images.c_str(),vec_p[i],40,pathGeo.c_str(),0.1,dL,dB);
//        }
//}
//


int main(int argc,char* argv[])
{
    UAVPOSProcessExtract extractPos;
    UAVProcessGeoCorrect correctGeo;
    vector<openMVG::Mat34> matrixP;
    UAVCalibParams calibCameras;
    calibCameras._ppx_ = 7360/2;
    calibCameras._ppy_ = 4912/2;
    calibCameras._flen_x_=7360;
    calibCameras._flen_y_=7360;
    calibCameras._ccd_size_=1;

    extractPos.UAVPorcessPOSGet("/home/wuwei/data/POS.txt",true);
    extractPos.UAVPOSProc_ExtractToP(matrixP,calibCameras);
    correctGeo.UAVGeoCorrectExterior("/home/wuwei/data/DSC00006.JPG",matrixP[0],0,"/home/wuwei/data/DSC00006.tif",0.5,113,29);
    return 0;
}