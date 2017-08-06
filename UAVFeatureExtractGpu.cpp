//
// Created by wuwei on 17-8-6.
//
#include "UAVFeatureExtract.h"


#include "openMVG/sfm/sfm.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/system/timer.hpp"

#include <stdlib.h>
#include <vector>
#include <iostream>
using std::vector;
using std::iostream;
using namespace openMVG;
using namespace openMVG::sfm;

bool UAVFeatsSIFTGpu::UAVExportFeatsToFile(string path, vector<SiftGPU::SiftKeypoint>, vector<float> desc) {

    return true;
}

bool UAVFeatsSIFTGpu::UAVFeatsExtract() {

    std::string sOutDir = _info_._g_feature_dir_;
    SiftGPU  *sift = new SiftGPU;
    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        return false;

    //通过GPU进行解析
    SfM_Data sfm_data;
    if (!Load(sfm_data,_info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return ;
    }

    openMVG::system::Timer timer;
    //获取影像路径
    string root = sfm_data.s_root_path;
    for(int i=0;i<static_cast<int>(sfm_data.views.size());++i){
        Views::const_iterator iterViews = sfm_data.views.begin();
        std::advance(iterViews, i);
        const View * view = iterViews->second.get();
        const std::string
                sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
                sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
                sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

        vector<float >          descriptor(1);
        vector<SiftGPU::SiftKeypoint> keys(1);

        if(sift->RunSIFT(sView_filename))
        {
            //get feature count
            num1 = sift->GetFeatureNum();

            //allocate memory
            keys1.resize(num1);
            descriptors1.resize(128*num1);
            sift->GetFeatureVector(&keys[0], &descriptor[0]);
            //this can be used to write your own sift file.
            //将特征点和特征描述写入文件中
        }

    }


    return true;
}

bool UAVFeatsSIFTGpu::UAVMatchesExtract() {


    return true;
}