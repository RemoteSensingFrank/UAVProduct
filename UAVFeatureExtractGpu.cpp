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
#include "openMVG/matching_image_collection/Pair_Builder.hpp"

#include <stdlib.h>
#include <vector>
#include <iostream>
using std::vector;
using std::iostream;
using namespace openMVG;
using namespace openMVG::sfm;

enum EPairMode
{
    PAIR_EXHAUSTIVE = 0,
    PAIR_CONTIGUOUS = 1,
    PAIR_FROM_FILE  = 2
};

bool UAVExportFeatsToFile(string pathfeats,string pathdesc, vector<SiftGPU::SiftKeypoint> feats, vector<float> desc) {

    string feats_path=pathfeats;
    string descs_path=pathdesc;

    ofstream ofs_feats(feats_path,ios_base::binary);
    ofstream ofs_descs(descs_path,ios_base::binary);

    if(!ofs_feats.is_open()||!ofs_descs.is_open())
        return false;
    int num = feats.size();
    ofs_feats.write((const char*)&num,sizeof(int));
    ofs_descs.write((const char*)&num,sizeof(int));
    for (int i = 0; i < num; ++i) {
        float x = feats[i].x;
        float y = feats[i].y;
        float descf[128];
        for(int j=0;j<128;++j)
        {
            descf[j]=desc[128*i+j];
        }
        ofs_feats.write((const char*)&x,sizeof(float));
        ofs_feats.write((const char*)&y,sizeof(float));
        ofs_feats.write((const char*)descf,sizeof(float)*128);
    }
    ofs_feats.close();
    ofs_descs.close();
    return true;
}

bool UAVImportFeatsToFile(string pathdesc, vector<float> &desc)
{
    ifstream ifs(pathdesc,ios_base::binary|ios_base::in);
    if(!ifs.is_open())
        return false;

    int num;
    ifs.read((char*)&num,sizeof(int));
    try
    {
        float* descf=new float[128*num];
        ifs.read((char*)descf,sizeof(float)*num*128);

        desc.resize(128*num);
        memcpy(desc,descf,sizeof(float)*num*128);

        delete[]descf;descf=NULL;
    }catch (std::bad_alloc e)
    {
        printf(e.what());
        return false;
    }
}

bool UAVExportMatchesToFile(string path,int matchnum,int (*matches)[2])
{
    ofstream ofs(path,ios_base::app);
    ofs<<matchnum<<endl;
    for(int i=0;i<matchnum;++i){
        ofs<<matches[i][0]
    }
    return true;
}



bool UAVFeatsSIFTGpu::UAVFeatsExtract() {
    std::string sOutDir = _info_._g_feature_dir_;
    SiftGPU *sift = new SiftGPU;
    if (sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        return false;

    //通过GPU进行解析
    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }
    {
        openMVG::system::Timer timer;
        //获取影像路径
        string root = sfm_data.s_root_path;

        for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i) {
            Views::const_iterator iterViews = sfm_data.views.begin();
            std::advance(iterViews, i);
            const View *view = iterViews->second.get();
            const std::string
                    sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
                    sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "gpufeats"),
                    sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "gpudesc");

            vector<float> descriptor(1);
            vector<SiftGPU::SiftKeypoint> keys(1);

            if (sift->RunSIFT(sView_filename.c_str())) {
                //get feature count
                int num1 = sift->GetFeatureNum();
                //allocate memory
                keys.resize(num1);
                descriptor.resize(128 * num1);
                sift->GetFeatureVector(&keys[0], &descriptor[0]);
                //this can be used to write your own sift file.
                //将特征点和特征描述写入文件中
                UAVExportFeatsToFile(sFeat,sDesc, keys, descriptor);
            }
        }
        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
    }
    delete sift;
    return true;
}

bool UAVFeatsSIFTGpu::UAVMatchesExtract() {

    std::string feature_path = _info_._g_feature_dir_;
    std::string match_path = _info_._g_match_dir_;

    SiftMatchGPU *matcher = new SiftMatchGPU(4096);
    matcher->VerifyContextGL();

    //通过GPU进行解析
    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }
    int iMatchingVideoMode = -1;
    std::string sPredefinedPairList = stlplus::create_filespec(_info_._g_auxiliary_dir,"MatchNeighbor.txt");
    std::string sNearestMatchingMethod = "AUTO";
    EPairMode ePairmode = (iMatchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;
    if(stlplus::file_exists(sPredefinedPairList))
    {
        ePairmode = PAIR_FROM_FILE;
    }

    system::Timer timer;
    {
        Pair_Set pairs;
        Pair_Set pairs_filter;
        switch (ePairmode)
        {
            case PAIR_EXHAUSTIVE: pairs = exhaustivePairs(sfm_data.GetViews().size()); break;
            case PAIR_CONTIGUOUS: pairs = contiguousWithOverlap(sfm_data.GetViews().size(), iMatchingVideoMode); break;
            case PAIR_FROM_FILE:
                if(!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs))
                {
                    return false;
                }
                break;
        }
        int matchpairs=pairs.size();
        for ( const auto & cur_pair : pairs )
        {
            int img1 = cur_pair.first,
                img2 = cur_pair.second;

            //load feats and describes;
            string name1 = sfm_data.GetViews().at(img1)->s_Img_path;
            string name2 = sfm_data.GetViews().at(img2)->s_Img_path;

            vector<float> desc1,desc2;
            if(!UAVImportFeatsToFile(name1,desc1)||!UAVImportFeatsToFile(name2,desc2));
            {
                continue;
            }
            int num1 = desc1.size()/128;
            int num2 = desc2.size()/128;
            //matches
            matcher->SetDescriptors(0, num1, &desc1[0]); //image 1
            matcher->SetDescriptors(1, num2, &desc2[0]); //image 2
            int (*match_buf)[2] = new int[num1][2];
            int num_match = matcher->GetSiftMatch(num1, match_buf);
            for(int i  = 0; i < num_match; ++i)
            {
                //How to get the feature matches:
                //SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
                //SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
                //key1 in the first image matches with key2 in the second image
            }

        }


    }


    return true;
}