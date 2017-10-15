//
// Created by wuwei on 17-8-30.
//

#include "UAVICPProc.h"
#include "SiftGPU/SiftGPU.h"

#include "openMVG/sfm/sfm.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include "openMVG/stl/stl.hpp"

#include "third_party/progress/progress.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::image;


using namespace std;
//
bool UAVICPExtract::UAVICPExtractMatchesEnvi(string img1,string img2,string  match,EXTRACT_TYPE type)
{
    if(type==EXTRACT_CPU)
    {
        Image<unsigned char> imageL, imageR;
        ReadImage(img1.c_str(), &imageL);
        ReadImage(img2.c_str(), &imageR);

        //--
        // Detect regions thanks to an image_describer
        //--
        using namespace openMVG::features;
        std::unique_ptr<Image_describer> image_describer(new SIFT_Image_describer);
        std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;
        image_describer->Describe(imageL, regions_perImage[0]);
        image_describer->Describe(imageR, regions_perImage[1]);

        const PointFeatures
                featsL = regions_perImage.at(0)->GetRegionsPositions(),
                featsR = regions_perImage.at(1)->GetRegionsPositions();

        const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
        const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

        //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
        std::vector<IndMatch> vec_PutativeMatches;
        {
            // Find corresponding points
            matching::DistanceRatioMatch(
                    0.8, matching::BRUTE_FORCE_L2,
                    *regions_perImage.at(0).get(),
                    *regions_perImage.at(1).get(),
                    vec_PutativeMatches);


            ofstream file1;
            file1.open(match, ios::out);
            if (!file1.is_open())
                return false;
            file1 << "; ENVI Image to Image GCP File" << endl;
            file1 << "; base file: " << img1.c_str() << endl;
            file1 << "; warp file: " << img2.c_str() << endl;
            file1 << "; Base Image (x,y), Warp Image (x,y)" << endl;
            file1 << ";" << endl;

            for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
                //Get back linked feature, draw a circle and link them by a line
                const SIOPointFeature L = regionsL->Features()[vec_PutativeMatches[i].i_];
                const SIOPointFeature R = regionsR->Features()[vec_PutativeMatches[i].j_];

                file1 << setw(10) << L.x() << setw(10) << L.y()
                      << setw(10) << R.x() << setw(10) << R.y() << endl;

            }
            if (file1.is_open())
                file1.close();
            return true;
        }
    } else if(type= EXTRACT_GPU){
        SiftMatchGPU *matcher = new SiftMatchGPU(4096);
        SiftGPU *sift = new SiftGPU;
        if (sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
            return false;
        matcher->VerifyContextGL();

        int num1,num2;
        vector<float> descriptor1(1);
        vector<SiftGPU::SiftKeypoint> keys1(1);
        if (sift->RunSIFT(img1.c_str())) {
            //get feature count
            num1 = sift->GetFeatureNum();
            //allocate memory
            keys1.resize(num1);
            descriptor1.resize(128 * num1);
            sift->GetFeatureVector(&keys1[0], &descriptor1[0]);
        }
        vector<float> descriptor2(1);
        vector<SiftGPU::SiftKeypoint> keys2(1);
        if (sift->RunSIFT(img2.c_str())) {
            //get feature count
            num2 = sift->GetFeatureNum();
            //allocate memory
            keys2.resize(num2);
            descriptor2.resize(128 * num2);
            sift->GetFeatureVector(&keys2[0], &descriptor2[0]);
        }

        matcher->SetDescriptors(0, num1, &descriptor1[0]); //image 1
        matcher->SetDescriptors(1, num2, &descriptor2[0]); //image 2
        printf("%d   %d",num1,num2);
        int (*match_buf)[2] = new int[num1][2];
        int num_match = matcher->GetSiftMatch(num1, match_buf);

        ofstream file1;
        file1.open(match, ios::out);
        if (!file1.is_open())
            return false;
        file1 << "; ENVI Image to Image GCP File" << endl;
        file1 << "; base file: " << img1.c_str() << endl;
        file1 << "; warp file: " << img2.c_str() << endl;
        file1 << "; Base Image (x,y), Warp Image (x,y)" << endl;
        file1 << ";" << endl;
        for (int i = 0; i < num_match; ++i) {
            file1 << setw(10) << keys1[match_buf[i][0]].x << setw(10) << keys2[match_buf[i][0]].y
                  << setw(10) << keys1[match_buf[i][1]].x << setw(10) << keys2[match_buf[i][1]].y << endl;
        }
        file1.close();
        delete[]match_buf[2];
    }

}

bool UAVICPImportICPs(string pathICPs,string pathSFM)
{
    //读取SFM数据，无控制点的
    //通过GPU进行解析
    SfM_Data sfm_data;
    if (!Load(sfm_data,pathSFM, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << pathSFM << "\" cannot be read." << std::endl;
        return false;
    }

    //自定义像控点格式
    //像控点以二进制方式存储
    //首先读取像控点
    FILE* fs=fopen(pathICPs.c_str(),"rb+");
    if(fs==NULL)
        return false;
    //获取像控点数目
    int gcps ;
    fread(&gcps,sizeof(int),1,fs);
    vector<Vec3f> ground_control_points;
    Views views = sfm_data.GetViews();
    Landmarks control_points = sfm_data.GetControl_Points();

    for(int i=0;i<gcps;++i)
    {
        double xyz[3];
        fread(xyz,sizeof(double),3,fs);
        Vec3f pnt(xyz[0],xyz[1],xyz[2]);
        ground_control_points.push_back(pnt);
        control_points[i].X << xyz[0], xyz[1], xyz[2];
    }


    int count_views=-1;
    for(int i=0;i<gcps;++i)
    {
        fread(&count_views,sizeof(int),1,fs);
        for(int j=0;i<count_views;++j)
        {
            int imgst[3];
            fread(imgst,sizeof(int),3,fs);
            const View * view = views[imgst[0]].get();
            //Vec2f pntimg = Vec2f((float)imgst[1],(float)imgst[2]);
            control_points[i].obs[j].x<<imgst[1],imgst[2];
            control_points[i].obs[j].id_feat=j;
        }
    }
    if (!Save(
            sfm_data,
            pathSFM.c_str(),
            ESfM_Data(ALL)))
    {
        return false;
    }

    return true;

}