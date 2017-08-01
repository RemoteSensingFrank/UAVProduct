//
// Created by wuwei on 17-7-31.
//

#include "UAVBundle.h"
#include "openMVG/cameras/Camera_Pinhole.hpp"
using namespace openMVG::sfm;
using namespace openMVG::cameras;

void UAVBundle::UAVBundleTwoViewExtract(string pathRotMat) {
    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<<  _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return ;
    }

    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(_info_._g_match_dir_, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        std::cerr << "Invalid: "
                  << sImage_describer << " regions type file." << std::endl;
        return ;
    }

    // Features reading
    std::shared_ptr<Regions_Provider> feats_provider = std::make_shared<Regions_Provider>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_, regions_type)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return ;
    }
    // Matches reading
    PairWiseMatches map_PutativesMatches;
    if // Try to read the two matches file formats
    (
    !(!Load(map_PutativesMatches, stlplus::create_filespec(_info_._g_match_dir_, "matches.e.txt")) ||
      !Load(map_PutativesMatches,stlplus::create_filespec(_info_._g_match_dir_, "matches.e.bin")))
    )
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return ;
    }

    //K
    Mat3 K;
    K(0,0)=_info_._g_focal_x;K(0,1)=0;K(0,2)=_info_._g_ppx;
    K(1,0)=0;K(1,1)=_info_._g_focal_y;K(1,2)=_info_._g_ppx;
    K(2,0)=0;K(2,1)=0;K(2,2)=1;

    FILE* file=fopen(pathRotMat.c_str(),"wb+");
    //get matches
    Pair_Set pairs = getPairs(map_PutativesMatches);
    int totalPairs = 0;
    fwrite(&totalPairs,sizeof(int),1,file);
    for (Pair_Set::iterator iter = pairs.begin();iter!=pairs.end();++iter) {

        sfm::RelativePose_Info relativePose_info;
        IndexT idx1 = iter->first;
        IndexT idx2 = iter->second;

        IndMatches matches = map_PutativesMatches.find(std::make_pair(idx1,idx2))->second;


        const std::vector<PointFeature>
                featsL = feats_provider->get(idx1)->GetRegionsPositions(),
                featsR = feats_provider->get(idx2)->GetRegionsPositions();
        const View * vl = sfm_data.views.at(idx1).get();
        const View * vr = sfm_data.views.at(idx2).get();

        std::pair<size_t, size_t> size_imaL(vl->ui_width, vl->ui_height);
        std::pair<size_t, size_t> size_imaR(vr->ui_width, vr->ui_height);

        int sizepnts1 = featsL.size();
        int sizepnts2 = featsR.size();

        int matchNum = matches.size();
        Mat xL(2, matchNum);
        Mat xR(2, matchNum);
        for (size_t k = 0; k < matchNum; ++k)  {
            const PointFeature & imaL = featsL[matches.at(k).i_];
            const PointFeature & imaR = featsR[matches.at(k).j_];
            xL.col(k) = imaL.coords().cast<double>();
            xR.col(k) = imaR.coords().cast<double>();
        }

        //相对定向，得到R和T相对定向元素
        if (!sfm::robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaR, 256))
        {
            std::cerr << " /!\\ Robust relative pose estimation failure."
                      << std::endl;
            return ;
        }

        std::cout << "\nFound an Essential matrix:\n"
                  << "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
                  << "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
                  << "\t#matches: " << matchNum
                  << std::endl;

        //DLT计算三维点的坐标
        Pinhole_Intrinsic intrinsic0(vl->ui_width,  vl->ui_height, K(0, 0), K(0, 2), K(1, 2));
        Pinhole_Intrinsic intrinsic1(vr->ui_width,  vr->ui_height, K(0, 0), K(0, 2), K(1, 2));

        const Pose3 pose0 = Pose3(Mat3::Identity(), Vec3::Zero());
        const Pose3 pose1 = relativePose_info.relativePose;

        // Init structure by inlier triangulation
        const Mat34 P1 = intrinsic0.get_projective_equivalent(pose0);
        const Mat34 P2 = intrinsic1.get_projective_equivalent(pose1);
        fwrite(&P1,sizeof(double),12,file);
        fwrite(&P2,sizeof(double),12,file);

        std::vector<Vec3> vec_3DPoints;
        for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i)  {
            const PointFeature & LL = featsL[matches[relativePose_info.vec_inliers[i]].i_];
            const PointFeature & RR = featsR[matches[relativePose_info.vec_inliers[i]].j_];
            // Point triangulation
            Vec3 X;
            TriangulateDLT(P1, LL.coords().cast<double>(), P2, RR.coords().cast<double>(), &X);
            // Reject point that is behind the camera
            if (pose0.depth(X) < 0 && pose1.depth(X) < 0)
                continue;
            vec_3DPoints.emplace_back(X);   //DLT 3D points
        }
        int size = vec_3DPoints.size();
        fwrite(&size,sizeof(unsigned long),1,file);
        for(size_t i = 0; i < vec_3DPoints.size(); ++i)
        {
            fwrite(&vec_3DPoints[i],sizeof(Vec3),1,file);
        }
        totalPairs++;
    }
    fseek(file,0,SEEK_SET);
    fwrite(&totalPairs,sizeof(int),1,file);
    fclose(file);
}

