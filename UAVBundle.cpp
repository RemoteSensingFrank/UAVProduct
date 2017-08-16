//
// Created by wuwei on 17-7-31.
//

#include "UAVBundle.h"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;

/***
 * 为了保持风格的统一，需要集成至Features_Provider能够方便进行处理和解算,
 * 在密集匹配的过程中欧给你应该也需要进行处理才对
 */
struct Features_Provider_Gpu:public Features_Provider
{
    /***
     * 导入GPU解算得到的特征点
     * @param sfm_data
     * @param feat_directory
     * @return
     */
    virtual bool load(
            const SfM_Data & sfm_data,
            const std::string & feat_directory)
    {
        C_Progress_display my_progress_bar( sfm_data.GetViews().size(),
                                            std::cout, "\n- Features Loading -\n" );
        // Read for each view the corresponding features and store them as PointFeatures
        bool bContinue = true;

        //导入特征点的过程应该比较快，不需要通过多线程操作了
        for (Views::const_iterator iter = sfm_data.GetViews().begin();
             iter != sfm_data.GetViews().end() && bContinue; ++iter) {

            const std::string sImageName = stlplus::create_filespec(sfm_data.s_root_path, iter->second.get()->s_Img_path);
            const std::string basename = stlplus::basename_part(sImageName);
            const std::string featFile = stlplus::create_filespec(feat_directory, basename, ".gpufeats");

            //导入特征点
            //通过读取二进制文件的方式导入
            if (!stlplus::file_exists(featFile))
            {
                bContinue= false;
                return bContinue;
            }
            ifstream ifs(featFile,ios_base::in|ios_base::binary);
            int num_feats;
            ifs.read((char*)&num_feats,sizeof(int));
            float* points = new float[2*num_feats];
            ifs.read((char*)points,sizeof(float)*2*num_feats);
            features::PointFeatures vec_points;
            for (int i = 0; i < num_feats; ++i) {
                vec_points.push_back( features::PointFeature(points[2*i+0],points[2*i+1]));
            }
            feats_per_view[iter->second.get()->id_view] =  vec_points;
            delete[]points;points=NULL;
            ++my_progress_bar;
        }

        return bContinue;
    }
};


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

void UAVBundle::UAVBundleGlobal() {
    //method
    int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
    int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);

    //判断方法是否正确
    if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
        iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
        std::cerr << "\n Rotation averaging method is invalid" << std::endl;
        return;
    }

    //cameras::Intrinsic_Parameter_Type;
    //调整所有的参数
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = Intrinsic_Parameter_Type::ADJUST_ALL;

    if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
        iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
        std::cerr << "\n Translation averaging method is invalid" << std::endl;
        return;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data,_info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return ;
    }

    //features and matches
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
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_, regions_type)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return ;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if // Try to read the two matches file formats
            (
            !(matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.e.txt")) ||
              matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.e.bin")))
            )
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return ;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return ;
    }

    if (!stlplus::folder_exists(_info_._g_point_cloud_dir))
    {
        if (!stlplus::folder_create(_info_._g_point_cloud_dir))
        {
            std::cerr << "\nCannot create the output directory" << std::endl;
        }
    }


    //---------------------------------------
    // Global SfM reconstruction process
    //---------------------------------------

    openMVG::system::Timer timer;
    GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
            sfm_data,
            _info_._g_point_cloud_dir,
            stlplus::create_filespec(_info_._g_point_cloud_dir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    bool b_use_motion_priors = _info_._g_Has_Pos;
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // Configure motion averaging method
    sfmEngine.SetRotationAveragingMethod(
            ERotationAveragingMethod(iRotationAveragingMethod));
    sfmEngine.SetTranslationAveragingMethod(
            ETranslationAveragingMethod(iTranslationAveragingMethod));

    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

        std::cout << "...Generating SfM_Report.html" << std::endl;
        Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
                            stlplus::create_filespec(_info_._g_point_cloud_dir, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & visualizable results)
        std::cout << "...Export SfM_Data to disk." << std::endl;
        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(_info_._g_point_cloud_dir, "sfm_data", ".bin"),
             ESfM_Data(ALL));

        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(_info_._g_point_cloud_dir, "cloud_and_poses", ".ply"),
             ESfM_Data(ALL));

        return ;
    }
    return ;
}

void UAVBundle::UAVBundleGlobalGpu() {
    //method
    int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
    int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);

    //判断方法是否正确
    if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
        iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
        std::cerr << "\n Rotation averaging method is invalid" << std::endl;
        return;
    }

    //cameras::Intrinsic_Parameter_Type;
    //调整所有的参数
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = Intrinsic_Parameter_Type::ADJUST_ALL;

    if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
        iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
        std::cerr << "\n Translation averaging method is invalid" << std::endl;
        return;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data,_info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return ;
    }

    // Features reading
    std::shared_ptr<Features_Provider_Gpu> feats_provider = std::make_shared<Features_Provider_Gpu>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return ;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if // Try to read the two matches file formats
            (
            !(matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.e.txt")) ||
              matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.e.bin")))
            )
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return ;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return ;
    }

    if (!stlplus::folder_exists(_info_._g_point_cloud_dir))
    {
        if (!stlplus::folder_create(_info_._g_point_cloud_dir))
        {
            std::cerr << "\nCannot create the output directory" << std::endl;
        }
    }


    //---------------------------------------
    // Global SfM reconstruction process
    //---------------------------------------

    openMVG::system::Timer timer;
    GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
            sfm_data,
            _info_._g_point_cloud_dir,
            stlplus::create_filespec(_info_._g_point_cloud_dir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    bool b_use_motion_priors = _info_._g_Has_Pos;
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // Configure motion averaging method
    sfmEngine.SetRotationAveragingMethod(
            ERotationAveragingMethod(iRotationAveragingMethod));
    sfmEngine.SetTranslationAveragingMethod(
            ETranslationAveragingMethod(iTranslationAveragingMethod));

    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

        std::cout << "...Generating SfM_Report.html" << std::endl;
        Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
                            stlplus::create_filespec(_info_._g_point_cloud_dir, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & visualizable results)
        std::cout << "...Export SfM_Data to disk." << std::endl;
        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(_info_._g_point_cloud_dir, "sfm_data", ".bin"),
             ESfM_Data(ALL));

        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(_info_._g_point_cloud_dir, "cloud_and_poses", ".ply"),
             ESfM_Data(ALL));

        return ;
    }
    return ;
}