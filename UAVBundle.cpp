//
// Created by wuwei on 17-7-31.
//

#include "UAVBundle.h"
#include "UAVCommon.h"

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/system/timer.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

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


bool UAVBundle::UAVBundleGlobal() {
    //method
    int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
    int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);

    //判断方法是否正确
    if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
        iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
        std::cerr << "\n Rotation averaging method is invalid" << std::endl;
        return false;
    }

    //cameras::Intrinsic_Parameter_Type;
    //调整所有的参数
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = Intrinsic_Parameter_Type::ADJUST_ALL;

    if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
        iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
        std::cerr << "\n Translation averaging method is invalid" << std::endl;
        return false;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data,_info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }

    //features and matches
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(_info_._g_match_dir_, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        std::cerr << "Invalid: "
                  << sImage_describer << " regions type file." << std::endl;
        return false;
    }

    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_, regions_type)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return false;
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
        return false;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return false;
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

        return false;
    }
    return true;
}

bool UAVBundle::UAVBundleGlobalGpu() {
    //method
    int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
    int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);

    //判断方法是否正确
    if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
        iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
        std::cerr << "\n Rotation averaging method is invalid" << std::endl;
        return false;
    }

    //cameras::Intrinsic_Parameter_Type;
    //调整所有的参数
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = Intrinsic_Parameter_Type::ADJUST_ALL;

    if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
        iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
        std::cerr << "\n Translation averaging method is invalid" << std::endl;
        return false;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data,_info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }

    // Features reading
    std::shared_ptr<Features_Provider_Gpu> feats_provider = std::make_shared<Features_Provider_Gpu>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return false;
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
        return false;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return false;
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

        return true;
    }
    return false;
}

bool UAVBundle::UAVBundleSequence() {
    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(_info_._g_feature_dir_, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        std::cerr << "Invalid: "
                  << sImage_describer << " regions type file." << std::endl;
        return false;
    }
    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_, regions_type)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return EXIT_FAILURE;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if // Try to read the two matches file formats
            (
            !(matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.f.txt")) ||
              matches_provider->load(sfm_data, stlplus::create_filespec(_info_._g_match_dir_, "matches.f.bin")))
            )
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return false;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return false;
    }

    if (!stlplus::folder_exists(_info_._g_point_cloud_dir))
    {
        if (!stlplus::folder_create(_info_._g_point_cloud_dir))
        {
            std::cerr << "\nCannot create the output directory" << std::endl;
            return false;
        }
    }

    //---------------------------------------
    // Sequential reconstruction process
    //---------------------------------------
    openMVG::system::Timer timer;
    SequentialSfMReconstructionEngine sfmEngine(
            sfm_data,
            _info_._g_point_cloud_dir,
            stlplus::create_filespec(_info_._g_point_cloud_dir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =Intrinsic_Parameter_Type::ADJUST_ALL;
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.SetUnknownCameraType(EINTRINSIC(PINHOLE_CAMERA_RADIAL3));
    bool b_use_motion_priors = _info_._g_Has_Pos;
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // Handle Initial pair parameter
    //不设初始匹配对，通过程序解算初始匹配对
    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

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

        return true;
    }
    return false;
}
bool UAVBundle::UAVBundleSequenceGpu() {
    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return false;
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    // Features reading
    std::shared_ptr<Features_Provider_Gpu> feats_provider = std::make_shared<Features_Provider_Gpu>();
    if (!feats_provider->load(sfm_data, _info_._g_feature_dir_)) {
        std::cerr << std::endl
                  << "Invalid features." << std::endl;
        return EXIT_FAILURE;
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
        return false;
    }


    if (_info_._g_point_cloud_dir.empty())  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return false;
    }

    if (!stlplus::folder_exists(_info_._g_point_cloud_dir))
    {
        if (!stlplus::folder_create(_info_._g_point_cloud_dir))
        {
            std::cerr << "\nCannot create the output directory" << std::endl;
            return false;
        }
    }

    //---------------------------------------
    // Sequential reconstruction process
    //---------------------------------------
    openMVG::system::Timer timer;
    SequentialSfMReconstructionEngine sfmEngine(
            sfm_data,
            _info_._g_point_cloud_dir,
            stlplus::create_filespec(_info_._g_point_cloud_dir, "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feats_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());

    // Configure reconstruction parameters
    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =Intrinsic_Parameter_Type::ADJUST_ALL;
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.SetUnknownCameraType(EINTRINSIC(PINHOLE_CAMERA_RADIAL3));
    bool b_use_motion_priors = _info_._g_Has_Pos;
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // Handle Initial pair parameter
    //不设初始匹配对，通过程序解算初始匹配对
    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

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

        return true;
    }
    return false;
}