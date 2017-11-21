//
// Created by wuwei on 17-11-20.
//
#include "openMVG/system/timer.hpp"
#include "UAVBundler.h"

UAVErr UAVProcessBundle::UAVProcessBundleGlobal(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_out) {
    if(feats_ptr== nullptr)
        return 7;
    unique_ptr<openMVG::sfm::Features_Provider> feat_provider = feats_ptr->UAVProcessFeatsProvide();
    if(feat_provider== nullptr)
        return 7;

    openMVG::sfm::SfM_Data sfm_data;
    if (!Load(sfm_data,sfm_in,  openMVG::sfm::ESfM_Data(openMVG::sfm::ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sfm_in << "\" cannot be read." << std::endl;
        return false;
    }

    std::shared_ptr<openMVG::sfm::Matches_Provider> matches_provider = std::make_shared<openMVG::sfm::Matches_Provider>();
    if (!(matches_provider->load(sfm_data, mathces)))
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return false;
    }

    openMVG::system::Timer timer;
    openMVG::sfm::SequentialSfMReconstructionEngine sfmEngine(
            sfm_data,
            stlplus::folder_part(sfm_out),
            stlplus::create_filespec(stlplus::folder_part(sfm_out), "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feat_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());
    bool b_use_motion_priors=true;
    for(auto iter:sfm_data.GetViews()){
        //b_use_motion_priors=b_use_motion_priors&&
        const openMVG::sfm::View *viewT=iter.second.get();
        int i=0;
        b_use_motion_priors=b_use_motion_priors&((openMVG::sfm::ViewPriors*)viewT)->b_use_pose_center_;

    }
    // Configure reconstruction parameters
    const openMVG::cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);
    sfmEngine.SetUnknownCameraType(openMVG::cameras::EINTRINSIC(openMVG::cameras::PINHOLE_CAMERA_RADIAL3));

    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

        std::cout << "...Generating SfM_Report.html" << std::endl;
        Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
                            stlplus::create_filespec(sfm_out, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & visualizable results)
        std::cout << "...Export SfM_Data to disk." << std::endl;
        Save(sfmEngine.Get_SfM_Data(),
             sfm_out,
             openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));

        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(stlplus::folder_part(sfm_out), "cloud_and_poses", ".ply"),
             openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));

        return 0;
    }
    return 7;
}

UAVErr UAVProcessBundle::UAVProcessBundleSquence(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_out)
{
    if(feats_ptr== nullptr)
        return 7;
    unique_ptr<openMVG::sfm::Features_Provider> feat_provider = feats_ptr->UAVProcessFeatsProvide();
    if(feat_provider== nullptr)
        return 7;

    openMVG::sfm::SfM_Data sfm_data;
    if (!Load(sfm_data,sfm_in,  openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sfm_in << "\" cannot be read." << std::endl;
        return false;
    }

    std::shared_ptr<openMVG::sfm::Matches_Provider> matches_provider = std::make_shared<openMVG::sfm::Matches_Provider>();
    if (!(matches_provider->load(sfm_data, mathces)))
    {
        std::cerr << std::endl
                  << "Invalid matches file." << std::endl;
        return false;
    }

    openMVG::system::Timer timer;
    openMVG::sfm::GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
            sfm_data,
            stlplus::folder_part(sfm_out),
            stlplus::create_filespec(stlplus::folder_part(sfm_out), "Reconstruction_Report.html"));

    // Configure the features_provider & the matches_provider
    sfmEngine.SetFeaturesProvider(feat_provider.get());
    sfmEngine.SetMatchesProvider(matches_provider.get());
    bool b_use_motion_priors=true;
    for(auto iter:sfm_data.GetViews()){
        //b_use_motion_priors=b_use_motion_priors&&
        const openMVG::sfm::View *viewT=iter.second.get();
        int i=0;
        b_use_motion_priors=b_use_motion_priors&((openMVG::sfm::ViewPriors*)viewT)->b_use_pose_center_;

    }
    // Configure reconstruction parameters
    const openMVG::cameras::Intrinsic_Parameter_Type intrinsic_refinement_options = openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
    int iRotationAveragingMethod = int (openMVG::sfm::ROTATION_AVERAGING_L2);
    int iTranslationAveragingMethod = int (openMVG::sfm::TRANSLATION_AVERAGING_SOFTL1);
    sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

    // Configure motion averaging method
    sfmEngine.SetRotationAveragingMethod(
            openMVG::sfm::ERotationAveragingMethod(iRotationAveragingMethod));
    sfmEngine.SetTranslationAveragingMethod(
            openMVG::sfm::ETranslationAveragingMethod(iTranslationAveragingMethod));

    if (sfmEngine.Process())
    {
        std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

        std::cout << "...Generating SfM_Report.html" << std::endl;
        Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
                            stlplus::create_filespec(sfm_out, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & visualizable results)
        std::cout << "...Export SfM_Data to disk." << std::endl;
        Save(sfmEngine.Get_SfM_Data(),
             sfm_out,
             openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));

        Save(sfmEngine.Get_SfM_Data(),
             stlplus::create_filespec(stlplus::folder_part(sfm_out), "cloud_and_poses", ".ply"),
             openMVG::sfm::ESfM_Data(openMVG::sfm::ALL));

        return 7;
    }
    return 0;
}