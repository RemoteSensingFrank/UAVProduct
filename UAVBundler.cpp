//
// Created by wuwei on 17-11-20.
//
#define _USE_EIGEN
#include "openMVG/system/timer.hpp"
#include "openMVG/image/image_io.hpp"
#include "UAVBundler.h"
#include "UAVDMatch.h"

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


UAVErr UAVProcessBundle::UAVProcessBundleToMVS(std::string sfm, std::string mvs) {

    //输出文件的路径
    string export_mvs = mvs;//stlplus::create_filespec(_info_._g_point_cloud_dir,"trans",".mvs");
    string sfm_data_path = sfm;//stlplus::create_filespec(_info_._g_point_cloud_dir,"sfm_data",".bin");
    string mvs_dir = stlplus::folder_part(mvs);

    // Create undistorted images directory structure
    if (!stlplus::is_folder(mvs_dir))
    {
        stlplus::folder_create(mvs_dir);
        if (!stlplus::is_folder(mvs_dir))
        {
            std::cerr << "Cannot access to one of the desired output directory" << std::endl;
            return 8;
        }
    }

    //整理成输出文件
    openMVG::sfm::SfM_Data sfm_data;
    if (!Load(sfm_data, sfm_data_path, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sfm_data_path << "\" cannot be read." << std::endl;
        return 8;
    }

    // Export data :
    MVSInterface::Interface scene;
    size_t nPoses(0);
    const uint32_t nViews((uint32_t)sfm_data.GetViews().size());

    C_Progress_display my_progress_bar(nViews);

    // OpenMVG can have not contiguous index, use a map to create the required OpenMVS contiguous ID index
    std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

    // define a platform with all the intrinsic group
    for (const auto& intrinsic: sfm_data.GetIntrinsics())
    {
        if (isPinhole(intrinsic.second.get()->getType()))
        {
            const openMVG::cameras::Pinhole_Intrinsic * cam = dynamic_cast<const openMVG::cameras::Pinhole_Intrinsic*>(intrinsic.second.get());
            if (map_intrinsic.count(intrinsic.first) == 0)
                map_intrinsic.insert(std::make_pair(intrinsic.first, scene.platforms.size()));
            MVSInterface::Interface::Platform platform;
            // add the camera
            MVSInterface::Interface::Platform::Camera camera;
            camera.K = cam->K();
            // sub-pose
            camera.R = openMVG::Mat3::Identity();
            camera.C = openMVG::Vec3::Zero();
            platform.cameras.push_back(camera);
            scene.platforms.push_back(platform);
        }
    }

    // define images & poses
    scene.images.reserve(nViews);
    for (const auto& view : sfm_data.GetViews())
    {
        map_view[view.first] = scene.images.size();
        MVSInterface::Interface::Image image;
        const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view.second->s_Img_path);
        image.name = stlplus::create_filespec(mvs_dir, view.second->s_Img_path);
        image.platformID = map_intrinsic.at(view.second->id_intrinsic);
        MVSInterface::Interface::Platform& platform = scene.platforms[image.platformID];
        image.cameraID = 0;
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()) && stlplus::is_file(srcImage))
        {
            MVSInterface::Interface::Platform::Pose pose;
            image.poseID = platform.poses.size();
            const openMVG::geometry::Pose3 poseMVG(sfm_data.GetPoseOrDie(view.second.get()));
            pose.R = poseMVG.rotation();
            pose.C = poseMVG.center();
            // export undistorted images
            const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view.second->id_intrinsic).get();
            if (cam->have_disto())
            {
                // undistort image and save it
                openMVG::image::Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
                openMVG::image::ReadImage(srcImage.c_str(), &imageRGB);
                UndistortImage(imageRGB, cam, imageRGB_ud, openMVG::image::BLACK);
                openMVG::image::WriteImage(image.name.c_str(), imageRGB_ud);
            }
            else
            {
                // just copy image
                stlplus::file_copy(srcImage, image.name);
            }
            platform.poses.push_back(pose);
            ++nPoses;
        }
        else
        {
            // image have not valid pose, so set an undefined pose
            image.poseID = NO_ID;
            // just copy the image
            stlplus::file_copy(srcImage, image.name);
        }
        scene.images.push_back(image);
        ++my_progress_bar;
    }

    // define structure
    scene.vertices.reserve(sfm_data.GetLandmarks().size());
    for (const auto& vertex: sfm_data.GetLandmarks())
    {
        const openMVG::sfm::Landmark & landmark = vertex.second;
        MVSInterface::Interface::Vertex vert;
        MVSInterface::Interface::Vertex::ViewArr& views = vert.views;
        for (const auto& observation: landmark.obs)
        {
            const auto it(map_view.find(observation.first));
            if (it != map_view.end()) {
                MVSInterface::Interface::Vertex::View view;
                view.imageID = it->second;
                view.confidence = 0;
                views.push_back(view);
            }
        }
        if (views.size() < 2)
            continue;
        std::sort(
                views.begin(), views.end(),
                [] (const MVSInterface::Interface::Vertex::View& view0, const MVSInterface::Interface::Vertex::View& view1)
                {
                    return view0.imageID < view1.imageID;
                }
        );
        vert.X = landmark.X.cast<float>();
        scene.vertices.push_back(vert);
    }

    // normalize camera intrinsics
    for (size_t p=0; p<scene.platforms.size(); ++p)
    {
        MVSInterface::Interface::Platform& platform = scene.platforms[p];
        for (size_t c=0; c<platform.cameras.size(); ++c) {
            MVSInterface::Interface::Platform::Camera& camera = platform.cameras[c];
            // find one image using this camera
            MVSInterface::Interface::Image* pImage(NULL);
            for (MVSInterface::Interface::Image& image: scene.images)
            {
                if (image.platformID == p && image.cameraID == c && image.poseID != NO_ID)
                {
                    pImage = &image;
                    break;
                }
            }
            if (pImage == NULL)
            {
                std::cerr << "error: no image using camera " << c << " of platform " << p << std::endl;
                continue;
            }
            // read image meta-data
            openMVG::image::ImageHeader imageHeader;
            openMVG::image::ReadImageHeader(pImage->name.c_str(), &imageHeader);
            const double fScale(1.0/std::max(imageHeader.width, imageHeader.height));
            camera.K(0, 0) *= fScale;
            camera.K(1, 1) *= fScale;
            camera.K(0, 2) *= fScale;
            camera.K(1, 2) *= fScale;
        }
    }

    // write OpenMVS data
    if (!ARCHIVEInterface::SerializeSave(scene, export_mvs))
        return 8;

    std::cout
            << "Scene saved to OpenMVS interface format:\n"
            << "\t" << scene.images.size() << " images (" << nPoses << " calibrated)\n"
            << "\t" << scene.vertices.size() << " Landmarks\n";
    return 8;

}