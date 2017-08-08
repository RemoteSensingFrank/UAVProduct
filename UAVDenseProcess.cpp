//
// Created by wuwei on 17-8-7.
//
#define _USE_EIGEN
#include "UAVDMatch.h"

#include "UAVDenseProcess.h"

#include "UAVCommon.h"
#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"

using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG;

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>
#include <string>
using namespace std;

void UAVDenseProcess::UAVDP_ExportMVS() {

    //输出文件的路径
    string export_mvs = stlplus::create_filespec(_info_._g_point_cloud_dir,"trans",".mvs");
    string sfm_data_path = stlplus::create_filespec(_info_._g_point_cloud_dir,"sfm_data",".bin");
    string mvs_dir = stlplus::create_filespec(_info_._g_point_cloud_dir,"mvs_img");

    // Create undistorted images directory structure
    if (!stlplus::is_folder(mvs_dir))
    {
        stlplus::folder_create(mvs_dir);
        if (!stlplus::is_folder(mvs_dir))
        {
            std::cerr << "Cannot access to one of the desired output directory" << std::endl;
            return ;
        }
    }

    //整理成输出文件
    SfM_Data sfm_data;
    if (!Load(sfm_data, sfm_data_path, ESfM_Data(ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sfm_data_path << "\" cannot be read." << std::endl;
        return ;
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
            const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());
            if (map_intrinsic.count(intrinsic.first) == 0)
                map_intrinsic.insert(std::make_pair(intrinsic.first, scene.platforms.size()));
            MVSInterface::Interface::Platform platform;
            // add the camera
            MVSInterface::Interface::Platform::Camera camera;
            camera.K = cam->K();
            // sub-pose
            camera.R = Mat3::Identity();
            camera.C = Vec3::Zero();
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
                Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
                ReadImage(srcImage.c_str(), &imageRGB);
                UndistortImage(imageRGB, cam, imageRGB_ud, BLACK);
                WriteImage(image.name.c_str(), imageRGB_ud);
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
        const Landmark & landmark = vertex.second;
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
            ImageHeader imageHeader;
            ReadImageHeader(pImage->name.c_str(), &imageHeader);
            const double fScale(1.0/std::max(imageHeader.width, imageHeader.height));
            camera.K(0, 0) *= fScale;
            camera.K(1, 1) *= fScale;
            camera.K(0, 2) *= fScale;
            camera.K(1, 2) *= fScale;
        }
    }

    // write OpenMVS data
    if (!ARCHIVEInterface::SerializeSave(scene, export_mvs))
        return ;

    std::cout
            << "Scene saved to OpenMVS interface format:\n"
            << "\t" << scene.images.size() << " images (" << nPoses << " calibrated)\n"
            << "\t" << scene.vertices.size() << " Landmarks\n";
    return ;
}
