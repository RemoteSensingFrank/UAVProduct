//
// Created by wuwei on 17-11-5.
//

#include <sfm/pipelines/localization/SfM_Localizer.hpp>
#include <features/features.hpp>
#include "UAVProcImplement.h"
#include "openMVG/system/timer.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include "openMVG/types.hpp"

UAVProcFeature::UAVProcFeature() {
    param = new UAVParamFeature();
}
UAVProcFeature::~UAVProcFeature() {

}


UAVErr UAVProcFeature::UAVProcFeatureList(std::string feature_dir)
{
    if(param==NULL)
    {
        return 4;
    }
    this->UAVProcListFeature(feature_dir,*param);
    return 0;
}

UAVErr UAVProcFeature::UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel)
{
    int featureNums = this->param->_param_features_.size();
    if(featureNums==0)
        return 1;

    double step = 1.0/double(featureNums-1);
    bool bForce = false;
    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer;
    std::string sOutDir = stlplus::folder_part(param->_param_features_[0]._feature_out_);

    const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
    if (!bForce && stlplus::is_file(sImage_describer))
    {
        // Dynamically load the image_describer from the file (will restore old used settings)
        std::ifstream stream(sImage_describer.c_str());
        if (!stream.is_open())
            return false;

        try
        {
            cereal::JSONInputArchive archive(stream);
            archive(cereal::make_nvp("image_describer", image_describer));
        }
        catch (const cereal::Exception & e)
        {
            std::cerr << e.what() << std::endl
                      << "Cannot dynamically allocate the Image_describer interface." << std::endl;
            return false;
        }
    }
    else
    {
        // Create the desired Image_describer method.
        // Don't use a factory, perform direct allocation
        image_describer.reset(new openMVG::features::SIFT_Image_describer
                                          (openMVG::features::SIFT_Image_describer::Params(), true));
        if (!image_describer)
        {
            return 4;
        }
        // Export the used Image_describer and region type for:
        // - dynamic future regions computation and/or loading
        {
            std::ofstream stream(sImage_describer.c_str());
            if (!stream.is_open())
                return false;

            cereal::JSONOutputArchive archive(stream);
            archive(cereal::make_nvp("image_describer", image_describer));
            std::unique_ptr<Regions> regionsType;
            image_describer->Allocate(regionsType);
            archive(cereal::make_nvp("regions_type", regionsType));
        }
    }

    openMVG::system::Timer timer;
    openMVG::image::Image<unsigned char> imageGray, globalMask;

    const std::string sGlobalMask_filename = stlplus::create_filespec(sOutDir, "mask.png");
    if (stlplus::file_exists(sGlobalMask_filename))
    {
        if (ReadImage(sGlobalMask_filename.c_str(), &globalMask))
        {
            std::cout
                    << "Feature extraction will use a GLOBAL MASK:\n"
                    << sGlobalMask_filename << std::endl;
        }
    }

    for(size_t i = 0; i < param->_param_features_.size(); ++i)
    {
        const std::string
                sView_filename = (param->_param_features_[i]._image_in_),
                sFeat = (param->_param_features_[i]._feature_out_),
                sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

        //If features or descriptors file are missing, compute them
        if (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))
        {
            if (!ReadImage(sView_filename.c_str(), &imageGray))
                continue;

            openMVG::image::Image<unsigned char> * mask = nullptr; // The mask is null by default
            const std::string sImageMask_filename =
                    stlplus::create_filespec(this->_list_params_->_path_imagedir_,
                                             stlplus::basename_part(sView_filename) + "_mask", "png");

            openMVG::image::Image<unsigned char> imageMask;
            if (stlplus::file_exists(sImageMask_filename))
                ReadImage(sImageMask_filename.c_str(), &imageMask);

            // The mask point to the globalMask, if a valid one exists for the current image
            if (globalMask.Width() == imageGray.Width() && globalMask.Height() == imageGray.Height())
                mask = &globalMask;
            // The mask point to the imageMask (individual mask) if a valid one exists for the current image
            if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
                mask = &imageMask;

            // Compute features and descriptors and export them to files
            std::unique_ptr<Regions> regions;
            image_describer->Describe(imageGray, regions, mask);
            image_describer->Save(regions.get(), sFeat, sDesc);
        }
        if(pfnProgress!=NULL)
            pfnProgress(step*(i+1),"",pProgressArg);
    }
    std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
    return 0;
}