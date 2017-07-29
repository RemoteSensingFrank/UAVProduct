//
// Created by wuwei on 17-7-29.
//
#include "UAVFeatureExtract.h"
#include "common.h"

#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/system/timer.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include "openMVG/sfm/sfm.hpp"

#include "openMVG/features/features.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <string>
#include <iostream>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::sfm;

using namespace std;
static  UAVXYZToLatLonWGS84 CooridinateTrans;

features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)
{
    features::EDESCRIBER_PRESET preset;
    if(sPreset == "NORMAL")
        preset = features::NORMAL_PRESET;
    else
    if (sPreset == "HIGH")
        preset = features::HIGH_PRESET;
    else
    if (sPreset == "ULTRA")
        preset = features::ULTRA_PRESET;
    else
        preset = features::EDESCRIBER_PRESET(-1);
    return preset;
}


bool UAVFeatsSIFT::UAVFeatsExtract() {
    std::string sSfM_Data_Filename=_info_._g_SFM_data;
    std::string sOutDir = _info_._g_feature_dir_;
    bool bUpRight = false;
    std::string sImage_Describer_Method = "SIFT";
    bool bForce = false;
    std::string sFeaturePreset = "";
#ifdef OPENMVG_USE_OPENMP
    int iNumThreads = 0;
#endif
    //---------------------------------------
    // a. Load input scene
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input file \""<< sSfM_Data_Filename << "\" cannot be read" << std::endl;
        return false;
    }

    // b. Init the image_describer
    // - retrieve the used one in case of pre-computed features
    // - else create the desired one

    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer;

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
        if (sImage_Describer_Method == "SIFT")
        {
            image_describer.reset(new SIFT_Image_describer
                                          (SIFT_Image_describer::Params(), !bUpRight));
        }
        if (!image_describer)
        {
            std::cerr << "Cannot create the designed Image_describer:"
                      << sImage_Describer_Method << "." << std::endl;
            return false;
        }
        else
        {
            if (!sFeaturePreset.empty())
                if (!image_describer->Set_configuration_preset(stringToEnum(sFeaturePreset)))
                {
                    std::cerr << "Preset configuration failed." << std::endl;
                    return false;
                }
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

    // Feature extraction routines
    // For each View of the SfM_Data container:
    // - if regions file exists continue,
    // - if no file, compute features
    {
        system::Timer timer;
        Image<unsigned char> imageGray, globalMask;

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

        C_Progress_display my_progress_bar( sfm_data.GetViews().size(),
                                            std::cout, "\n- EXTRACT FEATURES -\n" );

#ifdef OPENMVG_USE_OPENMP
        const unsigned int nb_max_thread = omp_get_max_threads();

    if (iNumThreads > 0) {
        omp_set_num_threads(iNumThreads);
    } else {
        omp_set_num_threads(nb_max_thread);
    }

    #pragma omp parallel for schedule(dynamic) if(iNumThreads > 0)
#endif
        for(int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
        {
            Views::const_iterator iterViews = sfm_data.views.begin();
            std::advance(iterViews, i);
            const View * view = iterViews->second.get();
            const std::string
                    sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
                    sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
                    sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

            //If features or descriptors file are missing, compute them
            if (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))
            {
                if (!ReadImage(sView_filename.c_str(), &imageGray))
                    continue;

                Image<unsigned char> * mask = nullptr; // The mask is null by default

                const std::string sImageMask_filename =
                        stlplus::create_filespec(sfm_data.s_root_path,
                                                 stlplus::basename_part(sView_filename) + "_mask", "png");

                Image<unsigned char> imageMask;
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
#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
            ++my_progress_bar;
        }
        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
    }
    return false;
}

bool UAVFeatsSIFT::UAVFeatsLoad() {

}