//
// Created by wuwei on 17-7-29.
//
#include "UAVFeatureExtract.h"
#include "common.h"
#include "UAVAuxiliary.h"

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/system/timer.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"

#include "openMVG/graph/graph.hpp"
#include "openMVG/stl/stl.hpp"

#include "openMVG/graph/graph.hpp"
#include "openMVG/stl/stl.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::image;
using namespace openMVG::matching_image_collection;

#include "third_party/progress/progress.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace std;
static  UAVXYZToLatLonWGS84 CooridinateTrans;


//列出影像匹配列表
enum ePairMode
{
    PAIR_MODE_EXHAUSTIVE = 0,
    PAIR_MODE_CONTIGUOUS = 1,
    PAIR_MODE_NEIGHBORHOOD = 2
};

enum EGeometricModel
{
    FUNDAMENTAL_MATRIX = 0,
    ESSENTIAL_MATRIX   = 1,
    HOMOGRAPHY_MATRIX  = 2
};

enum EPairMode
{
    PAIR_EXHAUSTIVE = 0,
    PAIR_CONTIGUOUS = 1,
    PAIR_FROM_FILE  = 2
};

bool UAVFeatureExtract::UAVMatchesList(int neighbor_count) {
    std::string s_SfM_Data_filename=_info_._g_SFM_data;
    std::string s_out_file=stlplus::create_filespec(_info_._g_auxiliary_dir,"MatchNeighbor.txt");
    int i_neighbor_count = neighbor_count;
    int i_mode(PAIR_MODE_EXHAUSTIVE);

    if (i_neighbor_count==0)
        i_mode = PAIR_MODE_EXHAUSTIVE;
    else if (i_neighbor_count==-1)
        i_mode = PAIR_MODE_CONTIGUOUS;
    else if (i_neighbor_count>2)
        i_mode = PAIR_MODE_NEIGHBORHOOD;

    // Input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, s_SfM_Data_filename, ESfM_Data(VIEWS|INTRINSICS)))
    {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << s_SfM_Data_filename << "\" cannot be read." << std::endl;
        return false;
    }

    std::cout
            << "Loaded a sfm_data scene with:\n"
            << " #views: " << sfm_data.GetViews().size() << "\n"
            << std::endl;

    // out file
    if (s_out_file.empty())
    {
        std::cerr << "Invalid output filename." << std::endl;
        return false;
    }

    //---------------------------------------
    // a. List the view pose as a linear sequence of ids.
    // b. Establish a pose graph according the user chosen mode:
    //    - E => upper diagonal pairs,
    //    - V => list the N closest pose ids,
    //    - G => list the N closest poses XYZ position.
    // c. Convert the pose graph edges to a view graph
    // d. Export the view graph to a file and a SVG adjacency list
    //---------------------------------------
    std::multimap<IndexT, IndexT> pose_id_toViewId;
    std::set<IndexT> set_poses;

    // a. Get nodes of the pose graph as a linear sequence
    for (const auto & viewIter : sfm_data.GetViews())
    {
        const View * v = viewIter.second.get();
        assert (viewIter.first == v->id_view);
        pose_id_toViewId.insert( std::make_pair(v->id_pose, v->id_view) );
        set_poses.insert(v->id_pose);
    }
    const std::vector<IndexT> vec_poses(set_poses.begin(), set_poses.end());


    // b. Create the pose graph pair relationship
    Pair_Set pose_pairs;

    switch (i_mode)
    {
        case PAIR_MODE_EXHAUSTIVE:
            //所有节点全部连接 连接矩阵的上半部分全为1
            pose_pairs = exhaustivePairs(sfm_data.GetViews().size());
            break;
        case PAIR_MODE_CONTIGUOUS:
            //连续的几张影像有重叠
            pose_pairs = contiguousWithOverlap(vec_poses.size(), i_neighbor_count);
            break;
        case PAIR_MODE_NEIGHBORHOOD:
        {
            // List the poses priors 根据POS计算相邻n个邻域范围内的影像
            std::vector<Vec3> vec_pose_centers;
            std::map<IndexT, IndexT> contiguous_to_pose_id;
            std::set<IndexT> used_pose_ids;
            for (const auto & view_it : sfm_data.GetViews())
            {
                const sfm::ViewPriors * prior = dynamic_cast<sfm::ViewPriors*>(view_it.second.get());
                if (prior != nullptr && prior->b_use_pose_center_ && used_pose_ids.count(prior->id_pose) == 0)
                {
                    vec_pose_centers.push_back( prior->pose_center_ );
                    contiguous_to_pose_id[contiguous_to_pose_id.size()] = prior->id_pose;
                    used_pose_ids.insert(prior->id_pose);
                }
            }
            if (vec_pose_centers.empty())
            {
                std::cerr << "You are trying to use the gps_mode but your data does"
                          << " not have any pose priors."
                          << std::endl;
            }
            // Compute i_neighbor_count neighbor(s) for each pose
            size_t contiguous_pose_id = 0;
            for (const Vec3 pose_it : vec_pose_centers)
            {
                matching::ArrayMatcherBruteForce<double> matcher;
                if (matcher.Build(vec_pose_centers[0].data(), vec_pose_centers.size(), 3))
                {
                    const double * query = pose_it.data();

                    IndMatches vec_indices;
                    std::vector<double> vec_distance;
                    const int NN = i_neighbor_count + 1; // since itself will be found
                    if (matcher.SearchNeighbours(query, 1, &vec_indices, &vec_distance, NN))
                    {
                        for (size_t i = 1; i < vec_indices.size(); ++i)
                        {
                            IndexT idxI = contiguous_to_pose_id.at(contiguous_pose_id);
                            IndexT idxJ = contiguous_to_pose_id.at(vec_indices[i].j_);
                            if (idxI > idxJ)
                                std::swap(idxI, idxJ);
                            pose_pairs.insert(Pair(idxI, idxJ));
                        }
                    }
                }
                ++contiguous_pose_id;
            }
        }
            break;
        default:
            std::cerr << "Unknown pair mode." << std::endl;
            return false;
    }


    // c. Convert the pose graph to a view graph
    Pair_Set view_pair;
    for (const auto & pose_pair : pose_pairs)
    {
        const IndexT poseA = pose_pair.first;
        const IndexT poseB = pose_pair.second;
        // get back the view related to those poses and create the pair (exhaustively)
        const auto range_a = pose_id_toViewId.equal_range(vec_poses[poseA]);
        for (auto view_id_a = range_a.first; view_id_a != range_a.second; view_id_a++)
        {
            const auto range_b = pose_id_toViewId.equal_range(vec_poses[poseB]);
            for (auto view_id_b = range_b.first; view_id_b != range_b.second; view_id_b++)
            {
                if (view_id_a != view_id_b)
                {
                    view_pair.insert(
                            Pair(std::min(view_id_a->second, view_id_b->second),
                                 std::max(view_id_a->second, view_id_b->second)));
                }
            }
        }
    }

    if (view_pair.empty())
    {
        std::cout << "Warning: The computed pair list is empty...!" << std::endl;
    }

    // d. Export the view graph to a file and a SVG adjacency list

    AdjacencyMatrixToSVG(sfm_data.GetViews().size(), view_pair,
                         stlplus::create_filespec(
                                 stlplus::folder_part(s_out_file),
                                 stlplus::filename_part(s_out_file), "svg"));

    if (savePairs(s_out_file, view_pair))
    {
        std::cout << "Exported " << view_pair.size() << " view pairs\n"
                  <<"from a view graph that have " << pose_pairs.size()
                  << " relative pose pairs." << std::endl;
        return false;
    }

    return true;
}

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

bool UAVFeatsSIFT::UAVMatchesExtract() {

    std::string sSfM_Data_Filename=_info_._g_SFM_data;
    std::string sMatchesDirectory = _info_._g_match_dir_;
    std::string sGeometricModel = "e";
    float fDistRatio = 0.6f;
    int iMatchingVideoMode = -1;
    std::string sPredefinedPairList = stlplus::create_filespec(_info_._g_auxiliary_dir,"MatchNeighbor.txt");
    std::string sNearestMatchingMethod = "AUTO";
    bool bForce = false;
    bool bGuided_matching = false;
    int imax_iteration = 2048;
    unsigned int ui_max_cache_size = 0;

    EPairMode ePairmode = (iMatchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    if (sPredefinedPairList.length()) {
        ePairmode = PAIR_FROM_FILE;
        if (iMatchingVideoMode>0) {
            std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
            return false;
        }
    }

    if (sMatchesDirectory.empty() || !stlplus::is_folder(sMatchesDirectory))  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return false;
    }

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    std::string sGeometricMatchesFilename = "";
    switch(sGeometricModel[0])
    {
        case 'f': case 'F':
            eGeometricModelToCompute = FUNDAMENTAL_MATRIX;  //通过基础矩阵进行约束
            sGeometricMatchesFilename = "matches.f.txt";
            break;
        case 'e': case 'E':
            eGeometricModelToCompute = ESSENTIAL_MATRIX;    //通过本质矩阵进行你约束
            sGeometricMatchesFilename = "matches.e.txt";
            break;
        case 'h': case 'H':
            eGeometricModelToCompute = HOMOGRAPHY_MATRIX;   //通过单应矩阵进行约束
            sGeometricMatchesFilename = "matches.h.txt";
            break;
        default:
            std::cerr << "Unknown geometric model" << std::endl;
            return false;
    }

    // -----------------------------
    // - Load SfM_Data Views & intrinsics data
    // a. Compute putative descriptor matches
    // b. Geometric filtering of putative matches
    // + Export some statistics
    // -----------------------------

    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return false;
    }

    //---------------------------------------
    // Load SfM Scene regions
    //---------------------------------------
    // Init the regions_type from the image describer file (used for image regions extraction)
    //导入特征点和特征点描述
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        std::cerr << "Invalid: "
                  << sImage_describer << " regions type file." << std::endl;
        return false;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //    - Descriptor matching (according user method choice)
    //    - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------

    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider;
    if (ui_max_cache_size == 0)
    {
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    }
    else
    {
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }

    if (!regions_provider->load(sfm_data, _info_._g_feature_dir_, regions_type)) {
        std::cerr << std::endl << "Invalid regions." << std::endl;
        return false;
    }

    PairWiseMatches map_PutativesMatches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t> > vec_imagesSize;
    {
        vec_fileNames.reserve(sfm_data.GetViews().size());
        vec_imagesSize.reserve(sfm_data.GetViews().size());
        for (Views::const_iterator iter = sfm_data.GetViews().begin();
             iter != sfm_data.GetViews().end();
             ++iter)
        {
            const View * v = iter->second.get();
            vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                             v->s_Img_path));
            vec_imagesSize.push_back( std::make_pair( v->ui_width, v->ui_height) );
        }
    }

    std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;
    // If the matches already exists, reload them
    if
            (
            !bForce
            && (stlplus::file_exists(sMatchesDirectory + "/matches.putative.txt")
                || stlplus::file_exists(sMatchesDirectory + "/matches.putative.bin"))
            )
    {
        if (!(Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.bin") ||
              Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.txt")) )
        {
            std::cerr << "Cannot load input matches file";
            return false;
        }
        std::cout << "\t PREVIOUS RESULTS LOADED;"
                  << " #pair: " << map_PutativesMatches.size() << std::endl;
    }
    else // Compute the putative matches
    {
        std::cout << "Use: ";
        switch (ePairmode)
        {
            case PAIR_EXHAUSTIVE: std::cout << "exhaustive pairwise matching" << std::endl; break;
            case PAIR_CONTIGUOUS: std::cout << "sequence pairwise matching" << std::endl; break;
            case PAIR_FROM_FILE:  std::cout << "user defined pairwise matching" << std::endl; break;
        }

        // Allocate the right Matcher according the Matching requested method
        std::unique_ptr<Matcher> collectionMatcher;
        if (sNearestMatchingMethod == "AUTO")
        {
            if (regions_type->IsScalar())
            {
                std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
                collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
            }
            else
            if (regions_type->IsBinary())
            {
                std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
                collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
            }
        }
        else
        if (sNearestMatchingMethod == "BRUTEFORCEL2")
        {
            std::cout << "Using BRUTE_FORCE_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_L2));
        }
        else
        if (sNearestMatchingMethod == "BRUTEFORCEHAMMING")
        {
            std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
        }
        else
        if (sNearestMatchingMethod == "ANNL2")
        {
            std::cout << "Using ANN_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, ANN_L2));
        }
        else
        if (sNearestMatchingMethod == "CASCADEHASHINGL2")
        {
            std::cout << "Using CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, CASCADE_HASHING_L2));
        }
        else
        if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2")
        {
            std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
        }
        if (!collectionMatcher)
        {
            std::cerr << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod << std::endl;
            return false;
        }
        // Perform the matching
        system::Timer timer;
        {
            // From matching mode compute the pair list that have to be matched:
            Pair_Set pairs;
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
            // Photometric matching of putative pairs
            collectionMatcher->Match(sfm_data, regions_provider, pairs, map_PutativesMatches);
            //---------------------------------------
            //-- Export putative matches
            //---------------------------------------
            if (!Save(map_PutativesMatches, std::string(sMatchesDirectory + "/matches.putative.bin")))
            {
                std::cerr
                        << "Cannot save computed matches in: "
                        << std::string(sMatchesDirectory + "/matches.putative.bin");
                return false;
            }
        }
        std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;
    }
    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                         map_PutativesMatches,
                                         stlplus::create_filespec(_info_._g_auxiliary_dir, "PutativeAdjacencyMatrix", "svg"));
    //-- export view pair graph once putative graph matches have been computed
    {
        std::set<IndexT> set_ViewIds;
        std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                       std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativesMatches));
        graph::exportToGraphvizData(
                stlplus::create_filespec(_info_._g_auxiliary_dir, "putative_matches"),
                putativeGraph);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //    - AContrario Estimation of the desired geometric model
    //    - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------

    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
            new ImageCollectionGeometricFilter(&sfm_data, regions_provider));

    if (filter_ptr)
    {
        system::Timer timer;
        std::cout << std::endl << " - Geometric filtering - " << std::endl;

        PairWiseMatches map_GeometricMatches;
        switch (eGeometricModelToCompute)
        {
            case HOMOGRAPHY_MATRIX:
            {
                const bool bGeometric_only_guided_matching = true;
                filter_ptr->Robust_model_estimation(GeometricFilter_HMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching,
                                                    bGeometric_only_guided_matching ? -1.0 : 0.6);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
            }
                break;
            case FUNDAMENTAL_MATRIX:
            {
                filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
            }
                break;
            case ESSENTIAL_MATRIX:
            {
                filter_ptr->Robust_model_estimation(GeometricFilter_EMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();

                //-- Perform an additional check to remove pairs with poor overlap
                std::vector<PairWiseMatches::key_type> vec_toRemove;
                for (PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
                     iterMap != map_GeometricMatches.end(); ++iterMap)
                {
                    const size_t putativePhotometricCount = map_PutativesMatches.find(iterMap->first)->second.size();
                    const size_t putativeGeometricCount = iterMap->second.size();
                    const float ratio = putativeGeometricCount / (float)putativePhotometricCount;
                    if (putativeGeometricCount < 50 || ratio < .3f)  {
                        // the pair will be removed
                        vec_toRemove.push_back(iterMap->first);
                    }
                }
                //-- remove discarded pairs
                for (std::vector<PairWiseMatches::key_type>::const_iterator
                             iter =  vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
                {
                    map_GeometricMatches.erase(*iter);
                }
            }
                break;
        }

        //---------------------------------------
        //-- Export geometric filtered matches
        //---------------------------------------
        if (!Save(map_GeometricMatches,
                  std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename)))
        {
            std::cerr
                    << "Cannot save computed matches in: "
                    << std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename);
            return false;
        }

        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;

        //-- export Adjacency matrix
        std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches"
                  << std::endl;
        PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                             map_GeometricMatches,
                                             stlplus::create_filespec(_info_._g_auxiliary_dir, "GeometricAdjacencyMatrix", "svg"));

        //-- export view pair graph once geometric filter have been done
        {
            std::set<IndexT> set_ViewIds;
            std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                           std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_GeometricMatches));
            graph::exportToGraphvizData(
                    stlplus::create_filespec(_info_._g_auxiliary_dir, "geometric_matches"),
                    putativeGraph);
        }
    }
    return true;
}
