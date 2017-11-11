//
// Created by wuwei on 17-11-6.
//

#include <memory>
#include "UAVPreProcess.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/matcher_brute_force.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/sfm/sfm.hpp"
#include <cereal/archives/json.hpp>
#ifdef DEBUG
#define LOG( format, args... )  printf( format, ##args )
#else
#define LOG( format, args... ) NULL;
#endif

UAVErr UAVProcessPOS::UAVProcessPOSExtractXYZ(double &centerx, double &centery, double &centerz) {
    size_t size = posList.size();
    if(size==0)
        return 1;

    POSPair::iterator iter;
    centerx=centery=centerz=0;
    for( iter=posList.begin(); iter!=posList.end(); iter++)
    {
        openMVG::Vec3 xyz=UAVProcessGeometry::UAVProcessGeoBLHToXYZ(iter->second.dB, iter->second.dL, iter->second.dH);
        iter->second.dB=xyz(0);
        iter->second.dL=xyz(1);
        iter->second.dH=xyz(2);
        centerx += xyz(0);
        centery += xyz(1);
        centerz += xyz(2);
    }
    centerx/=double(size);
    centery/=double(size);
    centerz/=double(size);

    return 0;
}

UAVErr UAVProcessPOS::UAVProcessPOSExtractUTM(double &centerx, double &centery, double &centerz) {
    size_t size = posList.size();
    if(size==0)
        return 1;

    POSPair::iterator iter;
    centerx=centery=centerz=0;
    for( iter=posList.begin(); iter!=posList.end(); iter++)
    {
        openMVG::Vec3 utm=UAVProcessGeometry::UAVProcessGeoLatLonToUTM(iter->second.dB, iter->second.dL, iter->second.dH);
        iter->second.dB=utm(0);
        iter->second.dL=utm(1);
        iter->second.dH=utm(2);
        centerx += utm(0);
        centery += utm(1);
        centerz += utm(2);
    }
    centerx/=double(size);
    centery/=double(size);
    centerz/=double(size);



    return 0;
}

UAVErr UAVProcessPOS::UAVProcessPOSExtractLocal(double &centerx, double &centery, double &centerz) {
    size_t size = posList.size();
    if(size==0)
        return 1;

    POSPair::iterator iter;
    centerx=centery=centerz=0;
    for( iter=posList.begin(); iter!=posList.end(); iter++)
    {
        openMVG::Vec3 utm=UAVProcessGeometry::UAVProcessGeoLatLonToUTM(iter->second.dB, iter->second.dL, iter->second.dH);
        iter->second.dB=utm(0);
        iter->second.dL=utm(1);
        iter->second.dH=utm(2);
        centerx += utm(0);
        centery += utm(1);
        centerz += utm(2);
    }
    centerx/=double(size);
    centery/=double(size);
    centerz/=double(size);

    for( iter=posList.begin(); iter!=posList.end(); iter++)
    {
        //openMVG::Vec3 utm=UAVProcessGeometry::UAVProcessGeoLatLonToUTM(iter->second.dB, iter->second.dL, iter->second.dH);
        iter->second.dB-=centerx;
        iter->second.dL-=centery;
        //iter->second.dH-=centerz;
    }

    return 0;
}

UAVErr UAVProcessPOSSimple::UAVPorcessPOSGet(std::string file, bool bGps) {
    UAVErr err = 0;
    //整理数据获取影像数和POS数据行数
    if(!bGps)
    {
        std::string image_dir = file;
        if ( !stlplus::folder_exists( image_dir) )
        {
            return 1;
        }
        std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
        std::sort(vec_image.begin(), vec_image.end());
        std::unique_ptr<openMVG::exif::Exif_IO> exifReader(new openMVG::exif::Exif_IO_EasyExif);
        for(int i=0;i<vec_image.size();++i)
        {
            if( exifReader->open( stlplus::create_filespec(image_dir,vec_image[i]) ) && exifReader->doesHaveExifInfo())
            {
                double latitude, longitude, altitude;
                if ( exifReader->GPSLatitude( &latitude ) &&
                     exifReader->GPSLongitude( &longitude ) &&
                     exifReader->GPSAltitude( &altitude ) )
                {
                    UAVPOSSt tmpPos;
                    tmpPos.dB=latitude;tmpPos.dL=longitude;tmpPos.dH=altitude;
                    tmpPos.dRoll=tmpPos.dPitch=tmpPos.dHeading=0;
                    posList.insert(std::make_pair(i+1,tmpPos));
                } else{
                    return 1;
                }
            }
        }
        return 0;
    } else{
        if(!stlplus::file_exists(file))
            return 1;
        else{
            FILE* ptrfile=NULL;
            ptrfile = fopen(file.c_str(),"r+");
            if(ptrfile==NULL)
                return 1;
            char posline[2048];
            int num=1;
            while(!feof(ptrfile))
            {
                UAVPOSSt tmpPos;
                fgets(posline,2048,ptrfile);
                sscanf(posline,"%lf%lf%lf%lf%lf%lf",&tmpPos.dL,&tmpPos.dB,&tmpPos.dH,&tmpPos.dRoll,&tmpPos.dPitch,&tmpPos.dHeading);
                posList.insert(std::make_pair(num,tmpPos));
                num++;
            };
            fclose(ptrfile);
            ptrfile=NULL;
            return 0;
        }
    }

}

//TODO:
//POS数据导出的功能暂时先不做
UAVErr UAVProcessPOSSimple::UAVProcessExport(std::string file, bool rLoc) {
    return 0;
}

UAVErr UAVProcessList::UAVProcessListGet(std::string dImage, std::string pPos, UAVCalibParams &cParam,std::string sfm_out,
                                         UAVProcessPOS *pPorc, CoordiListType typeCoordi) {
    //检查数据
    //特征点解算列表
    std::string image_dir = dImage;
    if(!stlplus::folder_exists(image_dir))
    {
        return 1;
    }
    std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
    std::sort(vec_image.begin(), vec_image.end());

    using namespace openMVG::sfm;
    std::string sfm_in = sfm_out;
    SfM_Data sfm_data;
    sfm_data.s_root_path =  image_dir;
    Views & views = sfm_data.views;
    Intrinsics & intrinsics = sfm_data.intrinsics;

    //获取POS数据
    bool bPos=true;
    if(stlplus::file_exists(pPos))
    {
        bPos=pPorc->UAVPorcessPOSGet(pPos,true);
    } else{
        bPos=pPorc->UAVPorcessPOSGet(dImage,false);
    }
    double X,Y,Z;
    if(!bPos){
        switch (typeCoordi)
        {
            case CoordinateXYZ:
                pPorc->UAVProcessPOSExtractXYZ(X,Y,Z);
                break;
            case CoordinateUTM:
                pPorc->UAVProcessPOSExtractUTM(X,Y,Z);
                break;
            case CoordinateLocal:
                pPorc->UAVProcessPOSExtractLocal(X,Y,Z);
                break;
            default:
                bPos=true;
        }
    }
    bool bCalibParam=IsUndefineCalibParam(cParam);
    double width,height,ppx,ppy,focal;
    if(!bPos&&pPorc->posList.size()<vec_image.size())
        return 1;
    POSPair::iterator iter = pPorc->posList.begin();
    int i=0;
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
          iter_image != vec_image.end(); ++iter_image,++i)
    {
        const std::string sImageFilename = stlplus::create_filespec(image_dir, vec_image[i]);
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        openMVG::image::ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue;

        width = imgHeader.width;
        height = imgHeader.height;
        if(bCalibParam)
        {

            ppx=cParam._ppx_;
            ppy=cParam._ppy_;
            focal=std::max(cParam._flen_x_,cParam._flen_y_);
        } else{
            ppx=width/2.0;
            ppy=height/2.0;
            std::unique_ptr<openMVG::exif::Exif_IO> exifReader(new openMVG::exif::Exif_IO_EasyExif);
            exifReader->open( sImageFilename );
            focal = std::max ( width, height ) * exifReader->getFocal() / exifReader->getFocal();
        }

        std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic(NULL);
        if(focal>0&&ppx>0&& ppy&&height>0&&width>0)
        {
            //initial intrinsic
            intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>
                    (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        }

        if(bPos)
        {
            ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);
            // Add intrinsic related to the image (if any)
            if (intrinsic == NULL)
            {
                v.id_intrinsic = openMVG::UndefinedIndexT;
            }
            else
            {
                intrinsics[v.id_intrinsic] = intrinsic;
            }
            views[v.id_view] = std::make_shared<ViewPriors>(v);
        }
        else
        {

            ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);
            // Add intrinsic related to the image (if any)
            if (intrinsic == NULL)
            {
                v.id_intrinsic = openMVG::UndefinedIndexT;
            }
            else
            {
                intrinsics[v.id_intrinsic] = intrinsic;
            }
            v.b_use_pose_center_ = true;
            double x = iter->second.dL;
            double y = iter->second.dB;
            double z = iter->second.dH;
            v.pose_center_ = openMVG::Vec3(x,y,z);
            views[v.id_view] = std::make_shared<ViewPriors>(v);
            iter++;
        }
    }
    if (!openMVG::sfm::Save(
            sfm_data,
            sfm_in,
            ESfM_Data(VIEWS|INTRINSICS)))
    {
        return 3;
    }
    openMVG::sfm::SfM_Data sfm_datatest;
    if (!openMVG::sfm::Load(sfm_datatest, "/home/wuwei/Data/UAVData/10/sfm.json", openMVG::sfm::ESfM_Data(VIEWS|INTRINSICS)))
    {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << sfm_in << "\" cannot be read." << std::endl;
        return 1;
    }
    return 0;

}
//matches
UAVErr UAVProcessMatches::UAVProcessMatchesList(std::string imageList, int neighbor_count, bool bGeo,
                                                std::string pMatch) {
    std::string imgLst=imageList;
    std::string matchList=pMatch;
    int i_neighbor_count = neighbor_count;
    MATCHMODEL i_mode=PAIR_MODE_EXHAUSTIVE;

    //首先check输入
    if(!stlplus::file_exists(imgLst)||pMatch.empty())
        return 1;
    if(bGeo&&neighbor_count==0)
        return 1;

    if (i_neighbor_count==0)
        i_mode = PAIR_MODE_EXHAUSTIVE;
    else if (i_neighbor_count>=1&&!bGeo)
        i_mode = PAIR_MODE_CONTIGUOUS;
    else if (i_neighbor_count>=1&&bGeo)
        i_mode = PAIR_MODE_NEIGHBORHOOD;

    openMVG::sfm::SfM_Data sfm_data;
    if (!Load(sfm_data, imgLst, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS)))
    {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << imgLst << "\" cannot be read." << std::endl;
        return 1;
    }
    LOG("Loaded a sfm_data scene with:\n#views: %d\n",sfm_data.GetViews().size());
    // out file


    //---------------------------------------
    // a. List the view pose as a linear sequence of ids.
    // b. Establish a pose graph according the user chosen mode:
    //    - E => upper diagonal pairs,
    //    - V => list the N closest pose ids,
    //    - G => list the N closest poses XYZ position.
    // c. Convert the pose graph edges to a view graph
    // d. Export the view graph to a file and a SVG adjacency list
    //---------------------------------------
    std::multimap<openMVG::IndexT, openMVG::IndexT> pose_id_toViewId;
    std::set<openMVG::IndexT> set_poses;

    // a. Get nodes of the pose graph as a linear sequence
    for (const auto & viewIter : sfm_data.GetViews())
    {
        const openMVG::sfm::View * v = viewIter.second.get();
        assert (viewIter.first == v->id_view);
        pose_id_toViewId.insert( std::make_pair(v->id_pose, v->id_view) );
        set_poses.insert(v->id_pose);
    }
    const std::vector<openMVG::IndexT> vec_poses(set_poses.begin(), set_poses.end());


    // b. Create the pose graph pair relationship
    MatchesList pose_pairs;

    switch (i_mode)
    {
        case PAIR_MODE_EXHAUSTIVE:
            //所有节点全部连接 连接矩阵的上半部分全为1
            pose_pairs = openMVG::exhaustivePairs(sfm_data.GetViews().size());
            break;
        case PAIR_MODE_CONTIGUOUS:
            //连续的几张影像有重叠
            pose_pairs = openMVG::contiguousWithOverlap(vec_poses.size(), i_neighbor_count);
            break;
        case PAIR_MODE_NEIGHBORHOOD:
        {
            // List the poses priors 根据POS计算相邻n个邻域范围内的影像
            std::vector<openMVG::Vec3> vec_pose_centers;
            std::map<openMVG::IndexT, openMVG::IndexT> contiguous_to_pose_id;
            std::set<openMVG::IndexT> used_pose_ids;
            for (const auto & view_it : sfm_data.GetViews())
            {
                const openMVG::sfm::ViewPriors * prior = dynamic_cast<openMVG::sfm::ViewPriors*>(view_it.second.get());
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
                return 1;
            }
            // Compute i_neighbor_count neighbor(s) for each pose
            size_t contiguous_pose_id = 0;
            for (const openMVG::Vec3 pose_it : vec_pose_centers)
            {
                openMVG::matching::ArrayMatcherBruteForce<double> matcher;
                if (matcher.Build(vec_pose_centers[0].data(), vec_pose_centers.size(), 3))
                {
                    const double * query = pose_it.data();

                    openMVG::matching::IndMatches vec_indices;
                    std::vector<double> vec_distance;
                    const int NN = i_neighbor_count + 1; // since itself will be found
                    if (matcher.SearchNeighbours(query, 1, &vec_indices, &vec_distance, NN))
                    {
                        for (size_t i = 1; i < vec_indices.size(); ++i)
                        {
                            openMVG::IndexT idxI = contiguous_to_pose_id.at(contiguous_pose_id);
                            openMVG::IndexT idxJ = contiguous_to_pose_id.at(vec_indices[i].j_);
                            if (idxI > idxJ)
                                std::swap(idxI, idxJ);
                            pose_pairs.insert(openMVG::Pair(idxI, idxJ));
                        }
                    }
                }
                ++contiguous_pose_id;
            }
        }
            break;
        default:
            std::cerr << "Unknown pair mode." << std::endl;
            return 4;
    }


    // c. Convert the pose graph to a view graph
    MatchesList view_pair;
    for (const auto & pose_pair : pose_pairs)
    {
        const openMVG::IndexT poseA = pose_pair.first;
        const openMVG::IndexT poseB = pose_pair.second;
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
                            openMVG::Pair(std::min(view_id_a->second, view_id_b->second),
                                 std::max(view_id_a->second, view_id_b->second)));
                }
            }
        }
    }

    if (view_pair.empty())
    {
        LOG("Warning: The computed pair list is empty...!");
    }

    // d. Export the view graph to a file and a SVG adjacency list

    UAVProcessAdjacencyMatrixToSVG(sfm_data.GetViews().size(), view_pair,
                         stlplus::create_filespec(
                                 stlplus::folder_part(pMatch),
                                 stlplus::filename_part(pMatch), "svg"));

    if (openMVG::savePairs(pMatch, view_pair))
    {
        LOG("Exported %d view pairs\n from a view graph that have %d relative pose pairs.\n" ,view_pair.size(),pose_pairs.size());
        return 0;
    }

    return 4;

}
void UAVProcessMatches::UAVProcessAdjacencyMatrixToSVG(const size_t NbImages,
                const openMVG::Pair_Set & corresponding_indexes,
                const std::string & sOutName)
{
    using namespace svg;
    if (!corresponding_indexes.empty())
    {
        const float scaleFactor = 5.0f;
        svgDrawer svgStream((NbImages+3)*5, (NbImages+3)*5);
        // List possible pairs
        for (openMVG::IndexT I = 0; I < NbImages; ++I)
        {
            for (openMVG::IndexT J = 0; J < NbImages; ++J)
            {
                // If the pair have matches display a blue boxes at I,J position.
                const auto iterSearch = corresponding_indexes.find(std::make_pair(I,J));
                if (iterSearch != corresponding_indexes.end())
                {
                    svgStream.drawSquare(J*scaleFactor, I*scaleFactor, scaleFactor/2.0f,
                                         svgStyle().fill("blue").noStroke());
                }
            }
        }
        // Display axes with 0 -> NbImages annotation : _|
        std::ostringstream osNbImages;
        osNbImages << NbImages;
        svgStream.drawText((NbImages+1)*scaleFactor, scaleFactor, scaleFactor, "0", "black");
        svgStream.drawText((NbImages+1)*scaleFactor,
                           (NbImages)*scaleFactor - scaleFactor, scaleFactor, osNbImages.str(), "black");
        svgStream.drawLine((NbImages+1)*scaleFactor, 2*scaleFactor,
                           (NbImages+1)*scaleFactor, (NbImages)*scaleFactor - 2*scaleFactor,
                           svgStyle().stroke("black", 1.0));

        svgStream.drawText(scaleFactor, (NbImages+1)*scaleFactor, scaleFactor, "0", "black");
        svgStream.drawText((NbImages)*scaleFactor - scaleFactor,
                           (NbImages+1)*scaleFactor, scaleFactor, osNbImages.str(), "black");
        svgStream.drawLine(2*scaleFactor, (NbImages+1)*scaleFactor,
                           (NbImages)*scaleFactor - 2*scaleFactor, (NbImages+1)*scaleFactor,
                           svgStyle().stroke("black", 1.0));

        std::ofstream svgFileStream(sOutName.c_str());
        svgFileStream << svgStream.closeSvgFile().str();
    }
}

UAVErr UAVProcessMatches::UAVProcessMatchesExport(MatchesList list, std::string pMatch) {
    if (openMVG::savePairs(pMatch, list))
    {
        LOG("Exported %d matches failed\n");
        return 0;
    }
    return 4;
}

UAVErr UAVProcessMatches::UAVProcessMatchesImport(MatchesList &list, std::string pMatch) {
    return 0;
}