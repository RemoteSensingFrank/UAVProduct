//
// Created by wuwei on 17-7-26.
//

#include "UAVDataList.h"
#include "UAVCommon.h"

#include "openMVG/stl/split.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "UAVXYZToLatLonWGS84.h"

#include <string>
#include <vector>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include <kml/dom.h>
#include <kml/engine.h>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::image;
using namespace openMVG::exif;

static  UAVXYZToLatLonWGS84 CooridinateTrans;
static int get_file_size(const char* file) {
    struct stat tbuf;
    stat(file, &tbuf);
    return tbuf.st_size;
}
#define PI 3.1415926534

///////////////////////////////////////////////////////////////////////////////////////////////////////
pair<Vec3f,Vec3f> UAVPosRead::ReadPOS(fstream &ifs) {
    double x,y,z,r,p,h;
    char line[1024];
    ifs.getline(line,1024);
    sscanf(line,"%lf%lf%lf%lf%lf%lf",&x,&y,&z,&r,&p,&h);
    Vec3f trans(x,y,z);
    Vec3f rots(r,p,h);

    return make_pair(trans,rots);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//list SFM data list
float UAVDataList::UAVList_CreateSFMList()
{
    //影像文件夹(判断是否存在)
    printf("%s\n",_info_._g_image_dir_.c_str());
    if ( !stlplus::folder_exists( _info_._g_image_dir_ ) )
    {
        std::cerr << "\nThe input directory doesn't exist" << std::endl;
        return EXIT_FAILURE;
    }

    //特征点文件夹
    if ( !stlplus::folder_exists( _info_._g_feature_dir_ ) )
    {
        if ( !stlplus::folder_create(  _info_._g_feature_dir_ ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }
    //特征点文件夹
    if ( !stlplus::folder_exists( _info_._g_match_dir_ ) )
    {
        if ( !stlplus::folder_create(  _info_._g_match_dir_ ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    //几何校正文件夹
    if ( !stlplus::folder_exists( _info_._g_geocorrect_dir_ ) )
    {
        if ( !stlplus::folder_create(  _info_._g_geocorrect_dir_ ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    //点云数据输出
    if ( !stlplus::folder_exists( _info_._g_point_cloud_dir ) )
    {
        if ( !stlplus::folder_create(  _info_._g_point_cloud_dir ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    //辅助数据输出
    if ( !stlplus::folder_exists( _info_._g_auxiliary_dir ) )
    {
        if ( !stlplus::folder_create(  _info_._g_auxiliary_dir ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    //得到文件夹下所有文件
    std::vector<std::string> vec_image = stlplus::folder_files(  _info_._g_image_dir_ );
    std::sort(vec_image.begin(), vec_image.end());

    SfM_Data sfm_data;
    sfm_data.s_root_path =  _info_._g_image_dir_; // Setup main image root_path
    Views & views = sfm_data.views;
    Intrinsics & intrinsics = sfm_data.intrinsics;
    float files_size = 0;

    //POS Data
    bool pos_file = false;
    fstream ifs;
    UAVPosRead readPOS;
    if(_info_._g_Pos_data!="")
    {
        //POS from file
        pos_file = true;
        ifs.open(_info_._g_Pos_data);
        if(!ifs.is_open())
            pos_file = false;

        for(int i=0;i<_info_._g_Pos_bias;++i){
            char line[1024];
            ifs.getline(line,1024);
        }

    }

    /*
    _info_._g_centerLatitude= _info_._g_centerLongitude=_info_._g_centerHeight=0;
    //计算文件大小的参数
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
          iter_image != vec_image.end(); ++iter_image )
    {
        const std::string sImageFilename = stlplus::create_filespec(  _info_._g_image_dir_, *iter_image );
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);
        std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
        exifReader->open( sImageFilename );
        //POS data from Exif
        if(pos_file)
        {
            _info_._g_Has_Pos = true;

            // Add ECEF XYZ position to the GPS position array
            pair<Vec3f ,Vec3f> trans_rot = readPOS.ReadPOS(ifs);
            _info_._g_centerLatitude+=trans_rot.first(0);
            _info_._g_centerLongitude+=trans_rot.first(1);
            _info_._g_centerHeight+=trans_rot.first(2);
        } else if ( exifReader->open( sImageFilename ) && exifReader->doesHaveExifInfo() ) {
            // Try to parse EXIF metada & check existence of EXIF data
            _info_._g_Has_Pos = true;
            double latitude, longitude, altitude;
            if ( exifReader->GPSLatitude( &latitude ) &&
                 exifReader->GPSLongitude( &longitude ) &&
                 exifReader->GPSAltitude( &altitude ) )
            {
                _info_._g_centerLatitude+=latitude;
                _info_._g_centerLongitude+=longitude;
                _info_._g_centerHeight+=altitude;
            }
        }
    }

    _info_._g_centerLatitude/=vec_image.size();
    _info_._g_centerLongitude/=vec_image.size();
    _info_._g_centerHeight/=vec_image.size();
    if(_info_._g_Pos_data!="")
    {
        ifs.seekg(0,ios::beg);
        //POS from file
        pos_file = true;
        if(ifs.is_open())
        {
            for(int i=0;i<_info_._g_Pos_bias;++i){
                char line[1024];
                ifs.getline(line,1024);
            }
        }
    }
    */
    double width = -1,height = -1,focal = -1;
    POSProc posPro;
    C_Progress_display my_progress_bar( vec_image.size(),
                                        std::cout, "\n- Image listing -\n" );
    std::ostringstream error_report_stream;
    std::pair<bool, Vec3> val(true, Vec3::Zero());
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();
          iter_image != vec_image.end(); ++iter_image, ++my_progress_bar )
    {
        _info_._g_ppx=_info_._g_ppy= width = height= focal = -1;
        const std::string sImageFilename = stlplus::create_filespec(  _info_._g_image_dir_, *iter_image );
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);
        files_size+=float(get_file_size(sImageFilename.c_str()))/1024.0f;

        ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue;

        width = imgHeader.width;
        height = imgHeader.height;
        _info_._g_ppx = width / 2.0;
        _info_._g_ppy = height / 2.0;

        std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
        exifReader->open( sImageFilename );
        if(_info_._g_focal_x == -1||_info_._g_focal_y==-1)
            focal = std::max ( width, height ) * exifReader->getFocal() / _info_._g_ccdsize;
        else
            focal = std::max(_info_._g_focal_x,_info_._g_focal_y);

        _info_._g_focal_x=_info_._g_focal_y=focal;
        std::shared_ptr<IntrinsicBase> intrinsic(NULL);
        if(focal>0&&_info_._g_focal_y>0&&
                _info_._g_ppx>0&&_info_._g_ppy&&height>0&&width>0)
        {
            //initial intrinsic
            intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                    (width, height, focal, _info_._g_ppx, _info_._g_ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        }

        //POS data from Exif
        if(pos_file)
        {
            _info_._g_Has_Pos = true;

            // Add ECEF XYZ position to the GPS position array
            pair<Vec3f ,Vec3f> trans_rot = readPOS.ReadPOS(ifs);
            val.first = true;
            //Vec3 centllat = Vec3(_info_._g_centerLongitude*PI/180,_info_._g_centerLatitude*PI/180,_info_._g_centerHeight);
            //Vec3 curtllat = Vec3(trans_rot.first(1)*PI/180,trans_rot.first(0)*PI/180,trans_rot.first(2));
            //posPro.POSProc_POSTrans(centllat,curtllat,val.second);

            val.second = CooridinateTrans.LatLonToUTM(trans_rot.first(0),trans_rot.first(1),trans_rot.first(2));
            //val.second=CooridinateTrans.LatLonToXYZ(trans_rot.first(0),trans_rot.first(1),trans_rot.first(2));
            //使用影像内置的POS数据
            ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

            // Add intrinsic related to the image (if any)
            if (intrinsic == NULL)
            {
                //Since the view have invalid intrinsic data
                // (export the view, with an invalid intrinsic field value)
                v.id_intrinsic = UndefinedIndexT;
            }
            else
            {
                // Add the defined intrinsic to the sfm_container
                intrinsics[v.id_intrinsic] = intrinsic;
            }

            v.b_use_pose_center_ = true;
            v.pose_center_ = val.second;
            //TODO 添加旋转矩阵的初始值如果是POS数据有一个转换的过程
            //v.pose_rotation_

            views[v.id_view] = std::make_shared<ViewPriors>(v);
        } else if ( exifReader->open( sImageFilename ) && exifReader->doesHaveExifInfo() ) {
            // Try to parse EXIF metada & check existence of EXIF data
            _info_._g_Has_Pos = true;
            double latitude, longitude, altitude;
            if ( exifReader->GPSLatitude( &latitude ) &&
                 exifReader->GPSLongitude( &longitude ) &&
                 exifReader->GPSAltitude( &altitude ) )
            {
                // Add ECEF XYZ position to the GPS position array
                val.first = true;
                //val.second = CooridinateTrans.LatLonToXYZ( latitude, longitude, altitude );
                //直接转换为UTM坐标是不是好一点
                val.second = CooridinateTrans.LatLonToUTM(latitude,longitude,altitude);

                //使用影像内置的POS数据
                ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

                // Add intrinsic related to the image (if any)
                if (intrinsic == NULL)
                {
                    //Since the view have invalid intrinsic data
                    // (export the view, with an invalid intrinsic field value)
                    v.id_intrinsic = UndefinedIndexT;
                }
                else
                {
                    // Add the defined intrinsic to the sfm_container
                    intrinsics[v.id_intrinsic] = intrinsic;
                }
                v.b_use_pose_center_ = true;
                v.pose_center_ = val.second;
                views[v.id_view] = std::make_shared<ViewPriors>(v);
            }
            else{
                _info_._g_Has_Pos = false;
                View v(*iter_image, views.size(), views.size(), views.size(), width, height);
                if (intrinsic == NULL)
                {
                    //Since the view have invalid intrinsic data
                    // (export the view, with an invalid intrinsic field value)
                    v.id_intrinsic = UndefinedIndexT;
                }
                else
                {
                    // Add the defined intrinsic to the sfm_container
                    intrinsics[v.id_intrinsic] = intrinsic;
                }
                // Add the view to the sfm_container
                views[v.id_view] = std::make_shared<View>(v);
            }
        } else{
            _info_._g_Has_Pos = false;
            View v(*iter_image, views.size(), views.size(), views.size(), width, height);
            if (intrinsic == NULL)
            {
                //Since the view have invalid intrinsic data
                // (export the view, with an invalid intrinsic field value)
                v.id_intrinsic = UndefinedIndexT;
            }
            else
            {
                // Add the defined intrinsic to the sfm_container
                intrinsics[v.id_intrinsic] = intrinsic;
            }
            // Add the view to the sfm_container
            views[v.id_view] = std::make_shared<View>(v);
        }
    }
    if (!Save(
            sfm_data,
            _info_._g_SFM_data.c_str(),
            ESfM_Data(VIEWS|INTRINSICS)))
    {
        return EXIT_FAILURE;
    }
    if(ifs.is_open())
        ifs.close();
    return files_size/1024.0f;
}

float UAVDataList::UAVList_CreateImageRange(double dGroundSize)
{
    using kmldom::FolderPtr;
    using kmldom::KmlPtr;
    using kmldom::KmlFactory;
    using kmldom::PlacemarkPtr;
    using kmldom::LineStringPtr;
    using kmldom::CoordinatesPtr;
    using kmldom::DocumentPtr;

    UAVXYZToLatLonWGS84 coordiTrans;

    double size=0;
    if(_info_._g_Has_Pos)
    {
       SfM_Data sfm_data;
        if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(ALL)))
        {
            std::cerr << std::endl
                      << "The input SfM_Data file \"" << _info_._g_SFM_data << "\" cannot be read." << std::endl;
            return -1;
        }

        vector<Vec3> points_range;
        for (const auto & viewIter : sfm_data.GetViews())
        {
            const sfm::ViewPriors * prior = dynamic_cast<sfm::ViewPriors*>(viewIter.second.get());
            //中心点的坐标
            Vec3 position=prior->pose_center_;
            //Vec3 utmPosition = coordiTrans.LatLonToUTM(position(0),position(1),0);

            double focalx = _info_._g_focal_x;
            double focaly = _info_._g_focal_y;
            //计算范围
            Vec3 pnt0=coordiTrans.XYZToLatLon(position(0),position(1),position(2));
            double xrange = pnt0(2)*(prior->ui_width) /focalx;
            double yrange = pnt0(2)*(prior->ui_height)/focaly;


            Vec3 pnt1=coordiTrans.XYZToLatLon(position(0)-xrange/2,position(1)-yrange/2,position(2));
            Vec3 pnt2=coordiTrans.XYZToLatLon(position(0)-xrange/2,position(1)+yrange/2,position(2));
            Vec3 pnt3=coordiTrans.XYZToLatLon(position(0)+xrange/2,position(1)+yrange/2,position(2));
            Vec3 pnt4=coordiTrans.XYZToLatLon(position(0)+xrange/2,position(1)-yrange/2,position(2));

            points_range.push_back(pnt1);
            points_range.push_back(pnt2);
            points_range.push_back(pnt3);
            points_range.push_back(pnt4);
            size=size+(xrange/dGroundSize)*(yrange/dGroundSize)*3.0/1024.0/1024.0;
        }

        int num = points_range.size()/4;
        KmlFactory* factory(KmlFactory::GetFactory());
        DocumentPtr document = factory->CreateDocument();

        for(int i=0;i<num;++i)
        {
            PlacemarkPtr placemark  = factory->CreatePlacemark();
            LineStringPtr linerstring = factory->CreateLineString();
            CoordinatesPtr coordinate = factory->CreateCoordinates();

            for(int j=0;j<4;++j)
            {
                coordinate->add_latlng(points_range[j+4*i](0),points_range[j+4*i](1));
            }
            coordinate->add_latlng(points_range[4*i](0),points_range[4*i](1));
            linerstring->set_coordinates(coordinate);
            placemark->set_geometry(linerstring);
            document->add_feature(placemark);
        }
        KmlPtr kml = factory->CreateKml();
        kml->set_feature(document);

        string pathRange = stlplus::create_filespec(_info_._g_auxiliary_dir,"range.kml");
        ofstream ofs(pathRange.c_str(),ios::out);
        ofs << kmldom::SerializePretty(kml);
        ofs.close();

        return size;

    }
    else
        return 0;
}