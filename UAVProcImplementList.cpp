//
// Created by wuwei on 17-11-4.
//

#include "UAVProcImplement.h"
#include "UAVExceptionImplement.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/sfm/sfm.hpp"

#include <algorithm>
#include <sys/stat.h>
#include <cstring>

#include <kml/dom.h>
#include <kml/engine.h>


UAVProcPos::UAVProcPos() {
    _list_params_=NULL;
    _list_params_ = new UAVParamList();
}
UAVProcPos::~UAVProcPos() {
    try {
        UAVExceptionCheck::UAVException_Check(UAVProcProcessClean());
    }catch(UAVExceptionPos except){
        printf("%s",except.what());
    }
}

UAVErr UAVProcPos::UAVProcProcessClean() {
    if(_list_params_!=NULL)
    {
        delete _list_params_;
        _list_params_=NULL;
    }

    if(_list_params_!=NULL)
        return 2;
    else
        return 0;
}

UAVErr UAVProcPos::UAVProcProcess(UAVProgressFunc pfnProgress, void *pProgressArg, bool bParallel) {
    return 0;
}

UAVErr UAVProcPos::UAVProcParam(UAVParam *param) {
    UAVParamList *paramList = (UAVParamList*)param;
    if(this->_list_params_==NULL) {
        this->_list_params_ = new UAVParamList();
        return 0;
    }
    else{
        if(param==NULL)
        {
            _list_params_ = NULL;
            return 0;
        }
        this->_list_params_->_sfm_data_in_=paramList->_sfm_data_in_;
        this->_list_params_->_sfm_data_out_=paramList->_sfm_data_out_;
        this->_list_params_->_gps_file_=paramList->_gps_file_;
        this->_list_params_->_path_imagedir_=paramList->_path_imagedir_;

        this->_list_params_->_calib_param_._flen_x_=paramList->_calib_param_._flen_x_;
        this->_list_params_->_calib_param_._flen_y_=paramList->_calib_param_._flen_y_;
        this->_list_params_->_calib_param_._ppx_=paramList->_calib_param_._ppx_;
        this->_list_params_->_calib_param_._ppy_=paramList->_calib_param_._ppy_;
        this->_list_params_->_calib_param_._ccd_size_=paramList->_calib_param_._ccd_size_;
        return 0;
    }
}

UAVErr UAVProcPos::UAVProcPos_GetRaw(POSParams poses, UAVProgressFunc pfnProgress, void *pProgressArg) {

    UAVErr err = 0;
    //整理数据获取影像数和POS数据行数
    std::string image_dir = this->_list_params_->_path_imagedir_;
    if ( !stlplus::folder_exists( image_dir) )
    {
        return 1;
    }
    std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
    std::sort(vec_image.begin(), vec_image.end());

    std::string pPath = this->_list_params_->_gps_file_;
    int iPosLines = 0;
    double step = 1.0/double(vec_image.size());

    if(!stlplus::file_exists(pPath))
    {
        //是否能从影像中获取GPS信息
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
                    poses[i]->dL=longitude;
                    poses[i]->dB=latitude;
                    poses[i]->dH=altitude;
                    poses[i]->dRoll=poses[i]->dPitch=poses[i]->dHeading=0;
                } else{
                    return 1;
                }
            }
        }
        return 0;

    } else
    {
        //是否能从文件中获取GPS信息
        FILE* file=NULL;
        file = fopen(pPath.c_str(),"r+");
        if(file==NULL)
            return 1;
        char posline[2048];
        while(!feof(file))
        {
            if(pfnProgress!=NULL)
                pfnProgress(iPosLines*step,"pos get",pProgressArg);
            poses[iPosLines]->UAVProcParamPos_Get(file);
            iPosLines++;
            if(iPosLines==vec_image.size())
                break;
        };

        if(iPosLines!=vec_image.size())
        {
            printf("POS　do not matches images!\n");
            fclose(file);
            file=NULL;
            return 1;
        }

        fclose(file);
        file=NULL;
        return 0;
    }


}

//TODO:
UAVErr UAVProcPos::UAVProcPos_ExtractReletative(POSParams posRaw,POSParams &posRel)
{
    return 0;
}

double UAVProcList::UAVProcListSize(){
    //获取文件大小
    std::string image_dir = this->_list_params_->_path_imagedir_;
    if ( !stlplus::folder_exists( image_dir) )
    {
        return -1;
    }
    std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
    std::sort(vec_image.begin(), vec_image.end());
    double fSize = 0;
    for (int i = 0; i < vec_image.size(); ++i) {
        struct stat tbuf;
        stat(vec_image[i].c_str(), &tbuf);
        fSize+=tbuf.st_size/1024.0;
    }
    printf("size of all images: %lf\n",fSize);
    return fSize;
}

UAVErr UAVProcList::UAVProcListRange(const char* pathKML)
{
    using kmldom::FolderPtr;
    using kmldom::KmlPtr;
    using kmldom::KmlFactory;
    using kmldom::PlacemarkPtr;
    using kmldom::LineStringPtr;
    using kmldom::CoordinatesPtr;
    using kmldom::DocumentPtr;

    string sfm = this->_list_params_->_sfm_data_in_;
    if(!stlplus::file_exists(sfm))
    {
        return 1;
    }
    return 0;

    /*
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

        string pathRange = stlplus::create_filespec(_info_._g_auxiliary_dir,pathKML);
        ofstream ofs(pathRange.c_str(),ios::out);
        ofs << kmldom::SerializePretty(kml);
        ofs.close();

        return size;

    }
    else
        return 0;
        */

}

UAVErr UAVProcList::UAVProcListFeature(std::string feature_dir,UAVParamFeature &feature_param)
{
    //特征点解算列表
    string image_dir = feature_param._path_imagedir_;
    if(!stlplus::folder_exists( image_dir))
    {
        return 1;
    }

    string featu_dir = feature_dir;
    if ( !stlplus::folder_exists( featu_dir) )
    {
        stlplus::folder_create(featu_dir);
    }
    //获取所有的影像

    std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
    std::sort(vec_image.begin(), vec_image.end());

    for (int i = 0; i < vec_image.size(); ++i) {
        std::string base_name = stlplus::basename_part(vec_image[i]);
        feature_param.UAVParamFeatureInsert(i,stlplus::create_filespec(image_dir,vec_image[i]),
                                              stlplus::create_filespec(featu_dir,base_name,"feats"));
    }

    return 0;
}

UAVErr UAVProcList::UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel)
{
    using namespace openMVG::sfm;

    if(this->_list_params_==NULL)
    {
        std::string msg = "empty parameters\n";
        this->UAVProcMsg(msg,std::cout);
        return 1;
    }

    string image_dir = _list_params_->_path_imagedir_;
    if(!stlplus::folder_exists( image_dir))
    {
        return 1;
    }
    std::vector<std::string> vec_image = stlplus::folder_files(image_dir);
    std::sort(vec_image.begin(), vec_image.end());

    string sfm_in = _list_params_->_sfm_data_in_;
    SfM_Data sfm_data;
    sfm_data.s_root_path =  image_dir;
    Views & views = sfm_data.views;
    Intrinsics & intrinsics = sfm_data.intrinsics;

    //判断解析POS
    POSParams pInfo;
    for(int i=0;i<vec_image.size();++i)
    {
        UAVParamPos* pParam = new UAVParamPos1();
        pInfo.insert(make_pair(i,pParam));
    }

    UAVErr bPOSErr = 0;
    bPOSErr = this->UAVProcPos_GetRaw(pInfo,pfnProgress,pProgressArg);

    double step = 1.0/double(vec_image.size());
    double width = -1,height = -1,focal = -1;
    UAVCalibParams cParams = this->_list_params_->_calib_param_;
    for(int i=0;i<vec_image.size();++i)
    {
        const std::string sImageFilename = stlplus::create_filespec(image_dir, vec_image[i]);
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        openMVG::image::ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue;

        width = imgHeader.width;
        height = imgHeader.height;
        cParams._ppx_=width/2.0;
        cParams._ppy_=height/2.0;

        std::unique_ptr<openMVG::exif::Exif_IO> exifReader(new openMVG::exif::Exif_IO_EasyExif);
        exifReader->open( sImageFilename );
        if(cParams._flen_x_==-1&& cParams._flen_y_==-1)
            focal = std::max ( width, height ) * exifReader->getFocal() / _list_params_->_calib_param_._ccd_size_;
        else
            focal = std::max(cParams._flen_x_,cParams._flen_x_);

        std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic(NULL);
        if(focal>0&&this->_list_params_->_calib_param_._ppx_>0&&
                this->_list_params_->_calib_param_._ppy_&&height>0&&width>0)
        {
            //initial intrinsic
            intrinsic = std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Radial_K3>
                    (width, height, focal, cParams._ppx_, cParams._ppy_, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
        }

        if(bPOSErr)
        {
            ViewPriors v(vec_image[i], views.size(), views.size(), views.size(), width, height);
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
            ViewPriors v(vec_image[i], views.size(), views.size(), views.size(), width, height);
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
            v.pose_center_ = openMVG::Vec3(pInfo[i]->dL,pInfo[i]->dB,pInfo[i]->dH);
            views[v.id_view] = std::make_shared<ViewPriors>(v);
        }
        if(pfnProgress!=NULL)
            pfnProgress((i+1)*step,"",pProgressArg);
    }
    if (!Save(
            sfm_data,
            this->_list_params_->_sfm_data_in_,
            ESfM_Data(VIEWS|INTRINSICS)))
    {
        return 3;
    }
    return 0;
}