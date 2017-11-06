//
// Created by wuwei on 17-11-6.
//

#include <memory>
#include "UAVPreProcess.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/sfm/sfm.hpp"

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
            case CoordinateUTM:
                pPorc->UAVProcessPOSExtractUTM(X,Y,Z);
            case CoordinateLocal:
                pPorc->UAVProcessPOSExtractLocal(X,Y,Z);
        }
    }


    bool bCalibParam=IsUndefineCalibParam(cParam);
    double width,height,ppx,ppy,focal;
    for(size_t i=0;i<vec_image.size();++i)
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

        if(!bPos)
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
            v.pose_center_ = openMVG::Vec3(pPorc->posList[i].dL,pPorc->posList[i].dB,pPorc->posList[i].dH);
            views[v.id_view] = std::make_shared<ViewPriors>(v);
        }
    }
    if (!Save(
            sfm_data,
            sfm_in,
            ESfM_Data(VIEWS|INTRINSICS)))
    {
        return 3;
    }
    return 0;

}