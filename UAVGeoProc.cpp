//
// Created by wuwei on 17-3-26.
//

#include "UAVGeoProc.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "gdal_alg.h"
#include "gdalwarper.h"

//#include"types.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "UAVXYZToLatLonWGS84.h"
/// Generic Image Collection image matching
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/stl/stl.hpp"
#include<Eigen/Dense>
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace Eigen;

#include "third_party/progress/progress.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "UAVCommon.h"

#include <omp.h>
#include <vector>
#include <map>

static  UAVXYZToLatLonWGS84 CooridinateTrans;
/**
* @brief 图像校正坐标转换结构体
*/
struct TransformChain
{
    /*! GDAL坐标转换函数指针 */
    GDALTransformerFunc GDALTransformer;
    /*! GDAL坐标转换参数 */
    void *              GDALTransformerArg;
    /*! 输出图像6参数 */
    double              adfGeotransform[6];
    /*! 输出图像逆6参数 */
    double              adfInvGeotransform[6];
};

/**
* @brief create transfom function
*
* 该函数用于创建一个将输出图像投影坐标转到输出图像的行列号坐标
* @sa DestroyGeoToPixelTransform
* @returns 返回GDAL坐标转换回调函数的参数 \ref GeoToPixelTransform
*/
void *CreateGeoToPixelTransform(GDALTransformerFunc GDALTransformer, void *GDALTransformerArg, double *padfGeotransform)
{
    TransformChain *pChain = new TransformChain;
    pChain->GDALTransformer = GDALTransformer;
    pChain->GDALTransformerArg = GDALTransformerArg;
    memcpy(pChain->adfGeotransform, padfGeotransform, sizeof(double) * 6);

    if (!GDALInvGeoTransform(pChain->adfGeotransform, pChain->adfInvGeotransform))
    {
        // 如果输出图像六参数不能计算逆六参数，则不能进行变换处理
        delete pChain;
        return NULL;
    }
    return (void*)pChain;
}

/**
* @brief 析构转换参数
*/
void DestroyGeoToPixelTransform(void *GeoToPixelTransfomArg)
{
    delete static_cast<TransformChain *>(GeoToPixelTransfomArg);
}

/**
* @brief 坐标转换函数
* @sa CreateGeoToPixelTransform
*/
int GeoToPixelTransform(void *pTransformerArg, int bDstToSrc, int nPointCount, double *x, double *y, double *z, int *panSuccess)
{
    TransformChain *pChain = static_cast<TransformChain*>(pTransformerArg);
    if (pChain == NULL)
        return FALSE;

    if (!bDstToSrc)	//坐标正变换
    {
        // 先调用GDAL库中的坐标转换函数，将坐标转为输出图像的投影坐标
        if (!pChain->GDALTransformer(pChain->GDALTransformerArg, bDstToSrc, nPointCount, x, y, z, panSuccess))
            return FALSE;

        // 再从输出图像投影坐标系转到输出图像行列号
#pragma omp parallel for
        for (int i = 0; i < nPointCount; ++i)
        {
            if (!panSuccess[i])
                continue;

            double xP = x[i];
            double yP = y[i];
            GDALApplyGeoTransform(pChain->adfInvGeotransform, xP, yP, &x[i], &y[i]);
        }
    }
    else	//坐标逆变换
    {
        // 先从输出图像行列号转到输出图像投影坐标系
#pragma omp parallel for
        for (int i = 0; i < nPointCount; ++i)
        {
            double P = x[i];
            double L = y[i];
            GDALApplyGeoTransform(pChain->adfGeotransform, P, L, &x[i], &y[i]);
        }
        // 再调用GDAL库中坐标转换函数从输出图像投影坐标转换到原始坐标
        if (!pChain->GDALTransformer(pChain->GDALTransformerArg, bDstToSrc, nPointCount, x, y, z, panSuccess))
            return FALSE;
    }

    return TRUE;
}


void UAVGeoProc::UAVGeoProc_GeoCorrection(string image,double* gcps,int gcpNum,double dGroundSize,double dL,double dB,string geoImage)
{
    //printf("\rstarting geo correct by GCPs");
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");	//中文路径
    // 打开原始图像并计算图像信息
    GDALDatasetH hSrcDS = GDALOpen(image.c_str(), GA_ReadOnly);
    GDALDataType eDT = GDALGetRasterDataType(GDALGetRasterBand(hSrcDS, 1));	//获取数据类型
    int iBandCount = GDALGetRasterCount(hSrcDS);

    // 创建几何多项式坐标转换关系
    GDAL_GCP* gcp_pairs = new GDAL_GCP[gcpNum];
    for (int i = 0; i<gcpNum; i++)
    {
        gcp_pairs[i].pszId = NULL;
        gcp_pairs[i].pszInfo = NULL;
        gcp_pairs[i].dfGCPPixel = gcps[5 * i + 0];
        gcp_pairs[i].dfGCPLine  = gcps[5 * i + 1];
        gcp_pairs[i].dfGCPX = gcps[5 * i + 2];
        gcp_pairs[i].dfGCPY = gcps[5 * i + 3];
        gcp_pairs[i].dfGCPZ = gcps[5 * i + 4];
    }
    void *hTransformArg = GDALCreateGCPTransformer(gcpNum, gcp_pairs, 1, FALSE);
    if (hTransformArg == NULL)
    {
        GDALClose(hSrcDS);
        return ;
    }

    // 使用SuggestedWarpOutput函数计算输出图像四至范围、大小、六参数等信息
    double adfGeoTransform[6];
    double adfExtent[4];
    int    nPixels, nLines;

    if (GDALSuggestedWarpOutput2(hSrcDS, GDALGCPTransform, hTransformArg,
                                 adfGeoTransform, &nPixels, &nLines, adfExtent, 0) != CE_None)
    {
        GDALClose(hSrcDS);
        return ;
    }
    double groudSize[2]={dGroundSize,-dGroundSize};
    // 下面开始根据用户指定的分辨率来反算输出图像的大小和六参数等信息
    double dResXSize = groudSize[0];
    double dResYSize = groudSize[1];

    //如果结果投影为投影坐标系统，则将其分辨率与原始影像一致
    float  scale = 10;
    if (dResXSize == 0.0 && dResYSize == 0.0)
    {
        double dGeoTrans[6] = { 0 };
        GDALGetGeoTransform(hSrcDS, dGeoTrans);
        dResXSize = ABS(adfGeoTransform[1]*scale);
        dResYSize = ABS(adfGeoTransform[5]*scale);
    }

    // 如果用户指定了输出图像的分辨率
    if (dResXSize != 0.0 || dResYSize != 0.0)
    {
        // 如果只指定了一个，使用自动计算的结果
        if (dResXSize == 0.0) dResXSize = adfGeoTransform[1];
        if (dResYSize == 0.0) dResYSize = adfGeoTransform[5];

        // 确保分辨率符号正确
        if (dResXSize < 0.0) dResXSize = -dResXSize;
        if (dResYSize > 0.0) dResYSize = -dResYSize;

        // 计算输出图像的范围
        double minX = adfGeoTransform[0];
        double maxX = adfGeoTransform[0] + adfGeoTransform[1] * nPixels;
        double maxY = adfGeoTransform[3];
        double minY = adfGeoTransform[3] + adfGeoTransform[5] * nLines;

        // 按照用户指定的分辨率来计算图像的输出大小以及范围
        nPixels = (int)(((maxX - minX) / dResXSize) + 0.5);
        nLines = (int)(((minY - maxY) / dResYSize) + 0.5);
        adfGeoTransform[0] = minX;
        adfGeoTransform[3] = maxY;
        adfGeoTransform[1] = dResXSize;
        adfGeoTransform[5] = dResYSize;
    }

    // 创建输出图像
    GDALDriverH hDriver = GDALGetDriverByName("GTiff");
    GDALDatasetH hDstDS = GDALCreate(hDriver, geoImage.c_str(), nPixels, nLines, iBandCount, eDT, NULL);

    OGRSpatialReference  oSRS;
    char *pszSRS_WKT = NULL;
    int nZone = dL/6+31;
    oSRS.SetUTM(nZone, TRUE);
    oSRS.SetWellKnownGeogCS("WGS84");
    oSRS.exportToWkt(&pszSRS_WKT);

    GDALSetProjection(hDstDS, pszSRS_WKT);
    GDALSetGeoTransform(hDstDS, adfGeoTransform);

    // 构造GDALWarp的变换选项
    GDALWarpOptions *psWO = GDALCreateWarpOptions();

    psWO->papszWarpOptions = CSLDuplicate(NULL);
    psWO->eWorkingDataType = eDT;
    psWO->eResampleAlg = GRA_NearestNeighbour;

    psWO->hSrcDS = hSrcDS;
    psWO->hDstDS = hDstDS;

    psWO->pfnTransformer = GeoToPixelTransform;
    psWO->pTransformerArg = CreateGeoToPixelTransform(GDALGCPTransform, hTransformArg, adfGeoTransform);

    psWO->nBandCount = iBandCount;
    psWO->panSrcBands = (int *)CPLMalloc(psWO->nBandCount*sizeof(int));
    psWO->panDstBands = (int *)CPLMalloc(psWO->nBandCount*sizeof(int));
    for (int i = 0; i < iBandCount; i++)
    {
        psWO->panSrcBands[i] = i + 1;
        psWO->panDstBands[i] = i + 1;
    }

    // 创建GDALWarp执行对象，并使用GDALWarpOptions来进行初始化
    GDALWarpOperation oWO;
    oWO.Initialize(psWO);

    // 执行处理
    oWO.ChunkAndWarpImage(0, 0, nPixels, nLines);

    // 释放资源和关闭文件
    DestroyGeoToPixelTransform(psWO->pTransformerArg);
    GDALDestroyWarpOptions(psWO);
    GDALDestroyGCPTransformer(hTransformArg);

    GDALClose(hSrcDS);
    GDALClose(hDstDS);
    delete[]gcp_pairs;
    //printf("-finish geo correct by GCPs\n");

    return ;
}


void UAVGeoProc::UAVGetProc_GeoCoordiTrans(double* gcps,int num,double dL,double dB)
{
    MatrixXd dEMMatrix(3,3), dPt(3,1),dModelPt(3,1);

    dL+=0.01;
    dEMMatrix(0,0) = -sin(D2R(dL));
    dEMMatrix(0,1) = -sin(D2R(dB))*cos(D2R(dL));
    dEMMatrix(0,2) = cos(D2R(dB))*cos(D2R(dL));
    dEMMatrix(1,0) = cos(D2R(dL));
    dEMMatrix(1,1) = -sin(D2R(dB))*sin(D2R(dL));
    dEMMatrix(1,2) = cos(D2R(dB))*sin(D2R(dL));
    dEMMatrix(2,0) = 0;
    dEMMatrix(2,1) = cos(D2R(dB));
    dEMMatrix(2,2) = sin(D2R(dB));

    Vec3 xyzCt =  CooridinateTrans.LatLonToXYZ(dB,dL,0);
    for(int i=0;i<num;++i)
    {

        dModelPt(0,0) = gcps[5*i+2];
        dModelPt(1,0) = gcps[5*i+3];
        dModelPt(2,0) = gcps[5*i+4];
        dPt = dEMMatrix*dModelPt;

        Vec3 transPnt;
        transPnt(0) = xyzCt(0) + dPt(0,0);
        transPnt(1) = xyzCt(1) + dPt(1,0);
        transPnt(2) = xyzCt(2) + dPt(2,0);
        Vec3 llat = CooridinateTrans.XYZToLatLon(transPnt(0),transPnt(1),transPnt(2));
        Vec3 utm  = CooridinateTrans.LatLonToUTM(llat(0),llat(1),llat(2));

        double x = utm(0);
        double y = utm(1);
        double z = utm(2);

        gcps[5*i+2] = x;
        gcps[5*i+3] = y;
        gcps[5*i+4] = z;
    }
}


bool UAVGeoProc::UAVGeoProc_GeoProc(string pathSFM,string pathDstDir,double dGroundSize,double dL,double dB)
{
    //output directory
    SfM_Data sfm_data;
    if (!Load(sfm_data, pathSFM, ESfM_Data(ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< pathSFM << "\" cannot be read." << std::endl;
        return false;
    }

    if ( !stlplus::folder_exists( pathDstDir ) )
    {
        if ( !stlplus::folder_create( pathDstDir ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return false;
        }
    }

    std::vector<string> image_list;
    string sView_filename ;
    for (Views::const_iterator iter = sfm_data.GetViews().begin();
         iter != sfm_data.GetViews().end();
         ++iter)
    {
        const View * v = iter->second.get();
        image_list.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                         v->s_Img_path));
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, v->s_Img_path);
    }

    const Landmarks & landmarks = sfm_data.GetLandmarks();

#pragma omp parallel for
    for(size_t k=0;k<image_list.size();++k)
    {
        vector<Vec3> groundPnts;
        vector<Vec2> featurePnts;
        for ( const auto & iterLandmarks : landmarks )
        {
            if(iterLandmarks.second.obs.find(k)!=iterLandmarks.second.obs.end())
            {
                groundPnts.push_back(iterLandmarks.second.X);
                featurePnts.push_back(iterLandmarks.second.obs.at(k).x);
            }
        }

        if(groundPnts.size()!=featurePnts.size()||groundPnts.size()<3)
        {
            cerr<<"error feature points and ground points\n";
            continue ;
        }
        else {
            int size = groundPnts.size();
            double *gcps = new double[5*size];
            for (int i = 0; i < size; ++i)
            {
                gcps[5*i+0]=featurePnts[i](0);
                gcps[5*i+1]=featurePnts[i](1);

                double x=gcps[5*i+2]=groundPnts[i](0);
                double y=gcps[5*i+3]=groundPnts[i](1);
                double z=gcps[5*i+4]=groundPnts[i](2);
                Vec3 lla;
                lla=CooridinateTrans.XYZToLatLon(x,y,z);
                Vec3 utm;
                utm=CooridinateTrans.LatLonToUTM(lla(0),lla(1),lla(2));
                gcps[5*i+2]=utm(0);
                gcps[5*i+3]=utm(1);
                gcps[5*i+4]=utm(2);

            }
            //UAVGetProc_GeoCoordiTrans(gcps,size,dL,dB);
            string dst=stlplus::create_filespec(pathDstDir, stlplus::basename_part(image_list[k]), "tif");
            if(_info_._g_Has_Pos)
                UAVGeoProc_GeoCorrection(image_list[k],gcps,size,dGroundSize,dL,dB,dst);
            else
                UAVGeoProc_GeoCorrection(image_list[k],gcps,size,0,dL,dB,dst);  //适应一下没有POS的校正。实际上有没有POS关系不大
            delete[]gcps;gcps=NULL;
        }

    }

    return true;
}

bool UAVGeoProc::UAVGeoProc_GeoProcWMT(string pathSFM,string pathDstDir,double dGroundSize,double dL,double dB)
{
    //output directory
    SfM_Data sfm_data;
    if (!Load(sfm_data, pathSFM, ESfM_Data(ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< pathSFM << "\" cannot be read." << std::endl;
        return false;
    }

    if ( !stlplus::folder_exists( pathDstDir ) )
    {
        if ( !stlplus::folder_create( pathDstDir ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return false;
        }
    }

    std::vector<string> image_list;
    string sView_filename ;
    for (Views::const_iterator iter = sfm_data.GetViews().begin();
         iter != sfm_data.GetViews().end();
         ++iter)
    {
        const View * v = iter->second.get();
        image_list.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                      v->s_Img_path));
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, v->s_Img_path);
    }

    const Landmarks & landmarks = sfm_data.GetLandmarks();

#pragma omp parallel for
    for(size_t k=0;k<image_list.size();++k)
    {
        vector<Vec3> groundPnts;
        vector<Vec2> featurePnts;
        for ( const auto & iterLandmarks : landmarks )
        {
            if(iterLandmarks.second.obs.find(k)!=iterLandmarks.second.obs.end())
            {
                groundPnts.push_back(iterLandmarks.second.X);
                featurePnts.push_back(iterLandmarks.second.obs.at(k).x);
            }
        }

        if(groundPnts.size()!=featurePnts.size()||groundPnts.size()<3)
        {
            cerr<<"error feature points and ground points\n";
            continue ;
        }
        else {
            int size = groundPnts.size();
            double *gcps = new double[5*size];
            for (int i = 0; i < size; ++i)
            {
                gcps[5*i+0]=featurePnts[i](0);
                gcps[5*i+1]=featurePnts[i](1);

                double x=gcps[5*i+2]=groundPnts[i](0);
                double y=gcps[5*i+3]=groundPnts[i](1);
                double z=gcps[5*i+4]=groundPnts[i](2);
                Vec3 lla;
                lla=CooridinateTrans.XYZToLatLon(x,y,z);
                Vec3 utm;
                utm=CooridinateTrans.LatLonToUTMWMT(lla(0),lla(1),lla(2));
                gcps[5*i+2]=utm(0);
                gcps[5*i+3]=utm(1);
                gcps[5*i+4]=utm(2);

            }
            //UAVGetProc_GeoCoordiTrans(gcps,size,dL,dB);
            string dst=stlplus::create_filespec(pathDstDir, stlplus::basename_part(image_list[k]), "tif");
            if(_info_._g_Has_Pos)
                UAVGeoProc_GeoCorrection(image_list[k],gcps,size,dGroundSize,dL,dB,dst);
            else
                UAVGeoProc_GeoCorrection(image_list[k],gcps,size,0,dL,dB,dst);  //适应一下没有POS的校正。实际上有没有POS关系不大
            delete[]gcps;gcps=NULL;
        }

    }

    return true;
}

bool UAVGeoProc::UAVGeoProc_GeoProc(double dGroundSize,double dL,double dB)
{
    string sfm=stlplus::create_filespec(_info_._g_point_cloud_dir, "sfm_data", ".bin");
    if(!stlplus::file_exists(sfm))
    {
        return false;
    } else{
        return UAVGeoProc_GeoProc(sfm,_info_._g_geocorrect_dir_,dGroundSize,dL,dB);
    }
}


bool UAVGeoProc::UAVGeoProc_GeoProcWMT(double dGroundSize,double dL,double dB)
{
    string sfm=stlplus::create_filespec(_info_._g_point_cloud_dir, "sfm_data", ".bin");
    if(!stlplus::file_exists(sfm))
    {
        return false;
    } else{
        return UAVGeoProc_GeoProcWMT(sfm,_info_._g_geocorrect_dir_,dGroundSize,dL,dB);
    }
}