//
// Created by wuwei on 17-3-26.
//
#include "UAVGeoProc.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "gdal_alg.h"
#include "gdalwarper.h"

//#include "openMVG/sfm/sfm.hpp"
#include "openMVG/cameras/cameras.hpp"
#include "UAVXYZToLatLonWGS84.h"
#include "UAVCommon.h"
#include "openMVG/multiview/triangulation.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace Eigen;

#include <omp.h>
#include <vector>
#include <map>
#include "UAVAuxiliary.h"

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


bool UAVGeoProc::UAVGeoProc_GeoProcDEM(string pathSFM, string pathDem,string pathDstDir, double dGroundSize, double dL, double dB)
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

//#pragma omp parallel for
    for(size_t k=0;k<image_list.size();++k)
    {
        vector<Vec3> groundPnts;
        vector<Vec2> featurePnts;
        FILE* fs=fopen("/home/wuwei/Data/求解结果误差.txt","w+");
        for ( const auto & iterLandmarks : landmarks )
        {
             if(iterLandmarks.second.obs.find(k)!=iterLandmarks.second.obs.end())
             {
                 groundPnts.push_back(iterLandmarks.second.X);
                 featurePnts.push_back(iterLandmarks.second.obs.at(k).x);

                 vector<Mat34> ps;
                 vector<Vec2>  pnts;
                 for(int l=0;l<iterLandmarks.second.obs.size();++l)
                 {
                     Pose3 pos3 = sfm_data.GetPoses().at(l);
                     Mat34 p = ((Pinhole_Intrinsic*)sfm_data.intrinsics.at(l).get())->get_projective_equivalent(pos3);
                     ps.push_back(p);
                     pnts.push_back(iterLandmarks.second.obs.at(l).x);
                     break;
                 }

                 //Vec3 llat = CooridinateTrans.XYZToLatLon(iterLandmarks.second.X(0),iterLandmarks.second.X(1),iterLandmarks.second.X(2));
                 for(int l=-300;l<=300;l+=10)
                 {
                     double z = iterLandmarks.second.X(2)+l;
                     Eigen::MatrixXd mat1(2,2);
                     mat1(0,0) = ps[0](2,0)*pnts[0](0)-ps[0](0,0);mat1(0,1) = ps[0](2,1)*pnts[0](0)-ps[0](0,1);
                     mat1(1,0) = ps[0](2,0)*pnts[0](1)-ps[0](1,0);mat1(1,1) = ps[0](2,1)*pnts[0](1)-ps[0](1,1);
                     Eigen::MatrixXd pa(2,1);
                     pa(0,0)=ps[0](0,2)*z+ps[0](0,3)-pnts[0](0)*(ps[0](2,2)*z+ps[0](2,3));
                     pa(1,0)=ps[0](1,2)*z+ps[0](1,3)-pnts[0](1)*(ps[0](2,2)*z+ps[0](2,3));

                     Eigen::MatrixXd XYZ(2,1);
                     XYZ=mat1.inverse()*pa;
                     fprintf(fs,"%lf   %lf \n",XYZ(0,0)-iterLandmarks.second.X(0),XYZ(1,0)-iterLandmarks.second.X(1));
                     int tmp=0;
                 }
                 fclose(fs);

            }
        }

        if(groundPnts.size()!=featurePnts.size()||groundPnts.size()<3)
        {
            cerr<<"error feature points and ground points\n";
            continue ;
        }
        else {
            int size = groundPnts.size();
            int xsize = sfm_data.GetViews().at(k).get()->ui_width;
            int ysize = sfm_data.GetViews().at(k).get()->ui_height;
            double flen = ((Pinhole_Intrinsic*)sfm_data.intrinsics.at(k).get())->focal();
            printf("%lf\n",flen);
            Pose3 pos3 = sfm_data.GetPoses().at(k);
            Vec3 cent = pos3.center();
            Mat34 p = ((Pinhole_Intrinsic*)sfm_data.intrinsics.at(k).get())->get_projective_equivalent(pos3);
            //printf("%lf   %lf   %lf\n",pos3.rotation()(0,0),pos3.rotation()(0,1),pos3.rotation()(0,2));
            //cout<<((Pinhole_Intrinsic*)sfm_data.intrinsics.at(k).get())->K()<<endl;
            double *gcps = new double[5 * size];
            for (int i = 0; i < size; ++i) {
                gcps[5 * i + 0] = featurePnts[i](0);
                gcps[5 * i + 1] = featurePnts[i](1);

                double x = gcps[5 * i + 2] = groundPnts[i](0);
                double y = gcps[5 * i + 3] = groundPnts[i](1);
                double z = gcps[5 * i + 4] = groundPnts[i](2);

                Vec3 lla;
                lla = CooridinateTrans.XYZToLatLon(x, y, z);
                Vec3 utm;
                utm = CooridinateTrans.LatLonToUTM(lla(0), lla(1), lla(2));

                gcps[5 * i + 2] = x;
                gcps[5 * i + 3] = y;
                gcps[5 * i + 4] = z;
            }
            //UAVGetProc_GeoCoordiTrans(gcps,size,dL,dB);
            Vec3 lla;
            lla = CooridinateTrans.XYZToLatLon(cent(0), cent(1), cent(2));
            Vec3 utm;
            utm = CooridinateTrans.LatLonToUTM(lla(0), lla(1), lla(2));
            string dst=stlplus::create_filespec(pathDstDir, stlplus::basename_part(image_list[k]), "tif");
            //UAVGeoProc_GeoCorrectionWithDEM(image_list[k],gcps,size,0.5,utm(0),utm(1),utm(2),flen,pathDem,dst);
            UAVGeoProc_GeoCorrectionWithDEM(image_list[k],p,dGroundSize,flen,pathDem,dst);
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


bool UAVGeoProc::UAVGeoProc_GeoProcWMT(double dGroundSize,double dL,double dB) {
    string sfm = stlplus::create_filespec(_info_._g_point_cloud_dir, "sfm_data", ".bin");
    if (!stlplus::file_exists(sfm)) {
        return false;
    } else {
        return UAVGeoProc_GeoProcWMT(sfm, _info_._g_geocorrect_dir_, dGroundSize, dL, dB);
    }
}


void UAVGeoProc::UAVGeoProc_GeoCorrectionWithDEM(string image,double* gcps,int gcpNum,double dGroundSize,double Xs,double Ys,double Zs,double fLen,string imageDem,string geoImageAccur)
{
    double params[6];
    Resection(gcps,gcpNum,fLen,Xs,Ys,Zs,params);

    //先不管坐标系统了
    GDALAllRegister();
    double data1=gcps[2];
    double data2=gcps[3];
    double data3=gcps[4];
    GDALDatasetH m_datasetsrc = GDALOpen(image.c_str(),GA_ReadOnly);
    int xsrc = GDALGetRasterXSize(m_datasetsrc);
    int ysrc = GDALGetRasterYSize(m_datasetsrc);
    int bands=GDALGetRasterCount(m_datasetsrc);

    GDALDatasetH m_datasetdem = GDALOpen(imageDem.c_str(),GA_ReadOnly);
    double adfGeoTransformdem[6];
    GDALGetGeoTransform(m_datasetdem,adfGeoTransformdem);
    int xdem = GDALGetRasterXSize(m_datasetdem);
    int ydem = GDALGetRasterYSize(m_datasetdem);
    float* dem = new float[xdem*ydem];
    GDALRasterIO(GDALGetRasterBand(m_datasetdem,1),GF_Read,0,0,xdem,ydem,dem,xdem,ydem,GDT_Float32,0,0);

    int hx = xsrc/2;
    int hy = ysrc/2;

    double dPhi = params[3];
    double dOmega = params[4];
    double dKappa = params[5];
    MatrixXd rotmat(3,3);
    rotmat(0,0)  = cos(dPhi)*cos(dKappa) - sin(dPhi)*sin(dOmega)*sin(dKappa);
    rotmat(0,1)  = -cos(dPhi)*sin(dKappa) - sin(dPhi)*sin(dOmega)*cos(dKappa);
    rotmat(0,2)  = -sin(dPhi)*cos(dOmega);
    rotmat(1,0) = cos(dOmega)*sin(dKappa);
    rotmat(1,1) = cos(dOmega)*cos(dKappa);
    rotmat(1,2) = -sin(dOmega);
    rotmat(2,0) = sin(dPhi)*cos(dKappa) + cos(dPhi)*sin(dOmega)*sin(dKappa);
    rotmat(2,1) = -sin(dPhi)*sin(dKappa) + cos(dPhi)*sin(dOmega)*cos(dKappa);
    rotmat(2,2) = cos(dPhi)*cos(dOmega);

    float* xPositions = new float[xsrc*ysrc];
    float* yPositions = new float[xsrc*ysrc];
    float* zPositions = new float[xsrc*ysrc];
    for(int i=0;i< xsrc*ysrc;++i)
    {
        zPositions[i]=400.0f;
    }

    //第一次计算
    for(int i=0;i<xsrc;++i){
        for(int j=0;j<ysrc;++j){
            //计算坐标
            MatrixXd ptImg(3,1),ptGeo(3,1);
            ptImg(0,0) = gcps[0];
            ptImg(1,0) = gcps[1];
            ptImg(2,0) = -fLen;

            ptGeo = rotmat.inverse()*ptImg;

            double X = (data3-Zs)*ptGeo(0,0)/ptGeo(2,0);
            double Y = (data3-Zs)*ptGeo(1,0)/ptGeo(2,0);
            //xPositions[j*xsrc+i]=Xs+X;
            //yPositions[j*xsrc+i]=Ys+Y;
            double tmp1 = Xs+X;
            double tmp2 = Ys+Y;

            int idem = (xPositions[j*xsrc+i]-adfGeoTransformdem[0])/adfGeoTransformdem[1];
            int jdem = (yPositions[j*xsrc+i]-adfGeoTransformdem[3])/adfGeoTransformdem[5];
            if(idem>xdem||idem<0||idem>ydem||jdem<0)
                zPositions[j*xsrc+i]=0;
            else
                zPositions[j*xsrc+i]=dem[jdem*xdem+idem];

        }
    }

    /*
    //第二次计算(不进行循环迭代计算了)
    for(int i=0;i<xsrc;++i){
        for(int j=0;j<ysrc;++j){
            //计算坐标
            MatrixXd ptImg(3,1),ptGeo(3,1);
            ptImg(0,0) = i-hx;
            ptImg(1,0) = hy-j;
            ptImg(2,0) = -fLen;

            ptGeo = rotmat.inverse()*ptImg;
            xPositions[j*xsrc+i]=Xs+(zPositions[j*xsrc+i]-Zs)*ptGeo(0,0)/ptGeo(2,0);
            yPositions[j*xsrc+i]=Ys+(zPositions[j*xsrc+i]-Zs)*ptGeo(1,0)/ptGeo(2,0);

            int idem = (xPositions[j*xsrc+i]-adfGeoTransformdem[0])/adfGeoTransformdem[1];
            int jdem = (yPositions[j*xsrc+i]-adfGeoTransformdem[3])/adfGeoTransformdem[5];
            if(idem>xdem||idem<0||idem>ydem||jdem<0)
                zPositions[j*xsrc+i]=0;
            else
                zPositions[j*xsrc+i]=dem[jdem*xdem+idem];
        }
    }
    */
    //
    double maxPt[2]={xPositions[0],yPositions[0]};
    double minPt[2]={xPositions[0],yPositions[0]};
    for(int i=0;i<xsrc*ysrc;++i)
    {
        maxPt[0]=max(maxPt[0],(double)xPositions[i]);
        maxPt[1]=max(maxPt[1],(double)yPositions[i]);

        minPt[0]=min(minPt[0],(double)xPositions[i]);
        minPt[1]=min(minPt[1],(double)yPositions[i]);
    }

    int xre = (maxPt[0]-minPt[0])/dGroundSize;
    printf("%d\n",xre);
    int yre = (maxPt[1]-minPt[1])/dGroundSize;
    printf("%d\n",yre);
    unsigned  char* pDataRe = new unsigned char[xre*yre];
    unsigned  char* pDataSrc=new unsigned char[xsrc*ysrc];
    GDALDatasetH m_datasetGeo = GDALCreate(GDALGetDriverByName("GTiff"),geoImageAccur.c_str(),xre,yre,bands,GDT_Byte,NULL);
    for(int i=0;i<bands;++i){
        GDALRasterIO(GDALGetRasterBand(m_datasetsrc,i+1),GF_Read,0,0,xsrc,ysrc,pDataSrc,xsrc,ysrc,GDT_Byte,0,0);
        UAVGeoProc_ImageResample(pDataSrc,xPositions,yPositions,maxPt,minPt,dGroundSize,xsrc,ysrc,xre,yre,pDataRe);
        GDALRasterIO(GDALGetRasterBand(m_datasetGeo,i+1),GF_Write,0,0,xre,yre,pDataSrc,xre,yre,GDT_Byte,0,0);
    }
    GDALClose(m_datasetsrc);
    GDALClose(m_datasetdem);
    GDALClose(m_datasetGeo);


    //重采样
    delete[] xPositions;xPositions=NULL;
    delete[] yPositions;yPositions=NULL;
    delete[] zPositions;zPositions=NULL;
    delete[] dem;dem=NULL;
    delete[] pDataRe;pDataRe=NULL;
    delete[] pDataSrc;pDataSrc=NULL;

}


void UAVGeoProc::UAVGeoProc_GeoCorrectionWithDEM(string image, Eigen::MatrixXd P,double dGroundSize, double fLen, string imageDem, string geoImageAccur)
{
    /*
    printf("%lf  %lf  %lf  %lf\n",P(0,0),P(0,1),P(0,2),P(0,3));
    printf("%lf  %lf  %lf  %lf\n",P(1,0),P(1,1),P(1,2),P(1,3));
    printf("%lf  %lf  %lf  %lf\n",P(2,0),P(2,1),P(2,2),P(2,3));
     */

    //Vec3 XYZCnt = CooridinateTrans.LatLonToXYZ(41.376846,124.780536,400);


    //先不管坐标系统了
    GDALAllRegister();
    GDALDatasetH m_datasetsrc = GDALOpen(image.c_str(),GA_ReadOnly);
    int xsrc = GDALGetRasterXSize(m_datasetsrc);
    int ysrc = GDALGetRasterYSize(m_datasetsrc);
    int bands=GDALGetRasterCount(m_datasetsrc);

    GDALDatasetH m_datasetdem = GDALOpen(imageDem.c_str(),GA_ReadOnly);
    double adfGeoTransformdem[6];
    GDALGetGeoTransform(m_datasetdem,adfGeoTransformdem);
    int xdem = GDALGetRasterXSize(m_datasetdem);
    int ydem = GDALGetRasterYSize(m_datasetdem);
    float* dem = new float[xdem*ydem];
    GDALRasterIO(GDALGetRasterBand(m_datasetdem,1),GF_Read,0,0,xdem,ydem,dem,xdem,ydem,GDT_Float32,0,0);

    float* xPositions = new float[xsrc*ysrc];
    float* yPositions = new float[xsrc*ysrc];
    float* zPositions = new float[xsrc*ysrc];
    memset(zPositions,0,sizeof(float)*xsrc*ysrc);

    //求解坐标
    for(int i=0;i<xsrc;++i){
        for(int j=0;j<ysrc;++j){
            //计算坐标
            Vec2 ptImg;
            ptImg(0,0) = i;
            ptImg(1,0) = j;

            double z = 500;//XYZCnt(2);
            Eigen::MatrixXd mat1(2,2);
            mat1(0,0) = P(2,0)*ptImg(0)-P(0,0);mat1(0,1) = P(2,1)*ptImg(0)-P(0,1);
            mat1(1,0) = P(2,0)*ptImg(1)-P(1,0);mat1(1,1) = P(2,1)*ptImg(1)-P(1,1);
            Eigen::MatrixXd pa(2,1);
            pa(0,0)=P(0,2)*z+P(0,3)-ptImg(0)*(P(2,2)*z+P(2,3));
            pa(1,0)=P(1,2)*z+P(1,3)-ptImg(1)*(P(2,2)*z+P(2,3));

            Eigen::MatrixXd XYZ(2,1);
            XYZ=mat1.inverse()*pa;
            //printf("%lf  %lf\n",XYZ(0),XYZ(1));
            /*
            //转换到UTM坐标系下
            Vec3 lla = CooridinateTrans.XYZToLatLon(XYZ(0),XYZ(1),z);
            double x=lla(0);
            double y=lla(1);
            Vec3 utm = CooridinateTrans.LatLonToUTM(lla(0),lla(1),z);
            xPositions[j*xsrc+i] = XYZ(0);
            yPositions[j*xsrc+i] = utm(1);
            */

            xPositions[j*xsrc+i] = XYZ(0);
            yPositions[j*xsrc+i] = XYZ(1);
            int idem = (xPositions[j*xsrc+i]-adfGeoTransformdem[0])/adfGeoTransformdem[1];
            int jdem = (yPositions[j*xsrc+i]-adfGeoTransformdem[3])/adfGeoTransformdem[5];
            if(idem>xdem||idem<0||idem>ydem||jdem<0)
                zPositions[j*xsrc+i]=0;
            else
                zPositions[j*xsrc+i]=dem[jdem*xdem+idem];
        }
    }

    double maxPt[2]={xPositions[0],yPositions[0]};
    double minPt[2]={xPositions[0],yPositions[0]};
    for(int i=0;i<xsrc*ysrc;++i)
    {
        maxPt[0]=max(maxPt[0],(double)xPositions[i]);
        maxPt[1]=max(maxPt[1],(double)yPositions[i]);

        minPt[0]=min(minPt[0],(double)xPositions[i]);
        minPt[1]=min(minPt[1],(double)yPositions[i]);
    }
    printf("%f %f\n",minPt[0],maxPt[0]);
    int xre = (maxPt[0]-minPt[0])/dGroundSize;
    printf("%d\n",xre);

    printf("%f %f\n",minPt[1],maxPt[1]);
    int yre = (maxPt[1]-minPt[1])/dGroundSize;
    printf("%d\n",yre);
    unsigned  char* pDataRe = new unsigned char[xre*yre];
    unsigned  char* pDataSrc=new unsigned char[xsrc*ysrc];
    GDALDatasetH m_datasetGeo = GDALCreate(GDALGetDriverByName("GTiff"),geoImageAccur.c_str(),xre,yre,bands,GDT_Byte,NULL);
    for(int i=0;i<bands;++i){
        GDALRasterIO(GDALGetRasterBand(m_datasetsrc,i+1),GF_Read,0,0,xsrc,ysrc,pDataSrc,xsrc,ysrc,GDT_Byte,0,0);
        UAVGeoProc_ImageResample(pDataSrc,xPositions,yPositions,maxPt,minPt,dGroundSize,xsrc,ysrc,xre,yre,pDataRe);
        GDALRasterIO(GDALGetRasterBand(m_datasetGeo,i+1),GF_Write,0,0,xre,yre,pDataSrc,xre,yre,GDT_Byte,0,0);
    }
    GDALClose(m_datasetsrc);
    GDALClose(m_datasetdem);
    GDALClose(m_datasetGeo);
    delete[] pDataRe;pDataRe=NULL;
    delete[] pDataSrc;pDataSrc=NULL;


    //重采样
    delete[] xPositions;xPositions=NULL;
    delete[] yPositions;yPositions=NULL;
    delete[] zPositions;zPositions=NULL;
    delete[] dem;dem=NULL;

}


void UAVGeoProc::UAVGeoProc_ImageResample(unsigned char* pDataSrc,float* xMap,float* yMap,double maxpt[],double minpt[],double dGroundSize,int xsrc,int ysrc,int xre,int yre,unsigned char* pDataRe)
{
    //
    double topleft[2]={minpt[0],maxpt[1]};  //左上角点

    float *fDGray = NULL;
    float *fDItem = NULL;
    try{
        fDGray = new float[xre*yre];
        fDItem = new float[xre*yre];
        memset(fDGray,0,sizeof(float)*xre*yre);
        memset(fDItem,0,sizeof(float)*xre*yre);
    }catch(bad_alloc){
        printf("allocat memory failed!\n");
        return ;
    }

    long long lOffset;
    float fTmpWeight[4];
    for(int i=0;i<xsrc;++i){
        for(int j=0;j<ysrc;++j){
            lOffset = j*xsrc+i;
            float x = fabs(xMap[lOffset]-topleft[0])/dGroundSize;
            float y = fabs(xMap[lOffset]-topleft[1])/dGroundSize;

            int nc = (int)x;
            int nr = (int)y;
            float fx = float(x-nc);
            float fy = float(y-nr);
            unsigned char fDN = pDataSrc[lOffset];

            fTmpWeight[0]=(float)(1-fx)*(1-fy)*fDN;
            fTmpWeight[1]=(float)(fx)*(1-fy)*fDN;
            fTmpWeight[2]=(float)(1-fx)*(fy)*fDN;
            fTmpWeight[3]=(float)(fx)*(fy)*fDN;

            if (nc>=0 && nc<xre && nr>=0 && nr<yre)
            {
                lOffset = nr*xre+nc;
                fDGray[lOffset] += fTmpWeight[0];
                fDItem[lOffset] += (1-fx)*(1-fy);						//左上点
                if (nc < xre-1)	//未处于右边界
                {
                    fDGray[lOffset+1] += fTmpWeight[1];
                    fDItem[lOffset+1] += fx*(1-fy);					//右上点
                }
                if (nr < yre-1)	//未处于下边界
                {
                    fDGray[lOffset+xre] += fTmpWeight[2];
                    fDItem[lOffset+xre] += (1-fx)*fy;		//左下点
                }
                if ( nc<xre-1 && nr<yre-1)
                {
                    fDGray[lOffset+xre+1] += fTmpWeight[3];
                    fDItem[lOffset+xre+1] += fx*fy;			//右下点
                }
            }
        }
    }

    for (int i = 0; i<xre; i++)
    {
        for (int j = 0; j<yre; j++)
        {
            lOffset = i*xre+j;
            if (fDItem[lOffset] != 0 )
            {
                pDataRe[lOffset] = (unsigned char)(fDGray[lOffset]/fDItem[lOffset]);
            }
            else	//修复黑点
            {
                if (i>0 && i<xre-1 && j>0 && j<yre-1)	//不处于边界位置
                {
                    float fSumValues = 0;
                    int nCount = 0;

                    if (fDItem[lOffset-xre-1] != 0)	//左上
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset-xre-1]/fDItem[lOffset-xre-1];
                    }
                    if (fDItem[lOffset-xre] != 0)		//上
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset-xre]/fDItem[lOffset-xre];
                    }
                    if (fDItem[lOffset-xre+1] != 0)	//右上
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset-xre+1]/fDItem[lOffset-xre+1];
                    }
                    if (fDItem[lOffset-1] != 0)					//左
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset-1]/fDItem[lOffset-1];
                    }
                    if (fDItem[lOffset+1] != 0)					//右
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset+1]/fDItem[lOffset+1];
                    }
                    if (fDItem[lOffset+xre-1] != 0)	//左下
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset+xre-1]/fDItem[lOffset+xre-1];
                    }
                    if (fDItem[lOffset+xre] != 0)		//下
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset+xre]/fDItem[lOffset+xre];
                    }
                    if (fDItem[lOffset+xre+1] != 0)	//右下
                    {
                        nCount++;
                        fSumValues += fDGray[lOffset+xre+1]/fDItem[lOffset+xre+1];
                    }
                    if (nCount >= 2)	//如果周围有五个以上不是黑点就进行均值处理
                    {
                        pDataRe[lOffset] = (unsigned char)(fSumValues/nCount);
                    }
                }
            }
        }
    }
    delete[]fDGray;fDGray=NULL;
    delete[]fDItem;fDItem=NULL;
}
