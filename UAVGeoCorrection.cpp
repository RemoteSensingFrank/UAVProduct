//
// Created by wuwei on 17-11-26.
//
#include <gdal_alg.h>
#include <ogr_spatialref.h>
#include <gdalwarper.h>
#include "UAVProcessGeometry.h"
#include "gdal_priv.h"
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

/**
 * 通过GCP进行几何校正
 * @param pathImg
 * @param gcps
 * @param gcpNumber
 * @param pathGeo
 * @return
 */
UAVErr UAVProcessGeoCorrect::UAVGeoCorrectGcps(std::string pathImg, GDAL_GCP *gcps,
                                               int gcpNumber, std::string pathGeo,
                                               double dGroundSize,double dL,double dB) {
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");	//中文路径
    // 打开原始图像并计算图像信息
    GDALDatasetH hSrcDS = GDALOpen(pathImg.c_str(), GA_ReadOnly);
    GDALDataType eDT = GDALGetRasterDataType(GDALGetRasterBand(hSrcDS, 1));	//获取数据类型
    int iBandCount = GDALGetRasterCount(hSrcDS);

    void *hTransformArg = GDALCreateGCPTransformer(gcpNumber, gcps, 1, FALSE);
    if (hTransformArg == NULL)
    {
        GDALClose(hSrcDS);
        return 9;
    }

    // 使用SuggestedWarpOutput函数计算输出图像四至范围、大小、六参数等信息
    double adfGeoTransform[6];
    double adfExtent[4];
    int    nPixels, nLines;

    if (GDALSuggestedWarpOutput2(hSrcDS, GDALGCPTransform, hTransformArg,
                                 adfGeoTransform, &nPixels, &nLines, adfExtent, 0) != CE_None)
    {
        GDALClose(hSrcDS);
        return 9;
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
    GDALDatasetH hDstDS = GDALCreate(hDriver, pathGeo.c_str(), nPixels, nLines, iBandCount, eDT, NULL);

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
    //delete[]gcp_pairs;
    return 0;
}

/**
 * 通过外参和平均高程进行几何校正
 * @param pathImg
 * @param P
 * @param avgHeight
 * @param pathGeo
 * @return
 */
UAVErr UAVProcessGeoCorrect::UAVGeoCorrectExterior(std::string pathImg, openMVG::Mat34 P,
                                                   double avgHeight, std::string pathGeo,
                                                   double dGroundSize,double dL,double dB) {
    GDALAllRegister();
    GDALDatasetH m_dataset = GDALOpen(pathImg.c_str(),GA_ReadOnly);
    if(m_dataset==NULL)
        return 1;

    int xsize = GDALGetRasterXSize(m_dataset);
    int ysize = GDALGetRasterYSize(m_dataset);

    //取五个点
    const int gcp_number = 5;
    double x[]={0,xsize/2,xsize,0,xsize};
    double y[]={0,ysize/2,ysize,ysize,0};
    GDAL_GCP gcp[gcp_number];

    for (int i = 0; i < 5; ++i) {
        Eigen::MatrixXd pa(2,1);
        Eigen::MatrixXd mat1(2,2);
        mat1(0,0) = P(2,0)*x[i]-P(0,0);mat1(0,1) = P(2,1)*x[i]-P(0,1);
        mat1(1,0) = P(2,0)*y[i]-P(1,0);mat1(1,1) = P(2,1)*y[i]-P(1,1);

        pa(0,0)=P(0,2)*avgHeight+P(0,3)-x[i]*(P(2,2)*avgHeight+P(2,3));
        pa(1,0)=P(1,2)*avgHeight+P(1,3)-y[i]*(P(2,2)*avgHeight+P(2,3));

        Eigen::MatrixXd XYZ(2,1);
        XYZ=mat1.inverse()*pa;

        gcp[i].dfGCPX = XYZ(0,0);
        gcp[i].dfGCPY = XYZ(1,0);
        gcp[i].dfGCPZ = avgHeight;
        gcp[i].dfGCPPixel = x[i];
        gcp[i].dfGCPLine  = y[i];
    }
    return UAVGeoCorrectGcps(pathImg,gcp,gcp_number,pathGeo,dGroundSize,dL,dB);
}
