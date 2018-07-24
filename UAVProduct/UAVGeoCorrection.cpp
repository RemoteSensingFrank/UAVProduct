//
// Created by wuwei on 17-11-26.
//
#include <gdal_alg.h>
#include <ogr_spatialref.h>
#include <gdalwarper.h>
#include <gdal.h>
#include "UAVProcessGeometry.h"
#include "gdal_priv.h"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/stl/stl.hpp"
#include "UAVPreProcess.h"

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


    //这个算法在使用平均高程计算过程中由于高程的偏差可能会出现极大的误差
    for (int i = 0; i < 5; ++i)
    {
        /*algorithm have some problem to extract P from POS data
        Eigen::MatrixXd pa(2,1);
        Eigen::MatrixXd mat1(2,2);
        mat1(0,0) = P(2,0)*x[i]-P(0,0);mat1(0,1) = P(2,1)*x[i]-P(0,1);
        mat1(1,0) = P(2,0)*y[i]-P(1,0);mat1(1,1) = P(2,1)*y[i]-P(1,1);

        pa(0,0)=P(0,2)*avgHeight+P(0,3)-x[i]*(P(2,2)*avgHeight+P(2,3));
        pa(1,0)=P(1,2)*avgHeight+P(1,3)-y[i]*(P(2,2)*avgHeight+P(2,3));

        Eigen::MatrixXd XYZ(2,1);
        XYZ=mat1.inverse()*pa  ;
         */
        std::cout<<XYZ<<endl;
        gcp[i].dfGCPX = XYZ(0,0);
        gcp[i].dfGCPY = XYZ(1,0);
        gcp[i].dfGCPZ = avgHeight;
        gcp[i].dfGCPPixel = x[i];
        gcp[i].dfGCPLine  = y[i];
    }
    return UAVGeoCorrectGcps(pathImg,gcp,gcp_number,pathGeo,dGroundSize,dL,dB);
}

/***
 * 根据ＤＥＭ数据进行几何校正
 * @param pathGeo
 * @param P
 * @param pathDEM
 * @param pathGeoAccur
 * @return
 */
UAVErr UAVProcessGeoCorrect::UAVGeoCorrectDEM(std::string pathGeo, openMVG::Mat34 P, std::string pathDEM, std::string pathGeoAccur)
{
    GDALAllRegister();
    GDALDatasetH m_datasetDEM = GDALOpen(pathDEM.c_str(),GA_ReadOnly);
    GDALDatasetH m_datasetGeo = GDALOpen(pathGeo.c_str(),GA_ReadOnly);

    int xsize = GDALGetRasterXSize(m_datasetGeo);
    int ysize = GDALGetRasterYSize(m_datasetGeo);
    int bands = GDALGetRasterCount(m_datasetGeo);

    int xsizeDem=GDALGetRasterXSize(m_datasetDEM);
    int ysizeDem=GDALGetRasterYSize(m_datasetDEM);

    double *geoDEM = nullptr;
    double *dataDEM = nullptr;
    try {
        geoDEM  = new double[xsize*ysize];
        dataDEM = new double[xsizeDem*ysizeDem];
        memset(geoDEM,0,sizeof(double)*xsize*ysize);
        memset(dataDEM,0,sizeof(double)*xsizeDem*ysizeDem);
    }catch (std::bad_alloc e){
        printf("%s",e.what());
    }

    double adfGeoTrans[6];
    double adgGeoTransDEM[6];
    double adfGetInvTrans[6];
    GDALGetGeoTransform(m_datasetGeo,adfGeoTrans);
    GDALGetGeoTransform(m_datasetGeo,adgGeoTransDEM);
    GDALInvGeoTransform(adgGeoTransDEM,adfGetInvTrans);
    //每一个点对应的地面高程
    for(size_t i=0;i<xsize;++i)
    {
        for(size_t j=0;j<ysize;++j)
        {
            double gx,gy;
            gx = i*adfGeoTrans[0]+j*adfGeoTrans[1]+adfGeoTrans[2];
            gy = i*adfGeoTrans[3]+j*adfGeoTrans[4]+adfGeoTrans[5];

            double px,py;
            px = gx*adfGetInvTrans[0]+gx*adfGetInvTrans[1]+adfGetInvTrans[2];
            py = gy*adfGetInvTrans[3]+gy*adfGetInvTrans[4]+adfGetInvTrans[5];

            if(px>0&&px<xsizeDem&&py>0&&py<ysizeDem){
                //TODO:采样方式的选择
                geoDEM[j*xsize+i] = dataDEM[int(py*xsizeDem+px)];
            }
        }
    }

    //迭代解算坐标耗时计算
    //TODO:添加指示进度的标志
    for(size_t i=0;i<xsize;++i)
    {
        for(size_t j=0;j<ysize;++j)
        {

            double hDelta = 8848.44;//地面最高点
            double hErr = 3;
            int maxIter = 5;
            int iter = 0;
            do {
                Eigen::MatrixXd pa(2, 1);
                Eigen::MatrixXd mat1(2, 2);
                mat1(0, 0) = P(2, 0) * i - P(0, 0);
                mat1(0, 1) = P(2, 1) * i - P(0, 1);
                mat1(1, 0) = P(2, 0) * j - P(1, 0);
                mat1(1, 1) = P(2, 1) * j - P(1, 1);

                pa(0, 0) = P(0, 2) * geoDEM[j * xsize + i] + P(0, 3) - i * (P(2, 2) * geoDEM[j * xsize + i] + P(2, 3));
                pa(1, 0) = P(1, 2) * geoDEM[j * xsize + i] + P(1, 3) - j * (P(2, 2) * geoDEM[j * xsize + i] + P(2, 3));

                Eigen::MatrixXd XYZ(2, 1);
                XYZ = mat1.inverse() * pa;
                //更新高程点
                double px,py;
                px = XYZ(0)*adfGetInvTrans[0]+XYZ(0)*adfGetInvTrans[1]+adfGetInvTrans[2];
                py = XYZ(1)*adfGetInvTrans[3]+XYZ(1)*adfGetInvTrans[4]+adfGetInvTrans[5];

                if(px>0&&px<xsizeDem&&py>0&&py<ysizeDem){
                    //TODO:采样方式的选择
                    geoDEM[j*xsize+i] = dataDEM[int(py*xsizeDem+px)];
                }
                iter++;
            }while(hDelta>hErr&&iter<maxIter);
        }
    }

    //及时清理避免出现异常
    delete[]dataDEM;dataDEM=NULL;

    float *geoX = nullptr;
    float *geoY = nullptr;
    try {
        geoX    = new float[xsize*ysize];
        geoY    = new float[xsize*ysize];
        memset(geoX,0,sizeof(float)*xsize*ysize);
        memset(geoY,0,sizeof(float)*xsize*ysize);
    }catch (std::bad_alloc e){
        printf("%s",e.what());
    }

    //迭代解算坐标
    for(size_t i=0;i<xsize;++i)
    {
        for(size_t j=0;j<ysize;++j)
        {

            Eigen::MatrixXd pa(2, 1);
            Eigen::MatrixXd mat1(2, 2);
            mat1(0, 0) = P(2, 0) * i - P(0, 0);
            mat1(0, 1) = P(2, 1) * i - P(0, 1);
            mat1(1, 0) = P(2, 0) * j - P(1, 0);
            mat1(1, 1) = P(2, 1) * j - P(1, 1);

            pa(0, 0) = P(0, 2) * geoDEM[j * xsize + i] + P(0, 3) - i * (P(2, 2) * geoDEM[j * xsize + i] + P(2, 3));
            pa(1, 0) = P(1, 2) * geoDEM[j * xsize + i] + P(1, 3) - j * (P(2, 2) * geoDEM[j * xsize + i] + P(2, 3));

            Eigen::MatrixXd XYZ(2, 1);
            XYZ = mat1.inverse() * pa;

            geoX[j*xsize+i] = XYZ(0);
            geoY[j*xsize+i] = XYZ(1);
        }
    }
    double maxPt[2]={geoX[0],geoY[0]};
    double minPt[2]={geoX[0],geoY[0]};
    for(size_t i=0;i<xsize*ysize;++i)
    {
        maxPt[0]=std::max(maxPt[0],(double)geoX[i]);
        maxPt[1]=std::max(maxPt[1],(double)geoY[i]);

        minPt[0]=std::min(minPt[0],(double)geoX[i]);
        minPt[1]=std::min(minPt[1],(double)geoY[i]);
    }

    delete[]geoDEM;geoDEM=NULL;

    double dGround=std::max(fabs(adfGeoTrans[0]),fabs(adfGeoTrans[1]));

    int xre = (maxPt[0]-minPt[0])/dGround;
    int yre = (maxPt[1]-minPt[1])/dGround;

    unsigned  char* pDataRe = new unsigned char[xre*yre];
    unsigned  char* pDataSrc=new unsigned char[xsize*ysize];
    GDALDatasetH m_datasetAccur = GDALCreate(GDALGetDriverByName("GTiff"),pathGeoAccur.c_str(),xre,yre,bands,GDT_Byte,NULL);
    for(int i=0;i<bands;++i){
        GDALRasterIO(GDALGetRasterBand(m_datasetGeo,i+1),GF_Read,0,0,xsize,ysize,pDataSrc,xsize,ysize,GDT_Byte,0,0);
        UAVGeoProc_ImageResample(pDataSrc,geoX,geoY,maxPt,minPt,dGround,xsize,ysize,xre,yre,pDataRe);
        GDALRasterIO(GDALGetRasterBand(m_datasetAccur,i+1),GF_Write,0,0,xre,yre,pDataSrc,xre,yre,GDT_Byte,0,0);
    }

    //清理数据
    delete[]geoX;geoX=NULL;
    delete[]geoY;geoY=NULL;
    delete[]pDataRe;pDataRe=NULL;
    delete[]pDataSrc;pDataSrc=NULL;

    return 0;
}

/***
 * 对几何校正结果进行重采样
 * @param pDataSrc
 * @param xMap
 * @param yMap
 * @param maxpt
 * @param minpt
 * @param dGroundSize
 * @param xsrc
 * @param ysrc
 * @param xre
 * @param yre
 * @param pDataRe
 */
void UAVProcessGeoCorrect::UAVGeoProc_ImageResample(unsigned char *pDataSrc, float *xMap, float *yMap, double maxpt[], double minpt[],
                                      double dGroundSize, int xsrc, int ysrc, int xre, int yre, unsigned char *pDataRe){
    double topleft[2]={minpt[0],maxpt[1]};  //左上角点

    float *fDGray = NULL;
    float *fDItem = NULL;
    try{
        fDGray = new float[xre*yre];
        fDItem = new float[xre*yre];
        memset(fDGray,0,sizeof(float)*xre*yre);
        memset(fDItem,0,sizeof(float)*xre*yre);
    }catch(std::bad_alloc){
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

/***
 * 对光束结果进行几何校正法平差
 * @param sfm
 * @param dGeo
 * @param typeCoordi
 * @param dGroundSize
 * @return
 */
UAVErr UAVProcessGeoBundler::UAVGeoCorrectSFM(std::string sfm,std::string dGeo,
                                              CoordiListType typeCoordi,double dGroundSize){
    //output directory
    openMVG::sfm::SfM_Data sfm_data;
    if (!Load(sfm_data, sfm, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \""<< sfm << "\" cannot be read." << std::endl;
        return 9;
    }
    if ( !stlplus::folder_exists( dGeo ) )
    {
        if ( !stlplus::folder_create( dGeo ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return 9;
        }
    }

    std::vector<std::string> image_list;
    std::string sView_filename ;
    for (auto iter = sfm_data.GetViews().begin(); iter != sfm_data.GetViews().end();++iter)
    {
        const openMVG::sfm::View * v = iter->second.get();
        image_list.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                      v->s_Img_path));
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, v->s_Img_path);
    }

    const openMVG::sfm::Landmarks & landmarks = sfm_data.GetLandmarks();
    double dL,dB;
    printf("center longitude=");scanf("%lf",&dL);
    printf("center latitude =");scanf("%lf",&dB);

    std::vector<int> errIdx;
#pragma omp parallel for
    for(size_t k=0;k<image_list.size();++k)
    {
        std::vector<openMVG::Vec3> groundPnts;
        std::vector<openMVG::Vec2> featurePnts;
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
            std::cerr<<"error feature points and ground points\n";
            continue ;
        }
        else {
            int size = groundPnts.size();
            GDAL_GCP *gcps = new GDAL_GCP[size];
            for (int i = 0; i < size; ++i)
            {
                gcps[i].dfGCPPixel=featurePnts[i](0);
                gcps[i].dfGCPLine=featurePnts[i](1);

                double x= gcps[i].dfGCPX=groundPnts[i](0);
                double y= gcps[i].dfGCPY=groundPnts[i](1);
                double z= gcps[i].dfGCPZ=groundPnts[i](2);

                openMVG::Vec3 utm;
                openMVG::Vec3 lla;
                switch (typeCoordi)
                {
                    case CoordinateUTM:
                        utm=openMVG::Vec3(x,y,z);
                        break;
                    case CoordinateXYZ:
                        lla=UAVProcessGeoXYZToLatLon(x,y,z);
                        utm=UAVProcessGeoLatLonToUTM(lla(0),lla(1),lla(2));
                        break;
                    case CoordinateLocal:
                        double cx,cy,cz;
                        printf("please input center coordinate:");
                        printf("center X=");scanf("%lf",&cx);
                        printf("center Y=");scanf("%lf",&cy);
                        printf("center Z=");scanf("%lf",&cz);
                        utm=openMVG::Vec3(x+cx,y+cy,z+cz);
                        break;
                    default:
                        break;
                }
                gcps[i].dfGCPX=utm(0);
                gcps[i].dfGCPY=utm(1);
                gcps[i].dfGCPZ=utm(2);
            }
            std::string dst=stlplus::create_filespec(dGeo, stlplus::basename_part(image_list[k]), "tif");
            if(UAVGeoCorrectGcps(image_list[k],gcps,size,dst,dGroundSize,dL,dB))
                errIdx.push_back((int)k);
            delete[]gcps;gcps=NULL;
        }
    }
    if(errIdx.empty())
        return 0;
    else{
        for_each(errIdx.begin(),errIdx.end(),[](int i){printf("%d ",i);});
        printf("correction failed\n");
        return 9;
    }
}
