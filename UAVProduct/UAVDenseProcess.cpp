//
// Created by wuwei on 17-8-7.
//
#include "UAVDenseProcess.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <boost/program_options.hpp>
#include <omp.h>
#include "OpenMVS/MVS.h"

void UAVDenseProcess::UAVDP_MVSProcInitialize() {

    //密集点云输出文件
    MVS::OPTDENSE::init();
    MVS::OPTDENSE::update();
    MVS::OPTDENSE::nResolutionLevel = 2;
    MVS::OPTDENSE::nMinResolution = 640;
    MVS::OPTDENSE::nNumViews = 4;
    MVS::OPTDENSE::nMinViewsFuse = 3;
    MVS::OPTDENSE::nEstimateColors = 1;
    MVS::OPTDENSE::nEstimateNormals = 0;

    // initialize global options
    Process::setCurrentProcessPriority((Process::Priority)-1);
#ifdef _USE_OPENMP
    omp_set_num_threads(2);
#endif
}

UAVErr UAVDenseProcess::UAVDP_MVSProc(std::string mvs,std::string dense)
{
    //初始化
    std::string export_mvs = mvs;
    std::string export_ply = stlplus::create_filespec(stlplus::folder_part(mvs),"dense",".ply");

    UAVDP_MVSProcInitialize();
    MVS::Scene scene(2);

    // load and estimate a dense point-cloud
    if (!scene.Load(MAKE_PATH_SAFE(export_mvs)))
        return 9;
    if (scene.pointcloud.IsEmpty()) {
        VERBOSE("error: empty initial point-cloud");
        return 9;
    }
    if ((ARCHIVE_TYPE)2 != ARCHIVE_MVS) {
        TD_TIMER_START();
        if (!scene.DenseReconstruction())
            return 9;
        VERBOSE("Densifying point-cloud completed: %u points (%s)", scene.pointcloud.points.GetSize(), TD_TIMER_GET_FMT().c_str());
    }

    // save the final mesh
    const String baseFileName(MAKE_PATH_SAFE(Util::getFullFileName(export_ply)));
    scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)2);
    scene.pointcloud.Save(baseFileName+_T(".ply"));
#if TD_VERBOSE != TD_VERBOSE_OFF
    if (VERBOSITY_LEVEL > 2)
        scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+_T(".ply"));
#endif
    return 0;
}

//
//#include "gdal_priv.h"
//#include "ogrsf_frmts.h"
//#include "gdal_alg.h"
//#include "cpl_progress.h"
//
//static void UAVDPPointsToDEM_Grid(double *points,int numPoints,double dL,double dB,double resolution,string pathDEM)
//{
//    double dfXMax=-99999999,dfXMin=99999999,dfYMax=-99999999,dfYMin=99999999;
//    double *padX = new double[numPoints];
//    double *padY = new double[numPoints];
//    double *padZ = new double[numPoints];
//
//    for(size_t i=0;i<numPoints;++i)
//    {
//        dfXMax = max(points[3*i+0],dfXMax);
//        dfXMin = min(points[3*i+0],dfXMin);
//        dfYMax = max(points[3*i+1],dfYMax);
//        dfYMin = min(points[3*i+1],dfYMin);
//
//        padX[i]= points[3*i+0];
//        padY[i]= points[3*i+1];
//        padZ[i]= points[3*i+2];
//    }
//
//    double pixResoultion = 0.5; //设置分辨率为0.5
//    GUInt32 nXSize = (dfXMax-dfXMin) / pixResoultion;
//    GUInt32 nYSize = (dfYMax-dfYMin) / pixResoultion;
//
//    GDALAllRegister();
//    float *pData  =  new float[nXSize*nYSize];
//
//    GDALGridNearestNeighborOptions *poOptions = new GDALGridNearestNeighborOptions();
//    poOptions->dfRadius1 = 0.5;
//    poOptions->dfRadius2 = 0.8;
//
//    GDALGridCreate(GGA_NearestNeighbor, poOptions, numPoints, padX, padY, padZ,
//                   dfXMin, dfXMax, dfYMin, dfYMax, nXSize, nYSize, GDT_Float32, pData, GDALTermProgress, NULL);
//
//    GDALDriver * pDriver = NULL;
//    pDriver = GetGDALDriverManager()->GetDriverByName("Gtiff");
//    GDALDataset *poDataset = pDriver->Create(pathDEM.c_str(), nXSize,nYSize, 1, GDT_Float32, NULL);
//
//    // 设置六参数
//    double adfGeoTransform[6] = {dfXMin, pixResoultion, 0 , dfYMax, 0, -pixResoultion};
//    poDataset->SetGeoTransform(adfGeoTransform );
//
//    int nZone = dL/6+39;
//    OGRSpatialReference oSrcSrs;
//    char *pszSRS_WKT = NULL;
//    oSrcSrs.SetUTM( nZone, TRUE );
//    oSrcSrs.SetWellKnownGeogCS( "WGS84" );
//    oSrcSrs.exportToWkt( &pszSRS_WKT );
//
//    // 写入影像
//    poDataset->RasterIO(GF_Write, 0, 0, nXSize, nYSize, pData, nXSize, nYSize, GDT_Float32, 1, 0, 0, 0, 0);
//    poDataset->SetProjection(pszSRS_WKT);
//
//    // 释放资源 关闭图像
//    delete poOptions;
//    delete []pData;
//    delete []padX;
//    delete []padY;
//    delete []padZ;
//    GDALClose(poDataset);
//    poDataset = NULL;
//}
//static void UAVDPPointsToDEM_Grid2(double *points,int numPoints,double dL,double dB,double resolution,string pathDEM)
//{
//    double dfXMax=-99999999,dfXMin=99999999,dfYMax=-99999999,dfYMin=99999999;
//
//    for(size_t i=0;i<numPoints;++i)
//    {
//        dfXMax = max(points[3*i+0],dfXMax);
//        dfXMin = min(points[3*i+0],dfXMin);
//        dfYMax = max(points[3*i+1],dfYMax);
//        dfYMin = min(points[3*i+1],dfYMin);
//    }
//
//    double pixResoultion = 0.5; //设置分辨率为0.5
//    GUInt32 nXSize = (dfXMax-dfXMin) / pixResoultion;
//    GUInt32 nYSize = (dfYMax-dfYMin) / pixResoultion;
//
//    GDALAllRegister();
//    float *pData  =  new float[nXSize*nYSize];
//    float *iData  =  new float[nXSize*nYSize];
//    memset(pData,0,sizeof(float)*nXSize*nYSize);
//    memset(iData,0,sizeof(int)*nXSize*nYSize);
//
//    int idxstep = numPoints/30;
//
//    printf("0%");
//    for(int i=0;i<numPoints;++i){
//        if(i%idxstep==0&&i>=idxstep)
//            printf(".");
//
//        //位置
//        int xgrid = floor((points[3*i+0]-dfXMin)/pixResoultion);
//        int ygrid = floor((points[3*i+1]-dfYMin)/pixResoultion);
//        ygrid=nYSize-ygrid;
//        pData[ygrid*nXSize+xgrid] +=  points[3*i+2];
//        iData[ygrid*nXSize+xgrid]++;
//    }
//    for(int i=0;i<nXSize*nYSize;++i){
//        //位置
//        if(iData[i]!=0)
//            pData[i]=pData[i]/iData[i];
//    }
//    printf("100\% -done\n");
//
//
//    GDALDriver * pDriver = NULL;
//    pDriver = GetGDALDriverManager()->GetDriverByName("Gtiff");
//    GDALDataset *poDataset = pDriver->Create(pathDEM.c_str(), nXSize,nYSize, 1, GDT_Float32, NULL);
//
//    // 设置六参数
//    double adfGeoTransform[6] = {dfXMin, pixResoultion, 0 , dfYMax, 0, -pixResoultion};
//    poDataset->SetGeoTransform(adfGeoTransform );
//
//    int nZone = dL/6+39;
//    OGRSpatialReference oSrcSrs;
//    char *pszSRS_WKT = NULL;
//    oSrcSrs.SetUTM( nZone, TRUE );
//    oSrcSrs.SetWellKnownGeogCS( "WGS84" );
//    oSrcSrs.exportToWkt( &pszSRS_WKT );
//
//    // 写入影像
//    poDataset->RasterIO(GF_Write, 0, 0, nXSize, nYSize, pData, nXSize, nYSize, GDT_Float32, 1, 0, 0, 0, 0);
//    poDataset->SetProjection(pszSRS_WKT);
//
//    // 释放资源 关闭图像
//    delete []pData;
//    delete []iData;
//    GDALClose(poDataset);
//    poDataset = NULL;
//}
