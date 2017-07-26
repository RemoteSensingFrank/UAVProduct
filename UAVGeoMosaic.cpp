//
// Created by wuwei on 17-4-2.
//

#include "UAVGeoMosaic.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
inline string UAVGeoMosaic::GDALTool_GetFileName(const char* pszFile)
{
    string temp = pszFile;
    size_t a = temp.find_last_of('\\');
    size_t aa = temp.find_last_of('/');

    if (a != string::npos && aa != string::npos)
        a = max(a, aa);
    else
        a = max(a, aa);

    size_t b = temp.find_last_of('.');
    string strLayerName = temp.substr(a + 1);
    return strLayerName;
}
GDALDatasetH
UAVGeoMosaic::GDALTool_GDALWarpCreateOutput(char **papszSrcFiles, const char *pszFilename,
                                         const char *pszFormat, char **papszTO,
                                         char ***ppapszCreateOptions, GDALDataType eDT)
{
    GDALDriverH hDriver;
    GDALDatasetH hDstDS;
    void *hTransformArg;
    GDALColorTableH hCT = NULL;
    double dfWrkMinX = 0, dfWrkMaxX = 0, dfWrkMinY = 0, dfWrkMaxY = 0;
    double dfWrkResX = 0, dfWrkResY = 0;
    int nDstBandCount = 0;

    //获取输出文件类型驱动
    hDriver = GDALGetDriverByName(pszFormat);
    if (hDriver == NULL || GDALGetMetadataItem(hDriver, GDAL_DCAP_CREATE, NULL) == NULL)
        return NULL;

    //计算所有拼接影像地理范围
    char    *pszThisTargetSRS = (char*)CSLFetchNameValue(papszTO, "DST_SRS");
    if (pszThisTargetSRS != NULL)
        pszThisTargetSRS = CPLStrdup(pszThisTargetSRS);

    for (int iSrc = 0; papszSrcFiles[iSrc] != NULL; iSrc++)
    {
        GDALDatasetH hSrcDS;
        const char *pszThisSourceSRS = CSLFetchNameValue(papszTO, "SRC_SRS");

        hSrcDS = GDALOpen(papszSrcFiles[iSrc], GA_ReadOnly);
        if (hSrcDS == NULL)
            return NULL;

        // 检查当前文件是否存在波段
        if (GDALGetRasterCount(hSrcDS) == 0)
            return NULL;

        if (eDT == GDT_Unknown)
            eDT = GDALGetRasterDataType(GDALGetRasterBand(hSrcDS, 1));

        //将第一个颜色表的数值应用于最后输出文件
        if (iSrc == 0)
        {
            nDstBandCount = GDALGetRasterCount(hSrcDS);
            hCT = GDALGetRasterColorTable(GDALGetRasterBand(hSrcDS, 1));
            if (hCT != NULL)
                hCT = GDALCloneColorTable(hCT);
        }

        //如果没有设置投影信息则获取投影信息
        if (pszThisSourceSRS == NULL)
        {
            const char *pszMethod = CSLFetchNameValue(papszTO, "METHOD");

            if (GDALGetProjectionRef(hSrcDS) != NULL &&
                strlen(GDALGetProjectionRef(hSrcDS)) > 0 &&
                (pszMethod == NULL || EQUAL(pszMethod, "GEOTRANSFORM")))
                pszThisSourceSRS = GDALGetProjectionRef(hSrcDS);
            else if (GDALGetGCPProjection(hSrcDS) != NULL &&
                     strlen(GDALGetGCPProjection(hSrcDS)) > 0 &&
                     GDALGetGCPCount(hSrcDS) > 1 &&
                     (pszMethod == NULL || EQUALN(pszMethod, "GCP_", 4)))
                pszThisSourceSRS = GDALGetGCPProjection(hSrcDS);
            else if (pszMethod != NULL && EQUAL(pszMethod, "RPC"))
                pszThisSourceSRS = SRS_WKT_WGS84;
            else
                pszThisSourceSRS = "";
        }

        if (pszThisTargetSRS == NULL)
            pszThisTargetSRS = CPLStrdup(pszThisSourceSRS);

        //创建从源影像到目标影像的投影转换
        hTransformArg =
                GDALCreateGenImgProjTransformer2(hSrcDS, NULL, papszTO);

        if (hTransformArg == NULL)
        {
            CPLFree(pszThisTargetSRS);
            GDALClose(hSrcDS);
            return NULL;
        }


        //获取输出影像大致位置
        double adfThisGeoTransform[6];
        double adfExtent[4];
        int    nThisPixels, nThisLines;

        if (GDALSuggestedWarpOutput2(hSrcDS,
                                     GDALGenImgProjTransform, hTransformArg,
                                     adfThisGeoTransform,
                                     &nThisPixels, &nThisLines,
                                     adfExtent, 0) != CE_None)
        {
            CPLFree(pszThisTargetSRS);
            GDALClose(hSrcDS);
            return NULL;
        }

        if (CPLGetConfigOption("CHECK_WITH_INVERT_PROJ", NULL) == NULL)
        {
            double MinX = adfExtent[0];
            double MaxX = adfExtent[2];
            double MaxY = adfExtent[3];
            double MinY = adfExtent[1];
            int bSuccess = TRUE;

            /* Check that the the edges of the target image are in the validity area */
            /* of the target projection */
#define N_STEPS 20
            int i, j;
            for (i = 0; i <= N_STEPS && bSuccess; i++)
            {
                for (j = 0; j <= N_STEPS && bSuccess; j++)
                {
                    double dfRatioI = i * 1.0 / N_STEPS;
                    double dfRatioJ = j * 1.0 / N_STEPS;
                    double expected_x = (1 - dfRatioI) * MinX + dfRatioI * MaxX;
                    double expected_y = (1 - dfRatioJ) * MinY + dfRatioJ * MaxY;
                    double x = expected_x;
                    double y = expected_y;
                    double z = 0;

                    /* Target SRS coordinates to source image pixel coordinates */
                    if (!GDALGenImgProjTransform(hTransformArg, TRUE, 1, &x, &y, &z, &bSuccess) || !bSuccess)
                        bSuccess = FALSE;
                    /* Source image pixel coordinates to target SRS coordinates */
                    if (!GDALGenImgProjTransform(hTransformArg, FALSE, 1, &x, &y, &z, &bSuccess) || !bSuccess)
                        bSuccess = FALSE;

                    if (fabs(x - expected_x) > (MaxX - MinX) / nThisPixels ||
                        fabs(y - expected_y) > (MaxY - MinY) / nThisLines)
                        bSuccess = FALSE;
                }
            }

            /* If not, retry with CHECK_WITH_INVERT_PROJ=TRUE that forces ogrct.cpp */
            /* to check the consistency of each requested projection result with the */
            /* invert projection */
            if (!bSuccess)
            {
                CPLSetConfigOption("CHECK_WITH_INVERT_PROJ", "TRUE");
                CPLDebug("WARP", "Recompute out extent with CHECK_WITH_INVERT_PROJ=TRUE");
                GDALDestroyGenImgProjTransformer(hTransformArg);
                hTransformArg =
                        GDALCreateGenImgProjTransformer2(hSrcDS, NULL, papszTO);

                if (GDALSuggestedWarpOutput2(hSrcDS,
                                             GDALGenImgProjTransform, hTransformArg,
                                             adfThisGeoTransform,
                                             &nThisPixels, &nThisLines,
                                             adfExtent, 0) != CE_None)
                {
                    CPLFree(pszThisTargetSRS);
                    GDALClose(hSrcDS);
                    return NULL;
                }
            }
        }

        /* -------------------------------------------------------------------- */
        /*      Expand the working bounds to include this region, ensure the    */
        /*      working resolution is no more than this resolution.             */
        /* -------------------------------------------------------------------- */
        if (dfWrkMaxX == 0.0 && dfWrkMinX == 0.0)
        {
            dfWrkMinX = adfExtent[0];
            dfWrkMaxX = adfExtent[2];
            dfWrkMaxY = adfExtent[3];
            dfWrkMinY = adfExtent[1];
            dfWrkResX = adfThisGeoTransform[1];
            dfWrkResY = ABS(adfThisGeoTransform[5]);
        }
        else
        {
            dfWrkMinX = MIN(dfWrkMinX, adfExtent[0]);
            dfWrkMaxX = MAX(dfWrkMaxX, adfExtent[2]);
            dfWrkMaxY = MAX(dfWrkMaxY, adfExtent[3]);
            dfWrkMinY = MIN(dfWrkMinY, adfExtent[1]);
            dfWrkResX = MIN(dfWrkResX, adfThisGeoTransform[1]);
            dfWrkResY = MIN(dfWrkResY, ABS(adfThisGeoTransform[5]));
        }

        GDALDestroyGenImgProjTransformer(hTransformArg);

        GDALClose(hSrcDS);
    }


    //检查是否存在不可用源文件
    if (nDstBandCount == 0)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "No usable source images.");
        CPLFree(pszThisTargetSRS);
        return NULL;
    }


    //获取输出影像行列数
    double adfDstGeoTransform[6];
    int nPixels, nLines;

    adfDstGeoTransform[0] = dfWrkMinX;
    adfDstGeoTransform[1] = dfWrkResX;
    adfDstGeoTransform[2] = 0.0;
    adfDstGeoTransform[3] = dfWrkMaxY;
    adfDstGeoTransform[4] = 0.0;
    adfDstGeoTransform[5] = -1 * dfWrkResY;

    nPixels = (int)((dfWrkMaxX - dfWrkMinX) / dfWrkResX + 0.5);
    nLines = (int)((dfWrkMaxY - dfWrkMinY) / dfWrkResY + 0.5);

    //用户是否改变了相应变量
    if (dfXRes != 0.0 && dfYRes != 0.0)
    {
        if (dfMinX == 0.0 && dfMinY == 0.0 && dfMaxX == 0.0 && dfMaxY == 0.0)
        {
            dfMinX = adfDstGeoTransform[0];
            dfMaxX = adfDstGeoTransform[0] + adfDstGeoTransform[1] * nPixels;
            dfMaxY = adfDstGeoTransform[3];
            dfMinY = adfDstGeoTransform[3] + adfDstGeoTransform[5] * nLines;
        }

        nPixels = (int)((dfMaxX - dfMinX + (dfXRes / 2.0)) / dfXRes);
        nLines = (int)((dfMaxY - dfMinY + (dfYRes / 2.0)) / dfYRes);
        adfDstGeoTransform[0] = dfMinX;
        adfDstGeoTransform[3] = dfMaxY;
        adfDstGeoTransform[1] = dfXRes;
        adfDstGeoTransform[5] = -dfYRes;
    }
    else if (nForcePixels != 0 && nForceLines != 0)
    {
        if (dfMinX == 0.0 && dfMinY == 0.0 && dfMaxX == 0.0 && dfMaxY == 0.0)
        {
            dfMinX = dfWrkMinX;
            dfMaxX = dfWrkMaxX;
            dfMaxY = dfWrkMaxY;
            dfMinY = dfWrkMinY;
        }

        dfXRes = (dfMaxX - dfMinX) / nForcePixels;
        dfYRes = (dfMaxY - dfMinY) / nForceLines;

        adfDstGeoTransform[0] = dfMinX;
        adfDstGeoTransform[3] = dfMaxY;
        adfDstGeoTransform[1] = dfXRes;
        adfDstGeoTransform[5] = -dfYRes;

        nPixels = nForcePixels;
        nLines = nForceLines;
    }
    else if (nForcePixels != 0)
    {
        if (dfMinX == 0.0 && dfMinY == 0.0 && dfMaxX == 0.0 && dfMaxY == 0.0)
        {
            dfMinX = dfWrkMinX;
            dfMaxX = dfWrkMaxX;
            dfMaxY = dfWrkMaxY;
            dfMinY = dfWrkMinY;
        }

        dfXRes = (dfMaxX - dfMinX) / nForcePixels;
        dfYRes = dfXRes;

        adfDstGeoTransform[0] = dfMinX;
        adfDstGeoTransform[3] = dfMaxY;
        adfDstGeoTransform[1] = dfXRes;
        adfDstGeoTransform[5] = -dfYRes;

        nPixels = nForcePixels;
        nLines = (int)((dfMaxY - dfMinY + (dfYRes / 2.0)) / dfYRes);
    }
    else if (nForceLines != 0)
    {
        if (dfMinX == 0.0 && dfMinY == 0.0 && dfMaxX == 0.0 && dfMaxY == 0.0)
        {
            dfMinX = dfWrkMinX;
            dfMaxX = dfWrkMaxX;
            dfMaxY = dfWrkMaxY;
            dfMinY = dfWrkMinY;
        }

        dfYRes = (dfMaxY - dfMinY) / nForceLines;
        dfXRes = dfYRes;

        adfDstGeoTransform[0] = dfMinX;
        adfDstGeoTransform[3] = dfMaxY;
        adfDstGeoTransform[1] = dfXRes;
        adfDstGeoTransform[5] = -dfYRes;

        nPixels = (int)((dfMaxX - dfMinX + (dfXRes / 2.0)) / dfXRes);
        nLines = nForceLines;
    }
    else if (dfMinX != 0.0 || dfMinY != 0.0 || dfMaxX != 0.0 || dfMaxY != 0.0)
    {
        dfXRes = adfDstGeoTransform[1];
        dfYRes = fabs(adfDstGeoTransform[5]);

        nPixels = (int)((dfMaxX - dfMinX + (dfXRes / 2.0)) / dfXRes);
        nLines = (int)((dfMaxY - dfMinY + (dfYRes / 2.0)) / dfYRes);

        dfXRes = (dfMaxX - dfMinX) / nPixels;
        dfYRes = (dfMaxY - dfMinY) / nLines;

        adfDstGeoTransform[0] = dfMinX;
        adfDstGeoTransform[3] = dfMaxY;
        adfDstGeoTransform[1] = dfXRes;
        adfDstGeoTransform[5] = -dfYRes;
    }

    //是否要添加alpha波段
    if (bEnableSrcAlpha)
        nDstBandCount--;

    if (bEnableDstAlpha)
        nDstBandCount++;

    //创建输出影像
    hDstDS = GDALCreate(hDriver, pszFilename, nPixels, nLines,
                        nDstBandCount, eDT, *ppapszCreateOptions);

    if (hDstDS == NULL)
    {
        CPLFree(pszThisTargetSRS);
        return NULL;
    }

    //将投影信息写入输出影像
    GDALSetProjection(hDstDS, pszThisTargetSRS);
    GDALSetGeoTransform(hDstDS, adfDstGeoTransform);

    /* -------------------------------------------------------------------- */
    /*      Try to set color interpretation of output file alpha band.      */
    /*      TODO: We should likely try to copy the other bands too.         */
    /* -------------------------------------------------------------------- */
    if (bEnableDstAlpha)
    {
        GDALSetRasterColorInterpretation(
                GDALGetRasterBand(hDstDS, nDstBandCount),
                GCI_AlphaBand);
    }

    /* -------------------------------------------------------------------- */
    /*      Copy the color table, if required.                              */
    /* -------------------------------------------------------------------- */
    if (hCT != NULL)
    {
        GDALSetRasterColorTable(GDALGetRasterBand(hDstDS, 1), hCT);
        GDALDestroyColorTable(hCT);
    }

    CPLFree(pszThisTargetSRS);
    return hDstDS;
}
void UAVGeoMosaic::GDALTool_TransformCutlineToSource(GDALDatasetH hSrcDS, void *hCutline,
                                                  char ***ppapszWarpOptions, char **papszTO_In)
{
    OGRGeometryH hMultiPolygon = OGR_G_Clone((OGRGeometryH)hCutline);
    char **papszTO = CSLDuplicate(papszTO_In);

    //检查源投影是一致的
    OGRSpatialReferenceH  hRasterSRS = NULL;
    const char *pszProjection = NULL;

    if (GDALGetProjectionRef(hSrcDS) != NULL
        && strlen(GDALGetProjectionRef(hSrcDS)) > 0)
        pszProjection = GDALGetProjectionRef(hSrcDS);
    else if (GDALGetGCPProjection(hSrcDS) != NULL)
        pszProjection = GDALGetGCPProjection(hSrcDS);

    if (pszProjection != NULL)
    {
        hRasterSRS = OSRNewSpatialReference(NULL);
        if (OSRImportFromWkt(hRasterSRS, (char **)&pszProjection) != CE_None)
        {
            OSRDestroySpatialReference(hRasterSRS);
            hRasterSRS = NULL;
        }
    }

    OGRSpatialReferenceH hSrcSRS = OGR_G_GetSpatialReference(hMultiPolygon);
    if (hRasterSRS != NULL && hSrcSRS != NULL)
    {
        /* ok, we will reproject */
    }
    else if (hRasterSRS != NULL && hSrcSRS == NULL)
    {
        fprintf(stderr,
                "Warning : the source raster dataset has a SRS, but the input vector layer\n"
                        "not.  Cutline results may be incorrect.\n");
    }
    else if (hRasterSRS == NULL && hSrcSRS != NULL)
    {
        fprintf(stderr,
                "Warning : the input vector layer has a SRS, but the source raster dataset does not.\n"
                        "Cutline results may be incorrect.\n");
    }

    if (hRasterSRS != NULL)
        OSRDestroySpatialReference(hRasterSRS);

    /* -------------------------------------------------------------------- */
    /*      Extract the cutline SRS WKT.                                    */
    /* -------------------------------------------------------------------- */
    if (hSrcSRS != NULL)
    {
        char *pszCutlineSRS_WKT = NULL;

        OSRExportToWkt(hSrcSRS, &pszCutlineSRS_WKT);
        papszTO = CSLSetNameValue(papszTO, "DST_SRS", pszCutlineSRS_WKT);
        CPLFree(pszCutlineSRS_WKT);
    }
    else
    {
        int iDstSRS = CSLFindString(papszTO, "DST_SRS");
        if (iDstSRS >= 0)
            papszTO = CSLRemoveStrings(papszTO, iDstSRS, 1, NULL);
    }

    //地理坐标到影像行列的转换
    CutlineTransformer oTransformer;

    oTransformer.hSrcImageTransformer =
            GDALCreateGenImgProjTransformer2(hSrcDS, NULL, papszTO);

    CSLDestroy(papszTO);

    if (oTransformer.hSrcImageTransformer == NULL)
        exit(1);

    OGR_G_Transform(hMultiPolygon,
                    (OGRCoordinateTransformationH)&oTransformer);

    GDALDestroyGenImgProjTransformer(oTransformer.hSrcImageTransformer);

    /* -------------------------------------------------------------------- */
    /*      Convert aggregate geometry into WKT.                            */
    /* -------------------------------------------------------------------- */
    char *pszWKT = NULL;

    OGR_G_ExportToWkt(hMultiPolygon, &pszWKT);
    OGR_G_DestroyGeometry(hMultiPolygon);

    *ppapszWarpOptions = CSLSetNameValue(*ppapszWarpOptions,
                                         "CUTLINE", pszWKT);
    CPLFree(pszWKT);
}
long UAVGeoMosaic::GDALTool_LoadCutline(const char *pszCutlineDSName, const char *pszCLayer,
                                     const char *pszCWHERE, const char *pszCSQL, void **phCutlineRet)
{
    OGRRegisterAll();

    //打开矢量数据集
    OGRDataSourceH hSrcDS;

    hSrcDS = OGROpen(pszCutlineDSName, FALSE, NULL);
    if (hSrcDS == NULL)
        return -1;

    //获取矢量图层
    OGRLayerH hLayer = NULL;

    if (pszCSQL != NULL)
        hLayer = OGR_DS_ExecuteSQL(hSrcDS, pszCSQL, NULL, NULL);
    else if (pszCLayer != NULL)
        hLayer = OGR_DS_GetLayerByName(hSrcDS, pszCLayer);
    else
        hLayer = OGR_DS_GetLayer(hSrcDS, 0);

    if (hLayer == NULL)
    {
        OGR_DS_Destroy(hSrcDS);
        return -2;
    }

    /* -------------------------------------------------------------------- */
    /*      Apply WHERE clause if there is one.                             */
    /* -------------------------------------------------------------------- */
    if (pszCWHERE != NULL)
        OGR_L_SetAttributeFilter(hLayer, pszCWHERE);

    /* -------------------------------------------------------------------- */
    /*      Collect the geometries from this layer, and build list of       */
    /*      burn values.                                                    */
    /* -------------------------------------------------------------------- */
    OGRFeatureH hFeat;
    OGRGeometryH hMultiPolygon = OGR_G_CreateGeometry(wkbMultiPolygon);

    OGR_L_ResetReading(hLayer);

    while ((hFeat = OGR_L_GetNextFeature(hLayer)) != NULL)
    {
        OGRGeometryH hGeom = OGR_F_GetGeometryRef(hFeat);

        if (hGeom == NULL)
        {
            OGR_DS_Destroy(hSrcDS);
            return -2;
        }

        OGRwkbGeometryType eType = wkbFlatten(OGR_G_GetGeometryType(hGeom));

        if (eType == wkbPolygon)
            OGR_G_AddGeometry(hMultiPolygon, hGeom);
        else if (eType == wkbMultiPolygon)
        {
            int iGeom;

            for (iGeom = 0; iGeom < OGR_G_GetGeometryCount(hGeom); iGeom++)
            {
                OGR_G_AddGeometry(hMultiPolygon,
                                  OGR_G_GetGeometryRef(hGeom, iGeom));
            }
        }
        else
        {
            OGR_DS_Destroy(hSrcDS);
            return -2;
        }
        OGR_F_Destroy(hFeat);
    }

    if (OGR_G_GetGeometryCount(hMultiPolygon) == 0)
    {
        return -2;
    }

    /* -------------------------------------------------------------------- */
    /*      Ensure the coordinate system gets set on the geometry.          */
    /* -------------------------------------------------------------------- */
    OGR_G_AssignSpatialReference(hMultiPolygon, OGR_L_GetSpatialRef(hLayer));

    *phCutlineRet = (void *)hMultiPolygon;

    //清除指针
    if (pszCSQL != NULL)
        OGR_DS_ReleaseResultSet(hSrcDS, hLayer);

    OGR_DS_Destroy(hSrcDS);
    return 0;
}
long UAVGeoMosaic::GDALTool_ImageMosaicing(vector<string> vStrSrcFiles, const char* pszCutLineFile, const char* pszOutFile,
                                        GDALResampleAlg eResampleMethod, const char *pszFormat)
{
    //输入输出文件
    GDALDatasetH		hDstDS;
    char              **papszSrcFiles = NULL;
    const char         *pszDstFilename = NULL;
    int                 bCreateOutput = FALSE;
    //坐标转换对象
    void               *hTransformArg, *hGenImgProjArg = NULL, *hApproxArg = NULL;
    char              **papszWarpOptions = NULL;
    double              dfErrorThreshold = 0.125;
    GDALTransformerFunc pfnTransformer = NULL;
    //输出文件选项
    char              **papszCreateOptions = NULL;
    GDALDataType        eOutputType = GDT_Unknown, eWorkingType = GDT_Unknown;

    GDALResampleAlg     eResampleAlg = (GDALResampleAlg)eResampleMethod;
    //NODATA设置
    const char         *pszSrcNodata = NULL;
    const char         *pszDstNodata = NULL;
    int                 bMulti = FALSE;
    char              **papszTO = NULL;
    //镶嵌线文件
    const char         *pszCutlineDSName = NULL;

    if (vStrSrcFiles.empty())
        return -1;

    GDALAllRegister();
    OGRRegisterAll();

    pszSrcNodata = "0 0 0";
    pszDstNodata = "0 0 0";
    dfXRes = 0.0;
    dfYRes = 0.0;
    bCreateOutput = TRUE;
    eResampleAlg = GRA_Bilinear;
    pszCutlineDSName = pszCutLineFile;

    for (size_t i = 0; i<vStrSrcFiles.size(); i++)
    {
        string strFile = vStrSrcFiles[i];
        papszSrcFiles = CSLAddString(papszSrcFiles, strFile.c_str());
    }

    pszDstFilename = pszOutFile;
    if (pszDstFilename == NULL)
        return -2;

    /* -------------------------------------------------------------------- */
    /*      输出文件是否已经存在？                                          */
    /* -------------------------------------------------------------------- */
    CPLPushErrorHandler(CPLQuietErrorHandler);
    hDstDS = GDALOpen(pszDstFilename, GA_Update);
    CPLPopErrorHandler();

    //避免重写已经存在的文件
    if (hDstDS == NULL)
    {
        CPLPushErrorHandler(CPLQuietErrorHandler);
        hDstDS = GDALOpen(pszDstFilename, GA_ReadOnly);
        CPLPopErrorHandler();

        if (hDstDS)
        {
            fprintf(stderr, "输出文件%s存在，但是不能写入\n", pszDstFilename);
            GDALClose(hDstDS);
            return -2;
        }
    }

    /* -------------------------------------------------------------------- */
    /*      创建输出文件                                                    */
    /* -------------------------------------------------------------------- */
    int   bInitDestSetForFirst = FALSE;

    if (hDstDS == NULL)
    {
        int iResult = 0;
        hDstDS = GDALTool_GDALWarpCreateOutput(papszSrcFiles, pszDstFilename, pszFormat,
                                               papszTO, &papszCreateOptions, eOutputType);

        if (iResult != 0)
            return iResult;

        bCreateOutput = TRUE;

        if (CSLFetchNameValue(papszWarpOptions, "INIT_DEST") == NULL &&
            pszDstNodata == NULL)
        {
            papszWarpOptions = CSLSetNameValue(papszWarpOptions,
                                               "INIT_DEST", "0");
            bInitDestSetForFirst = TRUE;
        }
        else if (CSLFetchNameValue(papszWarpOptions, "INIT_DEST") == NULL)
        {
            papszWarpOptions = CSLSetNameValue(papszWarpOptions,
                                               "INIT_DEST", "NO_DATA");
            bInitDestSetForFirst = TRUE;
        }

        CSLDestroy(papszCreateOptions);
        papszCreateOptions = NULL;
    }

    if (hDstDS == NULL)
        return -2;

    /* -------------------------------------------------------------------- */
    /*      遍历所有的输入文件，并将其写入输出文件                          */
    /* -------------------------------------------------------------------- */
    int iRev = 0;
    for (int iSrc = 0; papszSrcFiles[iSrc] != NULL; iSrc++)
    {
        GDALDatasetH hSrcDS;

        /* -------------------------------------------------------------------- */
        /*      打开文件                                                        */
        /* -------------------------------------------------------------------- */
        hSrcDS = GDALOpen(papszSrcFiles[iSrc], GA_ReadOnly);

        if (hSrcDS == NULL)
        {
            iRev = -2;
            goto CLEAN;
        }

        /* -------------------------------------------------------------------- */
        /*      检查输入文件是否不存在波段                                      */
        /* -------------------------------------------------------------------- */
        if (GDALGetRasterCount(hSrcDS) == 0)
        {
            fprintf(stderr, "输入文件 %s 不存在波段。\n", papszSrcFiles[iSrc]);
            iRev = -2;
            goto CLEAN;
        }

        /* -------------------------------------------------------------------- */
        /*      处理alpha波段                                                   */
        /* -------------------------------------------------------------------- */
        GDALColorInterp ci = GDALGetRasterColorInterpretation(
                GDALGetRasterBand(hSrcDS, GDALGetRasterCount(hSrcDS)));

        if (ci == GCI_AlphaBand && !bEnableSrcAlpha)
            bEnableSrcAlpha = TRUE;

        /* -------------------------------------------------------------------- */
        /*      创建转换参数从源坐标到目标坐标系                                */
        /* -------------------------------------------------------------------- */
        hTransformArg = hGenImgProjArg =
                GDALCreateGenImgProjTransformer2(hSrcDS, hDstDS, papszTO);

        if (hTransformArg == NULL)
        {
            iRev = -2;
            goto CLEAN;
        }

        pfnTransformer = GDALGenImgProjTransform;

        /* -------------------------------------------------------------------- */
        /*      Warp the transformer with a linear approximator unless the      */
        /*      acceptable error is zero.                                       */
        /* -------------------------------------------------------------------- */
        if (dfErrorThreshold != 0.0)
        {
            hTransformArg = hApproxArg =
                    GDALCreateApproxTransformer(GDALGenImgProjTransform,
                                                hGenImgProjArg, dfErrorThreshold);
            pfnTransformer = GDALApproxTransform;
        }

        /* -------------------------------------------------------------------- */
        /*      Clear temporary INIT_DEST settings after the first image.       */
        /* -------------------------------------------------------------------- */
        if (bInitDestSetForFirst && iSrc == 1)
            papszWarpOptions = CSLSetNameValue(papszWarpOptions,
                                               "INIT_DEST", NULL);

        /* -------------------------------------------------------------------- */
        /*      创建warp选项                                                    */
        /* -------------------------------------------------------------------- */
        GDALWarpOptions *psWO = GDALCreateWarpOptions();

        psWO->papszWarpOptions = CSLDuplicate(papszWarpOptions);
        psWO->eWorkingDataType = eWorkingType;
        psWO->eResampleAlg = eResampleAlg;

        psWO->hSrcDS = hSrcDS;
        psWO->hDstDS = hDstDS;

        psWO->pfnTransformer = pfnTransformer;
        psWO->pTransformerArg = hTransformArg;
        psWO->dfWarpMemoryLimit = 52428800;		//使用50M的内存

        /* -------------------------------------------------------------------- */
        /*      创建波段映射关系                                                */
        /* -------------------------------------------------------------------- */
        if (bEnableSrcAlpha)
            psWO->nBandCount = GDALGetRasterCount(hSrcDS) - 1;
        else
            psWO->nBandCount = GDALGetRasterCount(hSrcDS);

        psWO->panSrcBands = (int *)CPLMalloc(psWO->nBandCount*sizeof(int));
        psWO->panDstBands = (int *)CPLMalloc(psWO->nBandCount*sizeof(int));

        for (int i = 0; i < psWO->nBandCount; i++)
        {
            psWO->panSrcBands[i] = i + 1;
            psWO->panDstBands[i] = i + 1;
        }

        /* -------------------------------------------------------------------- */
        /*      构建alpha波段                                                   */
        /* -------------------------------------------------------------------- */
        if (bEnableSrcAlpha)
            psWO->nSrcAlphaBand = GDALGetRasterCount(hSrcDS);

        if (!bEnableDstAlpha
            && GDALGetRasterCount(hDstDS) == psWO->nBandCount + 1
            && GDALGetRasterColorInterpretation(
                GDALGetRasterBand(hDstDS, GDALGetRasterCount(hDstDS)))
               == GCI_AlphaBand)
        {
            printf("Using band %d of destination image as alpha.\n",
                   GDALGetRasterCount(hDstDS));

            bEnableDstAlpha = TRUE;
        }

        if (bEnableDstAlpha)
            psWO->nDstAlphaBand = GDALGetRasterCount(hDstDS);

        /* -------------------------------------------------------------------- */
        /*      创建  NODATA 选项                                               */
        /* -------------------------------------------------------------------- */
        if (pszSrcNodata != NULL && !EQUALN(pszSrcNodata, "n", 1))
        {
            char **papszTokens = CSLTokenizeString(pszSrcNodata);
            int  nTokenCount = CSLCount(papszTokens);

            psWO->padfSrcNoDataReal = (double *)
                    CPLMalloc(psWO->nBandCount*sizeof(double));
            psWO->padfSrcNoDataImag = (double *)
                    CPLMalloc(psWO->nBandCount*sizeof(double));

            for (int i = 0; i < psWO->nBandCount; i++)
            {
                if (i < nTokenCount)
                {
                    CPLStringToComplex(papszTokens[i],
                                       psWO->padfSrcNoDataReal + i,
                                       psWO->padfSrcNoDataImag + i);
                }
                else
                {
                    psWO->padfSrcNoDataReal[i] = psWO->padfSrcNoDataReal[i - 1];
                    psWO->padfSrcNoDataImag[i] = psWO->padfSrcNoDataImag[i - 1];
                }
            }

            CSLDestroy(papszTokens);

            psWO->papszWarpOptions = CSLSetNameValue(psWO->papszWarpOptions,
                                                     "UNIFIED_SRC_NODATA", "YES");
        }

        /* -------------------------------------------------------------------- */
        /*      没有指定NODATA值，从源文件中读取                                */
        /* -------------------------------------------------------------------- */
        if (pszSrcNodata == NULL)
        {
            int bHaveNodata = FALSE;
            double dfReal = 0.0;

            for (int i = 0; !bHaveNodata && i < psWO->nBandCount; i++)
            {
                GDALRasterBandH hBand = GDALGetRasterBand(hSrcDS, i + 1);
                dfReal = GDALGetRasterNoDataValue(hBand, &bHaveNodata);
            }

            if (bHaveNodata)
            {
                printf("Using internal nodata values (eg. %g) for image %s.\n",
                       dfReal, papszSrcFiles[iSrc]);
                psWO->padfSrcNoDataReal = (double *)
                        CPLMalloc(psWO->nBandCount*sizeof(double));
                psWO->padfSrcNoDataImag = (double *)
                        CPLMalloc(psWO->nBandCount*sizeof(double));

                for (int i = 0; i < psWO->nBandCount; i++)
                {
                    GDALRasterBandH hBand = GDALGetRasterBand(hSrcDS, i + 1);

                    dfReal = GDALGetRasterNoDataValue(hBand, &bHaveNodata);

                    if (bHaveNodata)
                    {
                        psWO->padfSrcNoDataReal[i] = dfReal;
                        psWO->padfSrcNoDataImag[i] = 0.0;
                    }
                    else
                    {
                        psWO->padfSrcNoDataReal[i] = -123456.789;
                        psWO->padfSrcNoDataImag[i] = 0.0;
                    }
                }
            }
        }

        /* -------------------------------------------------------------------- */
        /*      设置输出文件的NODATA为最大值                                    */
        /* -------------------------------------------------------------------- */
        if (pszDstNodata != NULL)
        {
            char **papszTokens = CSLTokenizeString(pszDstNodata);
            int  nTokenCount = CSLCount(papszTokens);

            psWO->padfDstNoDataReal = (double *)
                    CPLMalloc(psWO->nBandCount*sizeof(double));
            psWO->padfDstNoDataImag = (double *)
                    CPLMalloc(psWO->nBandCount*sizeof(double));

            for (int i = 0; i < psWO->nBandCount; i++)
            {
                if (i < nTokenCount)
                {
                    CPLStringToComplex(papszTokens[i],
                                       psWO->padfDstNoDataReal + i,
                                       psWO->padfDstNoDataImag + i);
                }
                else
                {
                    psWO->padfDstNoDataReal[i] = psWO->padfDstNoDataReal[i - 1];
                    psWO->padfDstNoDataImag[i] = psWO->padfDstNoDataImag[i - 1];
                }

                GDALRasterBandH hBand = GDALGetRasterBand(hDstDS, i + 1);
                int bClamped = FALSE, bRounded = FALSE;

#define CLAMP(val,type,minval,maxval) \
	do { if (val < minval) { bClamped = TRUE; val = minval; } \
		else if (val > maxval) { bClamped = TRUE; val = maxval; } \
		else if (val != (type)val) { bRounded = TRUE; val = (type)(val + 0.5); } } \
		while(0)

                switch (GDALGetRasterDataType(hBand))
                {
                    case GDT_Byte:
                        CLAMP(psWO->padfDstNoDataReal[i], GByte,
                              0.0, 255.0);
                        break;
                    case GDT_Int16:
                        CLAMP(psWO->padfDstNoDataReal[i], GInt16,
                              -32768.0, 32767.0);
                        break;
                    case GDT_UInt16:
                        CLAMP(psWO->padfDstNoDataReal[i], GUInt16,
                              0.0, 65535.0);
                        break;
                    case GDT_Int32:
                        CLAMP(psWO->padfDstNoDataReal[i], GInt32,
                              -2147483648.0, 2147483647.0);
                        break;
                    case GDT_UInt32:
                        CLAMP(psWO->padfDstNoDataReal[i], GUInt32,
                              0.0, 4294967295.0);
                        break;
                    default:
                        break;
                }

                if (bClamped)
                {
                    printf("for band %d, destination nodata value has been clamped "
                                   "to %.0f, the original value being out of range.\n",
                           i + 1, psWO->padfDstNoDataReal[i]);
                }
                else if (bRounded)
                {
                    printf("for band %d, destination nodata value has been rounded "
                                   "to %.0f, %s being an integer datatype.\n",
                           i + 1, psWO->padfDstNoDataReal[i],
                           GDALGetDataTypeName(GDALGetRasterDataType(hBand)));
                }

                if (bCreateOutput)
                {
                    GDALSetRasterNoDataValue(
                            GDALGetRasterBand(hDstDS, psWO->panDstBands[i]),
                            psWO->padfDstNoDataReal[i]);
                }
            }

            CSLDestroy(papszTokens);
        }

        /* -------------------------------------------------------------------- */
        /*      读取镶嵌线                                                      */
        /* -------------------------------------------------------------------- */
        void *hCutline = NULL;
        if (pszCutlineDSName != NULL)
        {
            string strFileName = GDALTool_GetFileName(papszSrcFiles[iSrc]);
            string strWhere = "影像路径=\"" + strFileName + "\"";
            GDALTool_LoadCutline(pszCutlineDSName, NULL, strWhere.c_str(), NULL, &hCutline);
        }

        if (hCutline != NULL)
        {
            GDALTool_TransformCutlineToSource(hSrcDS, hCutline, &(psWO->papszWarpOptions), papszTO);
        }

        /* -------------------------------------------------------------------- */
        /*      初始化执行warp                                                  */
        /* -------------------------------------------------------------------- */
        GDALWarpOperation oWO;
        CPLErr CE = CE_None;
        if (oWO.Initialize(psWO) == CE_None)
        {
            if (bMulti)
                CE = oWO.ChunkAndWarpMulti(0, 0, GDALGetRasterXSize(hDstDS), GDALGetRasterYSize(hDstDS));
            else
                CE = oWO.ChunkAndWarpImage(0, 0, GDALGetRasterXSize(hDstDS), GDALGetRasterYSize(hDstDS));
        }

        /* -------------------------------------------------------------------- */
        /*      清理资源                                                        */
        /* -------------------------------------------------------------------- */
        if (hApproxArg != NULL)
            GDALDestroyApproxTransformer(hApproxArg);

        if (hGenImgProjArg != NULL)
            GDALDestroyGenImgProjTransformer(hGenImgProjArg);

        GDALDestroyWarpOptions(psWO);

        if (hCutline != NULL)	//释放镶嵌块资源
        {
            OGR_G_DestroyGeometry((OGRGeometryH)hCutline);
            hCutline = NULL;
        }

        GDALClose(hSrcDS);

        if (CE != CE_None)
        {
            GDALClose(hDstDS);

            CSLDestroy(papszSrcFiles);
            CSLDestroy(papszWarpOptions);
            CSLDestroy(papszTO);

            GDALDumpOpenDatasets(stderr);
            OGRCleanupAll();
            return -1;
        }
    }

    /* -------------------------------------------------------------------- */
    /*      善后工作                                                        */
    /* -------------------------------------------------------------------- */
    CLEAN:

    GDALClose(hDstDS);

    CSLDestroy(papszSrcFiles);
    CSLDestroy(papszWarpOptions);
    CSLDestroy(papszTO);

    //GDALDumpOpenDatasets( stderr );

    return 0;
}

void UAVGeoMosaic::GDALTool_GetMosaicVector(string pszImageDir, vector<string> &vStrSrcFiles)
{
    if ( !stlplus::folder_exists( pszImageDir ) )
    {
        std::cerr << "\nThe input directory doesn't exist" << std::endl;
        return ;
    }
    vector<string> vStr = stlplus::folder_files( pszImageDir );
    for(int i=0;i<vStr.size();++i)
    {
        string str = pszImageDir+vStr[i];
        vStrSrcFiles.push_back(str);
    }
}