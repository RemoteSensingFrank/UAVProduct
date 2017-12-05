//
// Created by wuwei on 17-4-2.
//

#ifndef UAVPRODUCT_UAVGEOMOSAIC_H
#define UAVPRODUCT_UAVGEOMOSAIC_H

#include "UAVInterface.h"
#include "gdal_priv.h"
#include "gdalwarper.h"
#include "gdal_alg_priv.h"
#include "gdal_alg.h"
#include "ogrsf_frmts.h"

#include <iostream>

#include<vector>
#include<string>

using namespace std;

class UAVProcessGeoMosaicGDAL: public UAVProcessGeometry
{
private:

    //图像范围
    double	       dfMinX, dfMinY, dfMaxX, dfMaxY;
    //图像分辨率
    double	       dfXRes, dfYRes;
    //图像宽高
    int             nForcePixels, nForceLines;
    //图像是否具有Alpha通道
    int             bEnableDstAlpha, bEnableSrcAlpha;

public:
    UAVProcessGeoMosaicGDAL() :dfMinX(0.0), dfMinY(0.0), dfMaxX(0.0), dfMaxY(0.0), dfXRes(0.0), dfYRes(0.0),
                 nForceLines(0), nForcePixels(0), bEnableDstAlpha(FALSE), bEnableSrcAlpha(FALSE)
    {

    }

    /**
    转换投影坐标到图像行列号坐标
    用于图像裁切中，按照AOI来裁切时转换AOI坐标到图像行列号坐标
    **/
    class CutlineTransformer : public OGRCoordinateTransformation
    {
    public:

        /**
        * @brief 源图像转换参数
        */
        void *hSrcImageTransformer;

        /**
        * @brief 获取源数据空间参考坐标系
        * @return 返回源数据空间参考坐标系
        */
        virtual OGRSpatialReference *GetSourceCS() { return NULL; }

        /**
        * @brief 获取目标数据空间参考坐标系
        * @return 返回目标数据空间参考坐标系
        */
        virtual OGRSpatialReference *GetTargetCS() { return NULL; }

        /**
        * @brief 批量转换投影坐标到行列号
        * @param nCount		坐标点个数
        * @param x			x坐标串
        * @param y			y坐标串
        * @param z			z坐标串
        * @return 返回是否执行正确
        */
        virtual int Transform(int nCount, double *x, double *y, double *z = NULL)
        {
            int nResult;

            int *pabSuccess = (int *)CPLCalloc(sizeof(int), nCount);
            nResult = TransformEx(nCount, x, y, z, pabSuccess);
            CPLFree(pabSuccess);

            return nResult;
        }

        /**
        * @brief 批量转换投影坐标到行列号
        * @param nCount		坐标点个数
        * @param x			x坐标串
        * @param y			y坐标串
        * @param z			z坐标串
        * @param pabSuccess	保存执行成功的数组
        * @return 返回是否执行正确
        */
        virtual int TransformEx(int nCount, double *x, double *y,
                                double *z = NULL, int *pabSuccess = NULL)
        {
            return GDALGenImgProjTransform(hSrcImageTransformer, TRUE,
                                           nCount, x, y, z, pabSuccess);
        }
    };


    /*
    功能：获取文件名除去后缀
    参数：1.const char* pszFile：输入影像名称
    */
    inline string UAVGeoMosaic_GetFileName(const char* pszFile);


    /*
    功能：创建输出文件
    参数：1.char **papszSrcFiles：输入文件列表
    2.const char *pszFilename  ：输出文件路径
    3.const char *pszFormat    : 输出文件格式
    4.char **papszTO		    : 转换选项
    5.char ***ppapszCreateOptions: 创建文件选项
    6.GDALDataType eDT			：创建文件数据类型
    */
    GDALDatasetH UAVGeoMosaic_GDALWarpCreateOutput(char **papszSrcFiles, const char *pszFilename,
                                               const char *pszFormat, char **papszTO,
                                               char ***ppapszCreateOptions, GDALDataType eDT);


    /*
    功能：转换AOI到源文件之间的行列号
    参数：1.GDALDatasetH hSrcDS ：输入文件GDAL数据集句柄
    2.void *hCutline      ：裁切的几何形状
    3.char ***ppapszWarpOptionst: 转换选项，用于配置裁切参数
    4.char **papszTO		     : 转换选项
    5.char **papszTO_In		 : 转换选项
    */
    void UAVGeoMosaic_TransformCutlineToSource(GDALDatasetH hSrcDS, void *hCutline,
                                           char ***ppapszWarpOptions, char **papszTO_In);


    /*
    功能：加载镶嵌块
    参数：1.const char *pszCutlineDSName ：镶嵌块文件
    2.void *hCutline       ：裁切的几何形状
    3.const char *pszCLayer: 镶嵌块图层名称，可以为NULL
    4.const char *pszCWHERE: 镶嵌块过滤字段
    5.const char *pszCSQL	: 镶嵌块SQL过滤字段
    6.void **phCutlineRet : 返回的镶嵌块数据指针
    */
    long UAVGeoMosaic_LoadCutline(const char *pszCutlineDSName, const char *pszCLayer,
                              const char *pszCWHERE, const char *pszCSQL,
                              void **phCutlineRet);


    //获取影像数据集
    void UAVGeoMosaic_GetMosaicVector(string pszImageDir,vector<string> &vStrSrcFiles);

    /*
    功能：影像镶嵌
    * 注意事项，调用此函数之前，请先对影像进行几何纠正到同一投影坐标系统下，分辨率可以不同，
    * 但是投影信息以及输入的各个数据的波段个数必须一致，否则会出现不能正常完成镶嵌操作。
    * 图像的分辨率会按照输入影像的第一个影像获取，包括投影等信息					*
    * 函数参数：vector<string> vStrSrcFiles ：输入文件以及镶嵌线路径数组，			*
    默认第一个为参考影像，第一个图像在最下层，后面的依次向上				*
    *          const char* pszCutLineFile  ：输入镶嵌块文件文件路径数组				*
    *		   const char* pszOutFile: 输出文件路径									*
    *		   GDALResampleAlg eResampleMethod: 重采样方式							*
    *		   const char *pszCSQL	: 镶嵌块SQL过滤字段								*
    *		   const char *pszFormat:输出文件格式，详细参考GDAL支持数据类型			*
    */
    long UAVGeoMosaic_ImageMosaicing(vector<string> vStrSrcFiles, const char* pszCutLineFile, const char* pszOutFile,
                                 GDALResampleAlg eResampleMethod, const char *pszFormat);


    //TODO:影像拼接的过程需要添加拼接线处理和影像色调调整的步骤，
    // 拼接线具体的处理方式可以参考openCV对于拼接线处理的方式(主要问题)
    // 色调调整的方法可以参考openMVG的色调调整方法
};


#endif //UAVPRODUCT_UAVGEOMOSAIC_H
