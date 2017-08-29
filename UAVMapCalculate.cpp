#include"UAVMapCalculate.h"
#include "gdal_priv.h"
#include "UAVCommon.h"
#include "openMVG/sfm/sfm.hpp"
#include "Python.h"
#include "UAVXYZToLatLonWGS84.h"

#include <stdio.h>
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;
using namespace openMVG::image;

static  UAVXYZToLatLonWGS84 CooridinateTrans;



vector<MapUnit> UAVMapCalculate::UAVMapCalculateUnit(double centerUTMx,double centerUTMy,double width,double height,int level)
{
    Vec3 latlon1 = CooridinateTrans.UTMToLatLonWMT(centerUTMx-width/2-100,centerUTMy-height/2-100,0);
    Vec3 latlon2 = CooridinateTrans.UTMToLatLonWMT(centerUTMx+width/2+100,centerUTMy+height/2+100,0);

    int left, top;
    toTile(level,latlon1(0),latlon1(1),left,top);
    int right,bottom;
    toTile(level,latlon2(0),latlon2(1),right,bottom);
    vector<MapUnit> vec_mapUnits;
    for(int i=left;i<=right;++i){
      for(int j=bottom;j<=top;++j){
        MapUnit unit;
        unit.row=i;
        unit.col=j;
        unit.level=level;
        vec_mapUnits.push_back(unit);
      }
    }
    return vec_mapUnits;
}

bool UAVMapCalculate::UAVMapUnitCorner(MapUnit unitMap,double &cornerx,double &cornery)
{
  if(unitMap.level>18)
    return false;
  cornerx = MINXBound+Lods[unitMap.level][1]*unitMap.row*MAPUNITSIZE;
  cornery = MINYBound+Lods[unitMap.level][1]*unitMap.col*MAPUNITSIZE;

  return true;
}

bool UAVMapCalculate::UAVMapUnitCombie(vector<MapUnit> units,string dest)
{
    //
    int minx ,maxx,miny,maxy;
    minx=maxx=units[0].row;
    miny=maxy=units[0].col;

    for(int i=0;i<units.size();++i)
    {
      minx=min(minx,units[i].row);
      maxx=max(maxx,units[i].row);
      miny=min(miny,units[i].col);
      maxy=max(maxy,units[i].col);
    }

    int width = (maxx-minx+1)*MAPUNITSIZE;
    int height= (maxy-miny+1)*MAPUNITSIZE;
    unsigned char* dstData = NULL;
    try {
      dstData= new unsigned char[width*height];
    }catch(std::bad_alloc e){
      printf(e.what());
      return false;
    }


    //构建GDAL输出文件
    GDALAllRegister();
    double adfGeoTransform[6];
    GDALDatasetH m_dataset = GDALCreate(GDALGetDriverByName("GTiff"),dest.c_str(),width, height, 3, GDT_Byte, NULL);

    adfGeoTransform[0] = MINXBound+Lods[units[0].level][1]*units[0].row*MAPUNITSIZE;
    adfGeoTransform[1] = Lods[units[0].level][1];
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = MINYBound+Lods[units[0].level][1]*units[0].col*MAPUNITSIZE;
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = Lods[units[0].level][1];

    for (int k = 0; k <3 ; ++k) {

      for (int j = 0; j < units.size(); ++j) {
        GDALDatasetH m_datasrc = GDALOpen(units[j].unit_save.c_str(),GA_ReadOnly);
        //数据不大 不用做内存申请检查
        int widthsc  = GDALGetRasterXSize(m_datasrc);
        int heightsc = GDALGetRasterYSize(m_datasrc);
        unsigned char *data=new unsigned char[width*height];
        GDALRasterIO(GDALGetRasterBand(m_datasrc,k+1),GF_Read,0,0,widthsc,heightsc,data,widthsc,heightsc,GDT_Byte,0,0);

        int orix = (units[j].row-minx)*MAPUNITSIZE;
        int oriy = (units[j].col-miny)*MAPUNITSIZE;

        for (int l = 0; l < widthsc; ++l) {
          for (int n = 0; n < heightsc; ++n) {
            dstData[(oriy+n)*width+orix+l]=data[n*widthsc+l];
          }
        }
        delete[]data;data=NULL;
          GDALClose(m_datasrc);
      }
      GDALRasterIO(GDALGetRasterBand(m_dataset,k+1),GF_Write,0,0,width,height,dstData,width,height,GDT_Byte,NULL,NULL);
    }
    GDALSetGeoTransform(m_dataset,adfGeoTransform);
    GDALClose(m_dataset);
    if(dstData!=NULL)
    {
        delete []dstData;
        dstData=NULL;
    }
    return true;
}

//===================================================================================================================
bool UAVMapCalculateGoogle::UAVMapUnitURL(vector<MapUnit> &units)
{
  for(int i=0;i<units.size();++i)
  {
    char tile_url[256];
    sprintf(tile_url,"&x=%d&y=%d&z=%d",units[i].row,units[i].col,units[i].level);
    units[i].unit_url = Google_URL+string(tile_url);
    sprintf(tile_url,"./UAVProduct/Map/%d%d%d.jpg",units[i].row,units[i].col,units[i].level);
    units[i].unit_save = string(tile_url);
  }
    return true;

}

bool UAVMapCalculateGoogle::UAVMapUnitData(vector<MapUnit> units)
{
    for(int i=0;i<units.size();++i)
    {
        if(stlplus::file_exists(units[i].unit_save))
            continue;

        Py_Initialize();
        //PyRun_SimpleString("print 'hello'");
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.path.append('./UAVProduct/')");

        //导入模块
        PyObject *pModule = PyImport_ImportModule("GoogleMap");
        if (pModule==NULL)
        {
            printf("Python get module failed.\n");
            return 0;
        }

        //获取Insert模块内_add函数
        PyObject* pv = PyObject_GetAttrString(pModule, "GetImageForGoogleUrl");
        if (!pv || !PyCallable_Check(pv))
        {
            printf("Python get func failed.\n");
            return 0;
        }
        //初始化要传入的参数，args配置成传入两个参数的模式
        PyObject* args = PyTuple_New(2);
        //将Long型数据转换成Python可接收的类型
        PyObject* arg1 = PyString_FromString(units[i].unit_url.c_str());
        PyObject* arg2 =  PyString_FromString(units[i].unit_save.c_str());

        //将arg1配置为arg带入的第一个参数
        PyTuple_SetItem(args, 0, arg1);
        //将arg1配置为arg带入的第二个参数
        PyTuple_SetItem(args, 1, arg2);

        //传入参数调用函数，并获取返回值
        PyObject_CallObject(pv,args);

        Py_Finalize();
    }
    return true;
}

//封装得太好了反而失去灵活性了，但是调用方便
bool UAVMapCalculateGoogle::UAVMapGoogleRun()
{
    if(_info_._g_Has_Pos==false||_info_._g_Map_dir=="")
    {
        return false;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data, _info_._g_SFM_data, ESfM_Data(ALL)))
    {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << _info_._g_SFM_data << "\" cannot be read." << std::endl;
        return -1;
    }

    for (const auto & viewIter : sfm_data.GetViews())
    {
        const sfm::ViewPriors * prior = dynamic_cast<sfm::ViewPriors*>(viewIter.second.get());
        //中心点的坐标
        Vec3 position=prior->pose_center_;
        //Vec3 utmPosition = coordiTrans.LatLonToUTM(position(0),position(1),0);

        double focalx = _info_._g_focal_x;
        double focaly = _info_._g_focal_y;
        //计算范围
        Vec3 pnt0=CooridinateTrans.XYZToLatLon(position(0),position(1),position(2));
        Vec3 wmt0=CooridinateTrans.LatLonToUTMWMT(pnt0(0),pnt0(1),pnt0(2));
        double xrange = pnt0(2)*(prior->ui_width) /focalx;
        double yrange = pnt0(2)*(prior->ui_height)/focaly;
        vector<MapUnit> vec_mapUnits;
        vec_mapUnits = UAVMapCalculateUnit(wmt0(0),wmt0(1),xrange,yrange,17);

        if(vec_mapUnits.empty())
            continue;
        else
        {
            if(!UAVMapUnitURL(vec_mapUnits))
                continue;
            if(!UAVMapUnitData(vec_mapUnits))
                continue;
            if(!UAVMapUnitCombie(vec_mapUnits,_info_._g_Map_dir+prior->s_Img_path+string(".tif")))
                continue;
        }
    }

    return true;
}