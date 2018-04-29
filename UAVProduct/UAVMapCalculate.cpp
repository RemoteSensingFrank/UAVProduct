
#include <gdal.h>
#include "openMVG/sfm/sfm.hpp"
#include "gdal_priv.h"
#include "Python.h"
#include "UAVProcessGeometry.h"

inline void toTile(int zoom,double Lng,double lat,int &x,int &y) {
    double n = pow(2, zoom);
    double tileX = ((Lng + 180) / 360) * n;
    double tileY = (1 - (log(tan(lat*M_PI/180) + (1 / cos(lat*M_PI/180))) / M_PI)) / 2 * n;
    x=tileX;
    y=tileY;
}

void toLnglat(int zoom,double &lng,double &lat,int x,int y) {
    double n = pow(2, zoom);
    lng =x / n * 360.0 - 180.0;
    lat = atan(sinh(M_PI * (1 - 2 * y / n)));
    lat = lat * 180.0 / M_PI;
}

MapCalculateUnits UAVMapCalculate::UAVMapCalculateUnit(double centerUTMx,double centerUTMy,double width,double height,int level)
{
    int left, top;
    openMVG::Vec3 latlonwgs1 = UAVProcessGeometry::UAVProcessGeoUTMToLatLonWMT(centerUTMx-width/2,centerUTMy-height/2,0);
    if(!UAVProcessGeometry::OutOfChina(latlonwgs1(1),latlonwgs1(0)))
    {
        double lat,lon;
        UAVProcessGeometry::UAVProcessGeoWGSLatLonToGCJ(latlonwgs1(0),latlonwgs1(1),lon,lat);
        toTile(level,lon,lat,left,top);

    }else{
        toTile(level,latlonwgs1(0),latlonwgs1(1),left,top);
    }

    int right,bottom;
    openMVG::Vec3 latlonwgs2 = UAVProcessGeometry::UAVProcessGeoUTMToLatLonWMT(centerUTMx+width/2,centerUTMy+height/2,0);
    if(!UAVProcessGeometry::OutOfChina(latlonwgs2(1),latlonwgs2(0)))
    {
        double lat,lon;
        UAVProcessGeometry::UAVProcessGeoWGSLatLonToGCJ(latlonwgs2(0),latlonwgs2(1),lon,lat);
        toTile(level,lon,lat,right,bottom);

    }else{
        toTile(level,latlonwgs2(0),latlonwgs2(1),right,bottom);
    }

    MapCalculateUnits vec_mapUnits;
    for(int i=left;i<=right;++i){
        for(int j=bottom;j<=top;++j){
            MAPUNIT unit;
            unit.row=i;
            unit.col=j;
            unit.level=level;
            vec_mapUnits.push_back(unit);
        }
    }
    return vec_mapUnits;
}

bool UAVMapCalculate::UAVMapUnitCorner(MAPUNIT unitMap,double &cornerx,double &cornery)
{
      if(unitMap.level>18)
        return false;
      cornerx = MINXBound+Lods[unitMap.level][1]*unitMap.row*MAPUNITSIZE;
      cornery = MINYBound+Lods[unitMap.level][1]*unitMap.col*MAPUNITSIZE;

      return true;
}
bool UAVMapCalculate::UAVMapUnitGCPs(MAPUNIT unitMap,int xsize,int ysize,GDAL_GCP *gcps)
{
    double x[5] = {0,xsize,0,xsize,xsize/2};
    double y[5] = {0,ysize,ysize,0,ysize/2};

    for(int i=0;i<5;++i){
        double wMectorx = MINXBound+Lods[unitMap.level][1]*(unitMap.row*MAPUNITSIZE+x[i]);
        double wMectory = MAXYBound-Lods[unitMap.level][1]*(unitMap.col*MAPUNITSIZE+y[i]);

        openMVG::Vec3 latlng = UAVProcessGeometry::UAVProcessGeoUTMToLatLonWMT(wMectorx,wMectory,0);
        openMVG::Vec3 UTM    = UAVProcessGeoLatLonToUTM(latlng(1),latlng(0),latlng(2));
        std::string temp_str=std::to_string(i);
        gcps[i].pszId = "";
        gcps[i].pszInfo = "";
        gcps[i].dfGCPPixel = x[i];
        gcps[i].dfGCPLine  = y[i];
        gcps[i].dfGCPX     = UTM(0);
        gcps[i].dfGCPY     = UTM(1);
        gcps[i].dfGCPZ     = UTM(2);
    }
    return true;
}

bool UAVMapCalculate::UAVMapUnitCombie(MapCalculateUnits units,std::string dest)
{
      //
      int minx ,maxx,miny,maxy;
      minx=maxx=units[0].row;
      miny=maxy=units[0].col;

      for(size_t i=0;i<units.size();++i)
      {
          minx=std::min(minx,units[i].row);
          maxx=std::max(maxx,units[i].row);
          miny=std::min(miny,units[i].col);
          maxy=std::max(maxy,units[i].col);
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
          for (size_t j = 0; j < units.size(); ++j) {
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
bool UAVMapCalculateGoogle::UAVMapUnitURL(MapCalculateUnits &units)
{
     for(size_t i=0;i<units.size();++i)
     {
         char tile_url[256];
         sprintf(tile_url,"&x=%d&y=%d&z=%d",units[i].row,units[i].col,units[i].level);
         units[i].unit_url = Google_URL+std::string(tile_url);
         sprintf(tile_url,"./UAVProduct/Map/%d%d%d.jpg",units[i].row,units[i].col,units[i].level);
         units[i].unit_save = std::string(tile_url);
     }
     return true;
}

bool UAVMapCalculateGoogle::UAVMapUnitData(MapCalculateUnits units)
{
      for(size_t i=0;i<units.size();++i)
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
bool UAVMapCalculateGoogle::UAVMapGoogleRun(std::string sfm,std::string dMap)
{
      openMVG::sfm::SfM_Data sfm_data;
      if (!Load(sfm_data, sfm, openMVG::sfm::ESfM_Data(openMVG::sfm::ALL)))
      {
            std::cerr << std::endl
                    << "The input SfM_Data file \"" << sfm << "\" cannot be read." << std::endl;
            return -1;
      }

      for (const auto & viewIter : sfm_data.GetViews())
      {
            const openMVG::sfm::ViewPriors * prior = dynamic_cast<openMVG::sfm::ViewPriors*>(viewIter.second.get());
            //中心点的坐标
            openMVG::Vec3 position=prior->pose_center_;
            //Vec3 utmPosition = coordiTrans.LatLonToUTM(position(0),position(1),0);
            if(!prior->b_use_pose_center_)
                continue;
            int ind_ptr = prior->id_intrinsic;
           const openMVG::cameras::Pinhole_Intrinsic* intrinsic =dynamic_cast<openMVG::cameras::Pinhole_Intrinsic*> (sfm_data.GetIntrinsics().at(ind_ptr).get());
            double focalx = (intrinsic)->K()(0,0);
            double focaly = (intrinsic)->K()(1,1);
            //计算范围
            openMVG::Vec3 pnt0=UAVProcessGeometry::UAVProcessGeoXYZToLatLon(position(0),position(1),position(2));
            openMVG::Vec3 wmt0=UAVProcessGeometry::UAVProcessGeoLatLonToUTMWMT(pnt0(0),pnt0(1),pnt0(2));
            double xrange = pnt0(2)*(prior->ui_width) /focalx;
            double yrange = pnt0(2)*(prior->ui_height)/focaly;
            MapCalculateUnits vec_mapUnits;
            vec_mapUnits = UAVMapCalculateUnit(wmt0(0),wmt0(1),xrange,yrange,19);

            if(vec_mapUnits.empty())
                continue;
            else
            {
                if(!UAVMapUnitURL(vec_mapUnits))
                    continue;
                if(!UAVMapUnitData(vec_mapUnits))
                    continue;
                if(!UAVMapUnitCombie(vec_mapUnits,dMap+prior->s_Img_path+".tif"))
                  continue;
          }
     }

     return true;
}
