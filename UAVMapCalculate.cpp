#include"UAVMapCalculate.h"

const double Lods[19][3]={
{0,156543.03392800014,591657527.591555},
{1,78271.516963999937,295828763.79577702},
{2,39135.758482000092,147914381.89788899},
{3,19567.879240999919,73957190.948944002},
{4,9783.9396204999593,36978595.474472001},
{5,4891.9698102499797,18489297.737236001},
{6,2445.9849051249898,9244648.8686180003},
{7,1222.9924525624949,4622324.4343090001},
{8,611.49622628138,2311162.217155},
{9,305.748113140558,1155581.108577},
{10,152.874056570411,577790.554289},
{11,76.4370282850732,288895.277144},
{12,38.2185141425366,144447.638572},
{13,19.1092570712683,72223.819286},
{14,9.55462853563415,36111.909643},
{15,4.7773142679493699,18055.954822},
{16,2.3886571339746849,9027.9774109999998},
{17,1.1943285668550503,4513.9887049999998},
{18,0.59716428355981721,2256.994353}};

vector<MapUnit> UAVMapCalculate::UAVMapCalculateUnit(double centerUTMx,double centerUTMy,double width,double height,int level)
{
  int left = (centerUTMx-width/2-MINXBound)/Lods[level][2]/MAPUNITSIZE;
  int top  = (centerUTMy-height/2-MINYBound)/Lods[level][2]/MAPUNITSIZE;

  int right  = (centerUTMx+width/2-MINXBound)/Lods[level][2]/MAPUNITSIZE;
  int bottom = (centerUTMy+height/2-MINYBound)/Lods[level][2]/MAPUNITSIZE;

  vector<MapUnit> vec_mapUnits;
  for(int i=left;i<=right;++i){
    for(int j=top;j<=bottom;++j){
      MapUnit unit;
      unit.row=i;
      unit.col=j;
      unit.level=level;
      vec_mapUnits.push_back(unit);
    }
  }
}

bool UAVMapCalculate::UAVMapUnitCorner(MapUnit unitMap,double &cornerx,double &cornery)
{
  if(unitMap.level>18)
    return false;
  cornerx = MINXBound+Lods[MapUnit.level][2]*Lods[unitMap.row]*MAPUNITSIZE;
  cornery = MINYBound+Lods[MapUnit.level][2]*Lods[unitMap.col]*MAPUNITSIZE;

  return true;
}

bool UAVMapCalculate::UAVMapUnitCombie(vector<MapUnit> units,string dest)
{
  //
  int minx ,maxx,miny,maxy;
  minx=maxx=units[0].row;
  miny=maxy=units[0].col;

  for(int i=0;i<units;++i)
  {
    minx=min(minx,units[i].row);
    maxx=min(maxx,units[i].row);
    miny=min(miny,units[i].col);
    maxy=min(maxy,units[i].col);
  }

  int width = (maxx-minx)*MAPUNITSIZE;
  int height= (maxy-miny)*MAPUNITSIZE;
  //构建GDAL输出文件
  GDALAllRegister();
  double adfGeoTransform[6];
  GDALDatasetH m_dataset = GDALCreate(GDALGetDriverByName("GTiff");dest.c_str(),width, height, 3, GDT_Byte, NULL);


  return true;
}

//===========================================================================================================================================
virtual bool UAVMapCalculateGoogle::UAVMapUnitURL(vector<MapUnit> &units)
{
  for(int i=0;i<units.size();++i)
  {
    char tile_url[256];
    sprintf(tile_url,"&x=%d&y=%d&z=%d",units[i].row,units[i].col,units[i].level);
    units[i].unit_url = Google_URL+string(tile_url);
    sprintf(tile_url,"&x=%d&y=%d&z=%d.jpg",units[i].row,units[i].col,units[i].level);
    units[i].unit_save = Google_URL+string(tile_url);
    return true;
  }

}
