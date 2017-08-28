
#ifndef _UAVMAPCACULATE_H_
#define _UAVMAPCACULATE_H_


#include<string>
#include<vector>

using namespace std;
/*
	影像块结构体
	影像行号，列号，分辨率等级
*/
struct MapUnit{
	int row;
	int col;
	int level;
	string unit_save;
  string unit_url;
};

//只做接口
class UAVMapCalculate
{
public:
	//根据中心经纬度和影像范围解算切片块
	vector<MapUnit> UAVMapCalculateUnit(double centerUTMx,double centerUTMy,double width,double height,int level);

  	//对于单个切片计算其角点坐标
	bool UAVMapUnitCorner(MapUnit unitMap,double &cornerx,double &cornery);

  	// 将切片数据组装一下
  	bool UAVMapUnitCombie(vector<MapUnit> units,string dest);

  	//根据url解析得到影像坐标并拼装起来
	virtual bool UAVMapUnitURL(vector<MapUnit> &units)=0;

	//根据切片块组合为url并保存到文件中
	virtual bool UAVMapUnitData(vector<MapUnit> units)=0;

};


class UAVMapCalculateGoogle:public UAVMapCalculate
{
public:
    UAVMapCalculateGoogle(){
        Google_URL="http://mt3.google.cn/vt/lyrs=s@110&hl=zh-CN&gl=cn&src=app&";
    }

    //直接根据全局变量得到影像对应的瓦片地图
    bool UAVMapGoogleRun();

    //根据url解析得到影像坐标并拼装起来
    virtual bool UAVMapUnitURL(vector<MapUnit> &units);

    //根据切片块组合为url并保存到文件中
    virtual bool UAVMapUnitData(vector<MapUnit> units);

private:
  string Google_URL;
};

#endif
