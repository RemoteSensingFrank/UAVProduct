#pragma once
#include"common_header.h"
#include "openMVG/numeric/numeric.h"
#include<stdexcept>
#include<string>
#include<map>
#include <types.hpp>

typedef std::map<int,UAVPOSSt> POSPair;
typedef openMVG::Pair_Set MatchesList;
#define UndefinedCalibParam(cParam)  (memset(&cParam,0,sizeof(UAVCalibParams)))
#define IsUndefineCalibParam(cParam) (cParam._flen_x_&&cParam._flen_x_&&cParam._ppx_&&cParam._ppy_)

class UAV
{
public:
	void SoftwareInfo()
	{
		printf("developers: Wu Wei\n");
		printf("descriptior: Wu Wei\n");
		printf("github page: Wu Wei\n");
		printf("blog page: Wu Wei\n");
		printf("library cited: Wu Wei\n");
		printf("protocol: Wu Wei\n");
	}

	void LogThread(std::string logMsg);
};


//����POS����
class UAVProcessPOS:public UAV
{
public:
	virtual UAVErr UAVPorcessPOSGet(std::string file,bool bGps)	= 0;
	virtual UAVErr UAVProcessExport(std::string file,bool rLoc)	= 0;

	virtual UAVErr UAVProcessPOSExtractUTM(double &centerx,double &centery,double &centerz);
	virtual UAVErr UAVProcessPOSExtractXYZ(double &centerx,double &centery,double &centerz);
	virtual UAVErr UAVProcessPOSExtractLocal(double &centerx,double &centery,double &centerz);
public:
	POSPair posList;
};

//list ����
class UAVProcessList:public UAV
{
public:
	virtual UAVErr UAVProcessListGet(std::string image_dir,std::string sfm_out,
									 UAVCalibParams &cParam,EINTRINSIC camera_model_type,bool group_camera_model,
									 std::string pos_file,UAVProcessPOS* pPorc,CoordiListType typeCoordi);
	virtual UAVErr UAVProcessListRange(UAVCalibParams cParam,POSPair posList){return 0;}
	virtual double UAVProcessListSize(std::string dImage){return 0;}
};

class UAVProcessDense:public UAV
{
public:
	virtual UAVErr UAVProcessPointCloud(std::string pointClouds,std::string sfm_data);
	virtual UAVErr UAVProcessPointGrid(std::string pointClouds,std::string grid);
	virtual UAVErr UAVProcessMesh(std::string pointClouds,std::string grid);
};

class UAVProcessGeometry:public UAV
{
public:
	static openMVG::Vec3 UAVProcessGeoBLHToXYZ(double lat,double lon, double alt);
    static openMVG::Vec3 UAVProcessGeoXYZToBLH(double x,double y,double z);

    static openMVG::Vec3 UAVProcessGeoLatLonToXYZ(double lat,double lon,double alt);
    static openMVG::Vec3 UAVProcessGeoLatLonToUTM(double lat,double lon,double alt);
    static openMVG::Vec3 UAVProcessGeoXYZToLatLon(double x,double y,double z);

	//web Mector
    static openMVG::Vec3 UAVProcessGeoUTMToLatLonWMT(double x,double y,double z);
    static openMVG::Vec3 UAVProcessGeoLatLonToUTMWMT(double lat,double lon,double alt);

	//GCJ01
    static bool OutOfChina(double lat, double lon);
    static void UAVProcessGeoWGSLatLonToGCJ(double wgLon,double wgLat,double &mgLon,double &mgLat);
};