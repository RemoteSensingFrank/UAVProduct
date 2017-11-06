#pragma once
#include"UAVPreProcess.h"

class UAVProcessBundle:public UAVProcessFeatureSIFT
{
public:
	virtual UAVErr UAVProcessBundleGlobal(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
	virtual UAVErr UAVProcessBundleSquence(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
};

class UAVProcessBundleGpu:public UAVProcessFeatureSIFTGpu
{
public:
	virtual UAVErr UAVProcessBundleGlobal(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
	virtual UAVErr UAVProcessBundleSquence(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
};


class UAVProcessBundleGCPs:public UAVProcessBundleGpu
{
public:
	virtual UAVErr UAVProcessBindGCPs(std::string pGcps);

public:
	virtual UAVErr UAVProcessBundleGlobal(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
	virtual UAVErr UAVProcessBundleSquence(std::string dImage,std::string dFeatures,std::string pGPS,
		UAVProcessPOS* pPorc,CoordiListType typeCoordi,UAVCalibParams &cParam,
		std::string sfm_out);
}