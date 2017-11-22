#pragma once
#include"UAVPreProcess.h"

class UAVProcessBundle:public UAV
{
public:
	virtual UAVErr UAVProcessBundleGlobal(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);

	virtual UAVErr UAVProcessBundleSquence(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);

    virtual UAVErr UAVProcessBundleToMVS(std::string sfm,std::string mvs);
};


class UAVProcessBundleGCPs:public UAVProcessBundle
{
public:
	virtual UAVErr UAVProcessBindGCPs(std::string pGcps);

public:
	virtual UAVErr UAVProcessBundleGlobal(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);

	virtual UAVErr UAVProcessBundleSquence(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);

};