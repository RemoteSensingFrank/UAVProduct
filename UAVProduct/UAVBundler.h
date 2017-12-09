#pragma once
#include"UAVPreProcess.h"
class UAVProcessGCPs;

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
	virtual UAVErr UAVProcessBindGCPs(std::string pGcps,std::shared_ptr<UAVProcessGCPs> ptrGcps,COORDITRANSFNC coordiTransFnc,CoordiListType cooridTpGcp,CoordiListType cooridTpPos);

	virtual UAVErr UAVProcessBundleGlobal(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);

	virtual UAVErr UAVProcessBundleSquence(std::shared_ptr<UAVProcessFeature> feats_ptr,std::string mathces,std::string sfm_in,std::string sfm_dout);
private:
    UAVErr UAVProcessBindGCPs(openMVG::sfm::SfM_Data &sfm_data);

private:
    openMVG::sfm::Landmarks m_gcps;
};