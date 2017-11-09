#pragma once

#include"UAVInterface.h"

class UAVProcessPOSSimple:public UAVProcessPOS
{
public:
	virtual UAVErr UAVPorcessPOSGet(std::string file,bool bGps);
	virtual UAVErr UAVProcessExport(std::string file,bool rLoc);
};

class UAVProcessFeatureSIFT:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessMatchesList(std::string imageList,int neighbor_count,MatchesList &list);
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(MatchesList list,std::string pMatchData);
};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessMatchesList(std::string imageList,int neighbor_count,MatchesList &list);
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(MatchesList list,std::string pMatchData);
};