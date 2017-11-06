#pragma once

#include"UAVInterface.h"

class UAVProcessPOSSimple:public UAVProcessPOS
{
public:
	virtual UAVErr UAVPorcessPOSGet(std::string file,bool bGps)	= 0;
	virtual UAVErr UAVProcessExport(std::string file,bool rLoc)	= 0;
};

class UAVProcessFeatureSIFT:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(MatchesList list,std::string pMatchData);
};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(MatchesList list,std::string pMatchData);
};