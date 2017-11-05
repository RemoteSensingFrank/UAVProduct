#pragma once

#include"UAVInterface.h"
#include"common_header.h"
#include"UAVParamImplement.h"

typedef std::map<int,UAVParamPos*> POSParams;

class UAVProcPos:public UAVProcess
{
public:
    UAVProcPos();
    ~UAVProcPos();

    //get pos from file
    virtual UAVErr UAVProcPos_GetRaw(POSParams poses,UAVProgressFunc pfnProgress,void* pProgressArg);

    //extract pos and get relative coordinate
    virtual UAVErr UAVProcPos_ExtractReletative(POSParams posRaw,POSParams &posRel);

    //parameter
    virtual UAVErr UAVProcParam(UAVParam* param);

    //serial process
    virtual UAVErr UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel=false);

protected:
    //TODO:
    //parallel process
    //virtual UAVErr UAVProcProcessThread(void *args) = 0;

    virtual UAVErr UAVProcProcessClean();

protected:
    UAVParamList *_list_params_;
};

class UAVProcList:public UAVProcPos
{
public:
    //get size of all images
	virtual double UAVProcListSize();

    //list range in kml files
	virtual UAVErr UAVProcListRange(const char* pathKML);
    virtual UAVErr UAVProcListFeature(std::string feature_dir,UAVParamFeature &feature_param);

	virtual UAVErr UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel=false);
};

class UAVProcFeature:public UAVProcList
{
public:
    UAVProcFeature();
    ~UAVProcFeature();

    virtual UAVErr UAVProcFeatureList(std::string feature_dir);
    virtual UAVErr UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel=false)=0;

protected:
    UAVParamFeature* param;
};

class UAVProcFeatureSIFT:public UAVProcFeature
{
public:
    virtual UAVErr UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel=false);
};

