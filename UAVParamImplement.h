#pragma once
#include<map>
#include<vector>
#include "UAVInterface.h"

class UAVProcFeatureGpuSIFT;
class UAVProcFeatureCpuSIFT;
class UAVProcMatchGpuSIFT;
class UAVProcMatchCpuSIFT;

/**
 * POS param base class
 */
class UAVParamPos:UAVParam
{
public:
    //import parameters
    virtual UAVErr UAVProcParamImport(const char* pathParam);

    //export parameters
    virtual UAVErr UAVProcParamExport(const char* pathParam);

    //check the parameters
    virtual bool UAVProcParamCheck();

    virtual UAVErr UAVProcParamPos_Get(FILE* file) = 0;

public:
    double dL;
    double dB;
    double dH;
    double dRoll;
    double dPitch;
    double dHeading;
};

class UAVParamPos1: public UAVParamPos{
public:
    virtual UAVErr UAVProcParamPos_Get(FILE* file);
};

/**
 * Pre process param base class
 */
class UAVParamPre: public UAVParam
{
public:
	UAVErr UAVParamPreSet(UAVCalibParams calibParam,std::string imageDir);
	UAVErr UAVParamPreGet(UAVCalibParams &calibParam,std::string &imageDir);

	std::string _path_imagedir_;
	UAVCalibParams _calib_param_;	
};

/**
 * List image param base class
 */
class UAVParamList:public UAVParamPre
{
public:
    virtual UAVErr UAVProcParamImport(const char* pathParam);
    virtual UAVErr UAVProcParamExport(const char* pathParam);
    virtual bool UAVProcParamCheck();

public:
	std::string _gps_file_;
};

/**
 * Feature extract base class
 */
class UAVParamFeature:public UAVParamPre
{
	friend class UAVProcFeatureGpuSIFT;
	friend class UAVProcFeatureCpuSIFT;
	friend class UAVProcMatchGpuSIFT;
	friend class UAVProcMatchCpuSIFT;

public:
	struct FeatureParam
	{
		std::string _image_in_;
		std::string _feature_out_;
	};

    virtual UAVErr UAVProcParamImport(const char* pathParam);
    virtual UAVErr UAVProcParamExport(const char* pathParam);
    virtual bool UAVProcParamCheck();

	UAVErr UAVParamFeatureInsert(int viewIdx,std::string image,std::string feature);
	UAVErr UAVParamFeatureSearch(int &viewIdx,std::string image,std::string feature);

	std::map<int,FeatureParam> _param_features_;
};

/**
 * Matches base class
 */
class UAVParamMatch:public UAVParamPre
{
	friend class UAVProcMatchGpuSIFT;
	friend class UAVProcMatchCpuSIFT;

public:

	UAVErr UAVParamMatchSet(std::string matches,std::map<int,int> _params);
	UAVErr UAVParamMatchInsert(int src,int dst);
	std::vector<int> UAVParamMatchSearch(int srcIdx);

	std::string _match_file_;
	std::map<int,int> _param_match_;
};