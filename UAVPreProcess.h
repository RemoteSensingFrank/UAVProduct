#pragma once

#include"UAVInterface.h"
#include <thread>
#include <mutex>
#include "UAVProcessThreadPool.h"

class UAVProcessPOSSimple:public UAVProcessPOS
{
public:
	virtual UAVErr UAVPorcessPOSGet(std::string file,bool bGps);
	virtual UAVErr UAVProcessExport(std::string file,bool rLoc);
};

class UAVProcessFeatureSIFT:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread){
		if(feature.size()<=0)
			return 5;

		UAVErr err=0;
		if(!bThread)
		{
			for(const auto iter:feature){
				err=err|UAVProcessFeatExtractEach(iter.second);
			}
		} else{
            UAVProcessThreadPool threadPool(4);
            for(const auto iter:feature)
			{
                threadPool.UAVProcess_Enterqueue(&UAVProcessFeatureSIFT::UAVProcessFeatExtractEach,this,iter.second);
			}
		}
        return 0;
	}
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);

protected:
	virtual UAVErr UAVProcessFeatExtractEach(FeatureParam fParam);

    int threadProcNumber = 0;
    std::mutex exportFile_lock;
};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);
};