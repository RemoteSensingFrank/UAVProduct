#pragma once

#include"UAVInterface.h"
#include <thread>
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
            UAVProcessThreadPool threadPool(2);
			std::vector<std::future<UAVErr>> errReturn;
            for(const auto iter:feature)
			{
                errReturn.emplace_back(threadPool.UAVProcess_Enterqueue(
                        [](FeatureParam param){UAVProcessFeatureSIFT::UAVProcessFeatExtractEach(param);}
                        ,&iter.second
                ));
			}
            for (auto res:errReturn) {
                err+=res.get();
            }
		}

		if(err!=0)
			return 5;
		else
			return 0;
	}
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);

protected:
	virtual UAVErr UAVProcessFeatExtractEach(FeatureParam fParam);

};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);
};