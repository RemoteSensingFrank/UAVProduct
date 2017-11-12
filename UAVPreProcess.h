#pragma once

#include"UAVInterface.h"
#include <thread>
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

			for(const auto iter:feature)
			{
				std::thread threadFunc( &UAVProcessFeatureSIFT::UAVProcessFeatExtractEach, this, iter.second);
				threadFunc.detach();
			}
			//目前没有更好的办法进行线程的同步饿了
			while(true)
			{
				if(iterNumber==feature.size())
					break;
			};
		}

		if(err!=0)
			return 5;
		else
			return 0;
	}
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);

protected:
	virtual UAVErr UAVProcessFeatExtractEach(FeatureParam fParam);

protected:
	int iterNumber = 0;	//c++11 标准
};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract(bool bThread);
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);
};