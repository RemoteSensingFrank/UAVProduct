#pragma once

#include"UAVInterface.h"
#include "SiftGPU/SiftGPU.h"
#include "openMVG/sfm/sfm.hpp"

#include <fstream>
#include <string>
using namespace std;

struct Features_Provider_Gpu:public openMVG::sfm::Features_Provider
{
	/***
     * 导入GPU解算得到的特征点
     * @param viewID
     * @param sFeats
     * @return
     */
	virtual bool load_pre(int viewID,std::string sFeats)
	{
		const std::string featFile = sFeats;

		//导入特征点
		//通过读取二进制文件的方式导入
		if (!stlplus::file_exists(featFile))
		{
			return false;
		}
		ifstream ifs(featFile,ios_base::in|ios_base::binary);
		int num_feats;
		ifs.read((char*)&num_feats,sizeof(int));
		float* points = new float[2*num_feats];
		ifs.read((char*)points,sizeof(float)*2*num_feats);
		openMVG::features::PointFeatures vec_points;
		for (int i = 0; i < num_feats; ++i) {
			vec_points.push_back( openMVG::features::PointFeature(points[2*i+0],points[2*i+1]));
		}
		feats_per_view[viewID] =  vec_points;
		delete[]points;points=NULL;
		return true;
	}
};

struct Features_Provider_Cpu:public openMVG::sfm::Features_Provider
{
	virtual bool load_pre(int viewID,std::string sFeats)
	{
		const std::string featFile = sFeats;

		//导入特征点
		//通过读取二进制文件的方式导入
		if (!stlplus::file_exists(featFile))
		{
			return false;
		}
		std::unique_ptr<openMVG::features::Regions> regions(new openMVG::features::SIFT_Regions);
		if (!stlplus::file_exists(featFile) || !regions->LoadFeatures(featFile))
		{
			return false;
		}
		feats_per_view[viewID] = regions->GetRegionsPositions();
		return true;
	}
};


class UAVProcessPOSSimple:public UAVProcessPOS
{
public:
	virtual UAVErr UAVPorcessPOSGet(std::string file,bool bGps);
	virtual UAVErr UAVProcessExport(std::string file,bool rLoc);
};

//匹配
class UAVProcessMatches:public UAVProcessList
{
public:
	virtual UAVErr UAVProcessMatchesList(std::string imageList,int neighbor_count,bool bGeo,std::string pMatch);
	virtual void UAVProcessAdjacencyMatrixToSVG(const size_t NbImages,
												const MatchesList & corresponding_indexes,
												const std::string & sOutName);
	virtual UAVErr UAVProcessMatchesExport(MatchesList list,std::string pMatch);
	virtual UAVErr UAVProcessMatchesImport(MatchesList &list,std::string pMatch);

protected:
	struct FeatureParam
	{
		std::string _image_in_;
		std::string _feature_out_;
		std::string _descs_out_;
	};
	std::map<int,FeatureParam> feature;
};

class UAVProcessFeature:public UAVProcessMatches
{
	friend class UAVProcessBundle;
public:
	virtual UAVErr UAVProcessFeatList(std::string sfm_data_path,std::string feature_dir);
	//later I will add some parameter to control the number of the sift feature number
	virtual UAVErr UAVProcessFeatExtract() = 0;
	virtual unique_ptr<openMVG::sfm::Features_Provider> UAVProcessFeatsProvide()=0;
protected:
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData)=0;
	std::string sfm_data_path_;
};

class UAVProcessFeatureSIFT:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract();
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);
	virtual unique_ptr<openMVG::sfm::Features_Provider> UAVProcessFeatsProvide();
};

class UAVProcessFeatureSIFTGpu:public UAVProcessFeature
{
public:
	virtual UAVErr UAVProcessFeatExtract();
	virtual UAVErr UAVProcessMatchesExtract(std::string pMatchList,std::string pMatchData);
	virtual unique_ptr<openMVG::sfm::Features_Provider> UAVProcessFeatsProvide();
protected:
	virtual UAVErr UAVProcessExportFeatsToFile(std::string pathfeats,std::string pathdesc, std::vector<SiftKeypoint> feats, std::vector<float> desc);
	bool UAVImportFeatsToFile(std::string pathdesc, std::vector<float> &desc);
	bool UAVExportMatchesToFile(std::string path,int srcImg,int desImg,int matchnum,int (*matches)[2]);
};