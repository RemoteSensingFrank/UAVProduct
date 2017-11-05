#pragma once
#include"common_header.h"

#include<stdexcept>
#include<string>
#include<iostream>

//#include<pthread>
//#include<mutex>

//exception exception

class UAV
{
public:
	void SoftwareInfo()
	{
		printf("developers: Wu Wei\n");
		printf("descriptior: Wu Wei\n");
		printf("github page: Wu Wei\n");
		printf("blog page: Wu Wei\n");
		printf("library cited: Wu Wei\n");
		printf("protocol: Wu Wei\n");
	}
};

class UAVException :public UAV
{    
public:    
	virtual const char* what() const throw() = 0;   
}; 

//parameters interface
class UAVParam :public UAV
{
public:
	//import parameters
    virtual UAVErr UAVProcParamImport(const char* pathParam) = 0;

	//export parameters
    virtual UAVErr UAVProcParamExport(const char* pathParam) = 0;

	//check the parameters
    virtual bool UAVProcParamCheck() = 0;

public:
    std::string _sfm_data_in_;
    std::string _sfm_data_out_;
};

//process interface 
class UAVProcess:public UAV
{
public:
	//message
	virtual UAVErr UAVProcMsg(std::string &msg,std::ostream &stdoutDevice){
        stdoutDevice<<msg<<std::endl;
    }

	//parameter
	virtual UAVErr UAVProcParam(UAVParam* param)=0;
	//serial process
	virtual UAVErr UAVProcProcess(UAVProgressFunc pfnProgress,void* pProgressArg,bool bParallel=false) = 0;

protected:
	//TODO:
	//parallel process
	//virtual UAVErr UAVProcProcessThread(void *args) = 0;

	virtual UAVErr UAVProcProcessClean() = 0;

protected:
	//TODO:
	//std::mutex _process_mutex_;
	//std::thread _process_thread_;
};

//factory mode
class UAVFactory:public UAV
{
public:
	//virtual facotry 
	virtual UAVParam*		CreatorParameter(PARAMTYPE type);
	virtual UAVProcess*		CreatorProcess(PROCTYPE type);
};

