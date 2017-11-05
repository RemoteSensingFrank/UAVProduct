#pragma once
#include "UAVInterface.h"

//parameters factory
class UAVParamFactory: public UAVFactory
{
public:
	virtual UAVParam*	CreatorParameter();
};

class UAVProcFactory: public UAVFactory
{
public:
	virtual UAVParam*	CreatorProcess();
};