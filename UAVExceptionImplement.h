#pragma once

#include"UAVInterface.h"

class UAVExceptionInput:public UAVException
{    
public:    
	virtual const char* what() const throw() = 0;   
}; 

class UAVExceptionPos:public UAVException{
public:
	virtual const char* what() const throw()
	{
		return "error POS data\n";
	}
};

class UAVExceptionImage:public UAVExceptionInput
{
public:    
	virtual const char* what() const throw()
	{
		return "error input image list\n";
	}
};

class UAVExceptionCalibrate:public UAVExceptionInput
{
public:    
	virtual const char* what() const throw()
	{
		return "error input calibrate parameters\n";
	}
};

class UAVExceptionCheck:public UAVException
{
public:
    static void UAVException_Check(UAVErr err)
    {
        switch(err)
        {
            case 0x00000000:
                return ;
            case 0x00000001:
                throw UAVExceptionImage();
                break;
            case 0x00000010:
                throw UAVExceptionPos();
                break;

        }
    }
};
