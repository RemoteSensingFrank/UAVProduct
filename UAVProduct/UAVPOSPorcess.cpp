#include "UAVPOSProcess.h"

long UAVPOSProcessExtract::UAVPOSProc_ExtractToP(std::vector<openMVG::Mat34> &vec_P,std::vector<UAVCalibParams> instricParam)
{
    
}



int GeoPOSProcess::GeoPOSProc_EOQuadrant(int idxPOS)
{
    double yaw = this->posList[idxPOS].dHeading;
    long nFlag = 0;
    double temps = yaw;
    if (abs(int(temps)) % 360<10)					//North-East--1 quandrant
      nFlag = 1;
    else if (abs(int(temps - 90)) % 360<10)			//North_West--2 quandrant
      nFlag = 2;
    else if (abs(int(temps - 180)) % 360<10)		//South-West--3 quandrant
      nFlag = 3;
    else if (abs(int(temps - 270)) % 360<10)		//South-East--4 quandrant
      nFlag = 4;

    return nFlag;
}
