//
// Created by wuwei on 17-12-6.
//

#ifndef UAVPRODUCT_UAVPROCESSGCP_H
#define UAVPRODUCT_UAVPROCESSGCP_H

#include "UAVProcessGeometry.h"

class UAVProcessGCPsGUI:public UAVProcessGCPs{
public:
    virtual openMVG::sfm::Landmarks UAVGeoGCPImport(std::string fgcp,CoordiListType coordiTpIn,CoordiListType coordiTpOut);
};

#endif //UAVPRODUCT_UAVPROCESSGCP_H
