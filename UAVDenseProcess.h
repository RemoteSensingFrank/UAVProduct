//
// Created by wuwei on 17-8-7.
//

#ifndef UAVPRODUCT_UAVDENSEPROCESS_H
#define UAVPRODUCT_UAVDENSEPROCESS_H

#include "UAVInterface.h"
#include "common_header.h"
#include <string>


class UAVDenseProcess : public UAV
{
public:
    /**
     * 调用openMVS库得到密集匹配的点云数据
     */
    UAVErr UAVDP_MVSProc(std::string mvs,std::string dense);


    //void UAVDPCloud_ToDSM(string pathPly,string pathDsm,double dL,double dB,double dGround);

private:
    /*
     * 初始化
     * */
    void UAVDP_MVSProcInitialize();
};



#endif //UAVPRODUCT_UAVDENSEPROCESS_H
