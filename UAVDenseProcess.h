//
// Created by wuwei on 17-8-7.
//

#ifndef UAVPRODUCT_UAVDENSEPROCESS_H
#define UAVPRODUCT_UAVDENSEPROCESS_H
#include <string>
using namespace std;


class UAVDenseProcess {
public:
    /**
     * 将光束法平差得到的点云输出为MVS的格式方便调用openMVS构建点云
     */
    void UAVDP_ExportMVS();

    /**
     * 调用openMVS库得到密集匹配的点云数据
     */
    void UAVDP_MVSProc();


    void UAVDPCloud_ToDSM(string pathPly,string pathDsm,double dL,double dB,double dGround);

private:
    /*
     * 初始化
     * */
    void UAVDP_MVSProcInitialize();
};



#endif //UAVPRODUCT_UAVDENSEPROCESS_H
