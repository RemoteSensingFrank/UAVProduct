//
// Created by wuwei on 17-7-31.
//

#ifndef UAVPRODUCT_UAVBUNDLE_H
#define UAVPRODUCT_UAVBUNDLE_H

#include <string>
#include <cstdlib>



using namespace std;

class UAVBundle {
public:
    /**
     * 得到全局的旋转和平移矩阵
     */
    bool UAVBundleGlobal();
    bool UAVBundleGlobalGpu();  //对采用GPU解算得到的特征点和特征描述采用全局算法进行光束法解算

    /**
     * 采用序列算法进行求解
     */
    bool UAVBundleSequence();
    bool UAVBundleSequenceGpu();
};


#endif //UAVPRODUCT_UAVBUNDLE_H
