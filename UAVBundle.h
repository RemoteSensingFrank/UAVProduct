//
// Created by wuwei on 17-7-31.
//

#ifndef UAVPRODUCT_UAVBUNDLE_H
#define UAVPRODUCT_UAVBUNDLE_H

#include <string>
#include "common.h"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;

//TODO:全局的处理
//TODO:序列影像的处理方法
class UAVBundle {
    /**
     * 得到任意两个view之间的相对变换关系并将其输出到文件中
     * @param 输出文件,自定义文件格式并写工具进行处理
     */
    void UAVBundleTwoViewExtract(string pathRotMat);


};


#endif //UAVPRODUCT_UAVBUNDLE_H
