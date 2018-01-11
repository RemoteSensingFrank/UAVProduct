#ifndef _UAV_SEAM_FINDER_H_
#define _UAV_SEAM_FINDER_H_

#include"UAVProcessGeometry.h"
class UAVSeamFinder : public UAVProcessGeometry
{
public:
    /**
     * construct seam find data
     * vecImgs：image path
     * vecMasks:mask data
     * corner：top left point in  mosaic image
    * */
    void UAVSeamConstruct(std::vector<std::string> vecImgs,std::vector<std::string> vecMasks,std::vector<openMVG::Vec2f> corner);

    /**
     * get overlap of two images
     * img1:image 1 path
     * img2:image 2 path
     * lefttop1:lefttoppoint*/
    void UAVOverlapRoi(std::string img1,std::string img2, openMVG::Vec2f &lefttop1,openMVG::Vec2f &lefttop2,int &width,int &height);
}








#endif