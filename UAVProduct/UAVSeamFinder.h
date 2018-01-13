#ifndef _UAV_SEAM_FINDER_H_
#define _UAV_SEAM_FINDER_H_

#include"UAVProcessGeometry.h"

#ifndef SIZE_IMG
typedef struct ImgSize{
    int width;
    int height;
}SIZE_IMG;
#endif

//interface of class seam finder
class UAVSeamFinder : public UAVProcessGeometry
{
public:
    /**
     * construct seam find data
     * vecImgs：image path
     * vecMasks:mask data
     * corner：top left point in  mosaic image
    * */
    virtual void UAVSeamConstruct(std::vector<std::string> vecImgs,std::vector<std::string> vecMasks,std::vector<openMVG::Vec2f> corner) = 0;


    /**
     * Trans Mask File to shpfile to get seam
     * @param imgMasks mask images
     * @param shpFile  output shpfile
     */
    void UAVSeamMaskToShp(std::vector<std::string> imgMasks,std::string shpFile);

protected:
    /**
      * get overlap of two images
      * img1:image 1 path
      * img2:image 2 path
      * lefttop1: lefttop point
      * lefttop2: lefttop point
      * int width: overlap width
      * int height:overlap height
      * */
    void UAVOverlapRoi(std::string img1,std::string img2, openMVG::Vec2f &lefttop1,openMVG::Vec2f &lefttop2,int &width,int &height);

};

/**
 * Voronoi diagram-based seam estimator.
 * the algorithm ref source of openCV
 */
class UAVSeamVoronoiFinder : public UAVSeamFinder
{
public :
    /**
     * construct seam find data
     * vecImgs：image path
     * vecMasks:mask data
     * corner：top left point in  mosaic image
    * */
    virtual void UAVSeamConstruct(std::vector<std::string> vecImgs,std::vector<std::string> vecMasks,std::vector<openMVG::Vec2f> corner);

private:
    /**
     * find seam in image pairs
     * @param size1 :sizeof image1
     * @param size2 :sizeof image2
     * @param lefttop1 :lefttop point of overlap in image1
     * @param lefttop2 :lefttop point of overlap in image2
     * @param widthoverlap :overlap width
     * @param heightoverlap:overlap height
     * @param strMask1 :mask1 output path
     * @param stdMask2 :mask2 output path
     */
    void findInPair(SIZE_IMG size1,SIZE_IMG size2,openMVG::Vec2f lefttop1,openMVG::Vec2f lefttop2,
                    int widthoverlap,int heightoverlap,std::string strMask1,std::string stdMask2);
};







#endif