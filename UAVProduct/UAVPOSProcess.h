#include "UAVPreProcess.h"

class UAVPOSProcessExtract: public UAVProcessPOSSimple
{
public:
    /*
        Extract POS to matrix P
    */
    long UAVPOSProc_ExtractToP(std::vector<openMVG::Mat34> &vec_P,std::vector<UAVCalibParams> instricParam);

private:
    /**
     * extract rotation matrix
     * @param placementAngle
     * @param placementVec
     * @param P
     * @return
     */
    long UAVPOSProc_EOMatrixTurn(int idx,Vec3f placementAngle,Vec3f placementVec,openMVG::Mat34 &P);

    /*
      get Quadrant according to the direction of the POS
    */
    int UAVPOSProc_EOQuadrant(int idxPOS);
}
