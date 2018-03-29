#include "UAVPOSProcess.h"
using namespace Eigen;

#ifndef PI
#define PI 3.1415926534
#endif

long UAVPOSProcessExtract::UAVPOSProc_ExtractToP(std::vector<openMVG::Mat34> &vec_P,std::vector<UAVCalibParams> instricParam)
{
    
}

long UAVPOSProcessExtract::UAVPOSProc_EOMatrixTurn(int idx,Vec3f placementAngle,Vec3f placementVec,UAVCalibParams instric,openMVG::Mat34 &P)
{
    long lError = 0;
    double dB, dL, dH;

    //get BLH value
    dB = this->posList[idx].dB; dL = this->posList[idx].dL; dH = this->posList[idx].dH;
    MatrixXd EMMatrix(3,3);
    EMMatrix(0,0) = -sin(dL);
    EMMatrix(0,1) = cos(dL);
    EMMatrix(0,2) = 0;

    EMMatrix(1,0) = -sin(dB)*cos(dL);
    EMMatrix(1,1) = -sin(dB)*sin(dL);
    EMMatrix(1,2) = cos(dB);

    EMMatrix(2,0) = cos(dB)*cos(dL);
    EMMatrix(2,1) = cos(dB)*sin(dL);
    EMMatrix(2,2) = sin(dB);

    //coordinate trans from BLH to XYZ
    Vec3 XYZPnt=UAVProcessGeometry::UAVProcessGeoBLHToXYZ(dB,dL,0);
    int nQuandNum = UAVPOSProc_EOQuadrant(idx);

    //rotate matrix
    MatrixXd EGMatrix(3,3);	//
    MatrixXd GIMatrix(3,3);	//
    MatrixXd CIMatrix(3,3);	//
    MatrixXd ICMatrix(3,3);	//

    double roll, pitch, yaw;

    roll    = this->posList[idx].dRoll;
    pitch   = this->posList[idx].dPitch;
    yaw     = this->posList[idx].dHeading;

    //WGS84 trans to local coordinate system
    EGMatrix(0,0) = -sin(dB)*cos(dL); EGMatrix(0,1) = -sin(dL); EGMatrix(0.2) = -cos(dB)*cos(dL);
    EGMatrix(1,0) = -sin(dB)*sin(dL); EGMatrix(1,1) = cos(dL); EGMatrix(1,2) = -cos(dB)*sin(dL);
    EGMatrix(2,0) = cos(dB);		 EGMatrix(2,1) = 0; ; EGMatrix(2,2) = -sin(dB);

    //Local coordinate system to IMU coordinate system
    GIMatrix(0,0) = cos(pitch)*cos(yaw);
    GIMatrix(0,1) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    GIMatrix(0,2) = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    GIMatrix(1,0) = cos(pitch)*sin(yaw);
    GIMatrix(1,1) = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
    GIMatrix(1,2) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    GIMatrix(2,0) = -sin(pitch);
    GIMatrix(2,1) = sin(roll)*cos(pitch);
    GIMatrix(2,2) = cos(roll)*cos(pitch);

    //IMU to sensor coordinate system trans
    CIMatrix(0,0) = cos(placementAngle(1))*cos(placementAngle(2));
    CIMatrix(0,1) = cos(placementAngle(1))*sin(placementAngle(2));
    CIMatrix(0,2) = -sin(placementAngle(1));
    CIMatrix(1,0) = sin(placementAngle(0))*sin(placementAngle(1))*cos(placementAngle(2)) - cos(placementAngle(0))*sin(placementAngle(2));
    CIMatrix(1,1) = sin(placementAngle(0))*sin(placementAngle(1))*sin(placementAngle(2)) + cos(placementAngle(0))*cos(placementAngle(2));
    CIMatrix(1,2) = sin(placementAngle(0))*cos(placementAngle(1));
    CIMatrix(2,0) = cos(placementAngle(0))*sin(placementAngle(1))*cos(placementAngle(2)) + sin(placementAngle(0))*sin(placementAngle(2));
    CIMatrix(2,1) = cos(placementAngle(0))*sin(placementAngle(1))*sin(placementAngle(2)) - sin(placementAngle(0))*cos(placementAngle(2));
    CIMatrix(2,2) = cos(placementAngle(0))*cos(placementAngle(1));

    //seneor to image coordinate system which could be modified according to different seneor
    ICMatrix(0,0) = 0; 	ICMatrix(0,1) = -1;	ICMatrix(0,2) = 0;
    ICMatrix(1,0) = -1;	ICMatrix(1,1) = 0;	ICMatrix(1,2) = 0;
    ICMatrix(2,0) = 0;	ICMatrix(2,1) = 0;	ICMatrix(2,2) = -1;

//    double IMMatrix[9];
//    double M1[9], M2[9], M3[9];
//    double pVector[] = { 0,0,0 };

    MatrixXd IMMatrix(3,3);
    IMMatrix = EMMatrix*EGMatrix*GIMatrix*CIMatrix*ICMatrix;
//    MatrixMuti(EMMatrix, 3, 3, 3, EGMatrix, M1);
//    MatrixMuti(M1, 3, 3, 3, GIMatrix, M2);
//    MatrixMuti(M2, 3, 3, 3, CIMatrix, M3);
//    MatrixMuti(M3, 3, 3, 3, ICMatrix, IMMatrix);

    double dPhi, dOmega, dKappa;

    dPhi = asin(-IMMatrix(0,2));
    dOmega = atan(-IMMatrix(1,2) / IMMatrix(2,2));
    if (nQuandNum == 1 || nQuandNum == 2)
    {
        dKappa = abs(atan(-IMMatrix(0,1) / IMMatrix(0,0)));
    }
    else
    {
        dKappa = abs(atan(-IMMatrix(0,1) / IMMatrix(0,0))) - PI / 2;
    }

    //get sensor center pos in the
    Vec3 curPoint;
    curPoint = UAVProcessGeometry::UAVProcessGeoBLHToXYZ(dB,dL,dH);
    double dXs = (curPoint(0) - XYZPnt(0))*EMMatrix(0,0) + (curPoint(1) - XYZPnt(1))*EMMatrix(0,1) + (curPoint(2) - XYZPnt(2))*EMMatrix(0,2);
    double dYs = (curPoint(0) - XYZPnt(0))*EMMatrix(1,0) + (curPoint(1) - XYZPnt(1))*EMMatrix(1,1) + (curPoint(2) - XYZPnt(2))*EMMatrix(1,2);
    double dZs = (curPoint(0) - XYZPnt(0))*EMMatrix(2,0) + (curPoint(1) - XYZPnt(1))*EMMatrix(2,1) + (curPoint(2) - XYZPnt(2))*EMMatrix(2,2);

    // calculate the placement vector
    MatrixXd transMatrix(3,3);
    transMatrix = EGMatrix*GIMatrix;
    double dXl = transMatrix(0,0) * placementVec(0) + transMatrix(0,1) * placementVec(1) + transMatrix(0,2) * placementVec(2);
    double dYl = transMatrix(1,0) * placementVec(0) + transMatrix(1,1) * placementVec(1) + transMatrix(1,2) * placementVec(2);
    double dZl = transMatrix(2,0) * placementVec(0) + transMatrix(2,1) * placementVec(1) + transMatrix(2,2) * placementVec(2);

    //rotation matrix
    MatrixXd rotationMatrix(3,3);
    Eigen::AngleAxisd romega(dOmega, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rpitch(dPhi, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rkappa(dKappa, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = romega * rpitch * rkappa;

    rotationMatrix = q.matrix();
    MatrixXd instricMatrix=Eigen::Matrix3d::Identity();


    return 0;
//    pdfEO.m_dX = dXs + dXl;
//    pdfEO.m_dY = dYs + dYl;
//    pdfEO.m_dZ = dZs + dZl;
//    pdfEO.m_phia = dPhi;
//    pdfEO.m_omega = dOmega;
//    pdfEO.m_kappa = dKappa;

    //caculate matrix P
}


int UAVPOSProcessExtract::UAVPOSProc_EOQuadrant(int idxPOS)
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
