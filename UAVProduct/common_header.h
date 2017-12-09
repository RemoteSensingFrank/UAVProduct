#pragma once
//                              @@                                @@                                  
//                             @@@@                             @@@@@                                 
//                          @@@@@@@@@@                         @@@@@@@@@                         
//                   @@@@@@@@@@@@@@@@@@@@@@@@         @@@@@@@@@@@@@@@@@@@@@@@@@                       
//                    @@@@@@@@@@@@@@@@@@@@@@           @@@@@@@@@@@@@@@@@@@@@@@                        
//                             @@@@@                            @@@@@                                 
//                             @@@@@                            @@@@@                                 
//                             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                                 
//                             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                                 
//                             @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@                                  
//                                     @@@@@@@@@@@@@@@@@@@@@@                                         
//                                       @@@@@@@@@@@@@@@@@@                                           
//                                           @@@@@@@@@@                                               
//                                           @@@@@@@@@                                                
//                                             @@@@@@                                                 
//                                    @@@@@@@@@@@@@@@@@@@@@@@                                         
//                                    @@@    @@@@@@@@@@@@@@@@@                                        
//                                    @@@@@@@@@@@@       @@@@@                                        
//                                    @@@@@@@@@@@     @@  @@@@                                        
//                                    @@@@@@@@@@@     @@  @@@@                                        
//                                    @@@@@@@@@@@@       @@@@@                                        
//                                    @@@@@@@@@@@@@@@@@@@@@@@@                                        
//                                     @@@@@@@@@@@@@@@@@@@@@@                                         
//                                                                                                    
//@@      @@     @@@   @@      @@  @@@@@@@   @@@@@@@    @@@@@@@      @@@@@@   @@@@@@@  @@@@@@   @@@@@
//@@      @@    @@@@    @@     @@  @@  @@@   @@  @@@   @@@   @@@    @@@   @@ @@@      @@@  @@  @@   @@
//@@      @@    @@@@@   @@     @@  @@    @@  @@   @@@  @@      @@  @@@       @@@      @@       @@     
//@@      @@    @@ @@   @@    @@   @@    @@  @@   @@  @@@      @@  @@        @@@      @@@      @@@    
//@@      @@   @@  @@    @@   @@   @@   @@@  @@@@@@@  @@       @@  @@        @@@@@@@   @@@@     @@@@  
//@@      @@   @@   @@   @@  @@    @@@@@@@   @@@@@@   @@       @@  @@        @@@@@@@    @@@@     @@@@ 
//@@      @@  @@@@@@@@   @@@ @@    @@@@@     @@  @@@  @@       @@  @@        @@@           @@      @@@
//@@@     @@  @@@@@@@@@   @@ @@    @@        @@   @@  @@@      @@  @@        @@@           @@@      @@
// @@    @@   @@     @@   @@@@     @@        @@   @@   @@     @@@  @@@     @ @@@           @@       @@
// @@@@@@@@  @@      @@    @@@     @@        @@    @@   @@@@@@@@    @@@@@@@@ @@@@@@@  @@@@@@@ @@@@@@@@
// 
//�������
typedef long UAVErr;

//�궨�ṹ��
typedef struct calib{
    double _flen_x_;
    double _flen_y_;
    double _ppx_;
    double _ppy_;
    double _ccd_size_;
}UAVCalibParams;

//Camera Model
//I get this from the openMVG
//but I remove the fisheye and spherical
enum EINTRINSIC
{
  PINHOLE_CAMERA_START = 0,
  PINHOLE_CAMERA = 1,         // No distortion
  PINHOLE_CAMERA_RADIAL1 = 2, // radial distortion K1
  PINHOLE_CAMERA_RADIAL3 = 3, // radial distortion K1,K2,K3
  PINHOLE_CAMERA_BROWN = 4,   // radial distortion K1,K2,K3, tangential distortion T1,T2
  PINHOLE_CAMERA_END = 5
};

typedef struct uavposst{
	double dL;
    double dB;
    double dH;
    double dRoll;
    double dPitch;
    double dHeading;
}UAVPOSSt;

typedef enum coordilisttype
{
	CoordinateUTM,
	CoordinateXYZ,
	CoordinateLocal,
} CoordiListType;

typedef enum EGeometricModel
{
	FUNDAMENTAL_MATRIX = 0,
	ESSENTIAL_MATRIX   = 1,
	HOMOGRAPHY_MATRIX  = 2
}MATRIXMODEL;

typedef enum ePairMode
{
    PAIR_MODE_EXHAUSTIVE = 0,
    PAIR_MODE_CONTIGUOUS = 1,
    PAIR_MODE_NEIGHBORHOOD = 2
}MATCHMODEL;



 