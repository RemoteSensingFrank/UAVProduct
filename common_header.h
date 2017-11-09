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
//错误代码
typedef long UAVErr;

//标定结构体
typedef struct calib{
    double _flen_x_;
    double _flen_y_;
    double _ppx_;
    double _ppy_;
    double _ccd_size_;
}UAVCalibParams;

//POS数据结构体
typedef struct uavposst{
	double dL;
    double dB;
    double dH;
    double dRoll;
    double dPitch;
    double dHeading;
}UAVPOSSt;

typedef enum coordilisttype{
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



//进度条
typedef int (*UAVProgressFunc)(double dfComplete, const char *pszMessage, void *pProgressArg);

 