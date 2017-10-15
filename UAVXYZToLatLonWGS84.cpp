//
// Created by wuwei on 17-7-26.
//

#include "UAVXYZToLatLonWGS84.h"

#define WGS84LRadius 6378137
#define WGS84Eccentricity 0.0066943850
#define PI 3.1415926534
Vec3 UAVXYZToLatLonWGS84::BLHToXYZ(double lat, double lon, double alt)
{
    double dN = WGS84LRadius / sqrt(1 - WGS84Eccentricity*sin(lat)*sin(lat));
    double dX = (dN + alt)*cos(lat)*cos(lon);
    double dY = (dN + alt)*cos(lat)*sin(lon);
    double dZ = (dN*(1 - WGS84Eccentricity) + alt)*sin(lat);
    return Vec3(dX,dY,dZ);
}

Vec3 UAVXYZToLatLonWGS84::XYZToBLH(double x, double y, double z) {
    double dB0 = 0;
    double dB1 = 0;
    double dThreshold = 0;
    double dN = 0;

    dB0 = atan(z / sqrt(x*x + y*y));
    do
    {
        dN = WGS84LRadius / sqrt(1 - WGS84Eccentricity*sin(dB0)*sin(dB0));
        dB1 = atan((z + dN*WGS84Eccentricity*sin(dB0)) / sqrt(x*x + y*y));
        dThreshold = fabs(dB0 - dB1);
        dB0 = dB1;
    } while (dThreshold >= 0.00001);
    double dB = dB0;
    double dH = z / sin(dB) - dN*(1 - WGS84Eccentricity);
    double dL = atan(fabs(y / z)) * 180 / PI;
    if (x>0 && y>0)
    {
        ;
    }
    if (x<0 && y>0)
    {
        dL = 180 - dL;
    }
    if (x<0 && y<0)
    {
        dL = 180 + dL;
    }
    if (x>0 && y<0)
    {
        dL = 360 - dL;
    }
    dB = dB * 180 / PI;

    return Vec3(dB,dL,dH);
}

/**
 ** Convert WGS84 lon,lat,alt data to ECEF data (Earth Centered Earth Fixed)
 ** @param lat Latitude in degree
 ** @param lon Longitude in degree
 ** @param alt Altitude relative to the WGS84 ellipsoid
 ** @return ECEF corresponding coordinates
 **/
Vec3 UAVXYZToLatLonWGS84::LatLonToXYZ(double lat, double lon, double alt) {
    const double clat = cos( D2R(lat) );
    const double slat = sin( D2R(lat) );
    const double clon = cos( D2R(lon) );
    const double slon = sin( D2R(lon) );

    const double a2 = Square(WGS84_A);
    const double b2 = Square(WGS84_B);

    const double L = 1.0 / sqrt(a2 * Square(clat) + b2 * Square(slat));
    const double x = (a2 * L + alt) * clat * clon;
    const double y = (a2 * L + alt) * clat * slon;
    const double z = (b2 * L + alt) * slat;

    return Vec3(x, y, z);
}

/**
 ** Convert WGS84 lon,lat,alt data to UTM data (Universal Transverse Mercator)
 ** @param lat Latitude in degree
 ** @param lon Longitude in degree
 ** @param alt Altitude relative to the WGS84 ellipsoid
 ** @return UTM corresponding coordinates
 **/
Vec3 UAVXYZToLatLonWGS84::LatLonToUTM(double lat, double lon, double alt ,double a,double b) {
    a /= 1000; // meters to kilometers
    b /= 1000; // meters to kilometers

    /// CONSTANTS
    static const double N0_n = 0;
    static const double N0_s = 1e4;
    static const double E0 = 5e2;
    static const double k0 = 0.9996;
    const double f = (a - b) / a;

    const double n    = f / (2 - f);
    const double n_2  = n   * n;
    const double n_3  = n_2 * n;
    const double n_4  = n_3 * n;
    const double n_5  = n_4 * n;
    const double n_6  = n_5 * n;
    const double n_7  = n_6 * n;
    const double n_8  = n_7 * n;
    const double n_9  = n_8 * n;
    const double n_10 = n_9 * n;

    const int lon_zone = 1 + floor((lon + 180) / 6);

    const double lon_0 = D2R(3 + 6 * (lon_zone - 1) - 180);

    lat = D2R(lat);
    lon = D2R(lon);

    const double A = a / (1 + n) * (1 + n_2/4 + n_4/64 + n_6/256 + n_8*25.0/16384.0 + n_10*49.0/65536.0);

    const double a1 = (1.0/2.0)*n - (2.0/3.0)*n_2 + (5.0/16.0)*n_3 + (41.0/180.0)*n_4 - (127.0/288.0)*n_5 + (7891.0/37800.0)*n_6 + (72161.0/387072.0)*n_7 - (18975107.0/50803200.0)*n_8 + (60193001.0/290304000.0)*n_9 + (134592031.0/1026432000.0)*n_10;
    const double a2 = (13.0/48.0)*n_2 - (3.0/5.0)*n_3 + (557.0/1440.0)*n_4 + (281.0/630.0)*n_5 - (1983433.0/1935360.0)*n_6 + (13769.0/28800.0)*n_7 + (148003883.0/174182400.0)*n_8 - (705286231.0/465696000.0)*n_9 + (1703267974087.0/3218890752000.0)*n_10;
    const double a3 = (61.0/240.0)*n_3 - (103.0/140.0)*n_4 + (15061.0/26880.0)*n_5 + (167603.0/181440.0)*n_6 - (67102379.0/29030400.0)*n_7 + (79682431.0/79833600.0)*n_8 + (6304945039.0/2128896000.0)*n_9 - (6601904925257.0/1307674368000.0)*n_10;
    const double a4 = (49561.0/161280.0)*n_4 - (179.0/168.0)*n_5 + (6601661.0/7257600.0)*n_6 + (97445.0/49896.0)*n_7 - (40176129013.0/7664025600.0)*n_8 + (138471097.0/66528000.0)*n_9 + (48087451385201.0/5230697472000.0)*n_10;
    const double a5 = (34729.0/80640.0)*n_5 - (3418889.0/1995840.0)*n_6 + (14644087.0/9123840.0)*n_7 + (2605413599.0/622702080.0)*n_8 - (31015475399.0/2583060480.0)*n_9 + (5820486440369.0/1307674368000.0)*n_10;
    const double a6 = (212378941.0/319334400.0)*n_6 - (30705481.0/10378368.0)*n_7 + (175214326799.0/58118860800.0)*n_8 + (870492877.0/96096000.0)*n_9 - (1328004581729009.0/47823519744000.0)*n_10;
    const double a7 = (1522256789.0/1383782400.0)*n_7 - (16759934899.0/3113510400.0)*n_8 + (1315149374443.0/221405184000.0)*n_9 + (71809987837451.0/3629463552000.0)*n_10;
    const double a8 = (1424729850961.0/743921418240.0)*n_8 - (256783708069.0/25204608000.0)*n_9 + (2468749292989891.0/203249958912000.0)*n_10;
    const double a9 = (21091646195357.0/6080126976000.0)*n_9 - (67196182138355857.0/3379030566912000.0)*n_10;
    const double a10 = (77911515623232821.0/12014330904576000.0)*n_10;

    const double t = sinh(atanh(sin(lat)) - 2*sqrt(n)/(1+n) * atanh(2*sqrt(n)/(1+n)*sin(lat)));
    const double xi = atan(t/cos(lon-lon_0));
    const double eta = atanh(sin(lon-lon_0) / sqrt(1+t*t));

    const double N0 = (lat > 0 ? N0_n : N0_s);

    const double E = E0 + k0 * A * (eta + a1*cos(2*1*xi)*sinh(2*1*eta) + a2*cos(2*2*xi)*sinh(2*2*eta) + a3*cos(2*3*xi)*sinh(2*3*eta) + a4*cos(2*4*xi)*sinh(2*4*eta) + a5*cos(2*5*xi)*sinh(2*5*eta) + a6*cos(2*6*xi)*sinh(2*6*eta) + a7*cos(2*7*xi)*sinh(2*7*eta) + a8*cos(2*8*xi)*sinh(2*8*eta) + a9*cos(2*9*xi)*sinh(2*9*eta) + a10*cos(2*10*xi)*sinh(2*10*eta));
    const double N = N0 + k0 * A * (xi + a1*sin(2*1*xi)*cosh(2*1*eta) + a2*sin(2*2*xi)*cosh(2*2*eta) + a3*sin(2*3*xi)*cosh(2*3*eta) + a4*sin(2*4*xi)*cosh(2*4*eta) + a5*sin(2*5*xi)*cosh(2*5*eta) + a6*sin(2*6*xi)*cosh(2*6*eta) + a7*sin(2*7*xi)*cosh(2*7*eta) + a8*sin(2*8*xi)*cosh(2*8*eta) + a9*sin(2*9*xi)*cosh(2*9*eta) + a10*sin(2*10*xi)*cosh(2*10*eta));

    // Scale E,N from kilometers to meters
    return Vec3(E * 1000, N * 1000, alt);
}

/**
 ** Convert ECEF (XYZ) to lon,lat,alt values for the WGS84 ellipsoid
 ** @param x X ECEF coordinate
 ** @param y Y ECEF coordinate
 ** @param z Z ECEF coordinate
 ** @return LLA corresponding coordinates
 **/
// http://fr.mathworks.com/matlabcentral/newsreader/view_thread/142629
Vec3 UAVXYZToLatLonWGS84::XYZToLatLon(double x, double y, double z) {
    const double b = sqrt(WGS84_A*WGS84_A*(1-WGS84_E*WGS84_E));
    const double ep = sqrt((WGS84_A*WGS84_A-b*b)/(b*b));
    const double p = sqrt(x*x+y*y);
    const double th = atan2(WGS84_A*z,b*p);
    const double lon = atan2(y,x);
    const double lat = atan2((z+ep*ep*b* pow(sin(th),3)),(p-WGS84_E*WGS84_E*WGS84_A*pow(cos(th),3)));
    const double N = WGS84_A/sqrt(1-WGS84_E*WGS84_E*sin(lat)*sin(lat));
    const double alt = p/cos(lat)-N;

    return Vec3(R2D(lat), R2D(lon), alt);
}

/**
 ** Convert UTM data to WGS84 lon,lat,alt data(Universal Transverse Mercator)
 **/
Vec3 UAVXYZToLatLonWGS84::UTMToLatLonWMT(double x, double y, double z)
{
    double xt = x/20037508.34*180;
    double yt = y/20037508.34*180;
    yt= 180/M_PI*(2*atan(exp(yt*M_PI/180))-M_PI/2);
    Vec3 latlon;
    latlon(0)=xt;latlon(1)=yt;latlon(2)=z;
    return latlon;
}

Vec3 UAVXYZToLatLonWGS84::LatLonToUTMWMT(double lat,double lon, double alt)
{
    double x = lon *20037508.34/180;
    double y = log(tan((90+lat)*M_PI/360))/(M_PI/180);
    y = y *20037508.34/180;
    Vec3 wmt;
    wmt(0) = x;
    wmt(1) = y;
    wmt(2) = alt;
    return wmt ;
}


// 兲朝火星坐标系偏移公式
// https://on4wp7.codeplex.com/SourceControl/changeset/view/21483#EvilTransform.cs
static double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

static double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
}

bool UAVXYZToLatLonWGS84::OutOfChina(double lat, double lon)
 {
     if (lon < 72.004 || lon > 137.8347)
         return true;
     if (lat < 0.8293 || lat > 55.8271)
         return true;
     return false;
 }

//WGS84 的经纬度转换为GCJ02坐标系下的经纬度
void UAVXYZToLatLonWGS84::WGSLatLonToGCJ(double wgLon, double wgLat, double &mgLon, double &mgLat)
{
    const double a = 6378245.0;
    const double ee = 0.00669342162296594323;

    double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
    double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);

    double radLat = wgLat / 180.0 * M_PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;

    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * M_PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * M_PI);

    mgLat = wgLat + dLat;
    mgLon = wgLon + dLon;
}



long POSProc::POSProc_POSTrans(Vec3 centerllat,Vec3 curllat,Vec3 rot,Vec3 &eo)
{
    //成图坐标系到地心坐标系
    double dL = centerllat(0);
    double dB = centerllat(0);

    Eigen::MatrixXd EMMatrix(3,3);
    EMMatrix(0,0) = -sin(dL);
    EMMatrix(0,1) = cos(dL);
    EMMatrix(0,2) = 0;

    EMMatrix(1,0) = -sin(dB)*cos(dL);
    EMMatrix(1,1) = -sin(dB)*sin(dL);
    EMMatrix(1,2) = cos(dB);

    EMMatrix(2,0) = cos(dB)*cos(dL);
    EMMatrix(2,1) = cos(dB)*sin(dL);
    EMMatrix(2,2) = sin(dB);

    //rotate matrix
    Eigen::MatrixXd EGMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd GIMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd CIMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd ICMatrix=Eigen::MatrixXd::Identity(3,3);	//

    double dH;
    double roll, pitch, yaw;
    dB =curllat(0);
    dL =curllat(1);
    dH =curllat(2);

    roll =rot(0);
    pitch =rot(1);
    yaw =rot(2);

    //WGS84 trans to local coordinate system
    EGMatrix(0,0) = -sin(dB)*cos(dL); EGMatrix(0,1) = -sin(dL); EGMatrix(0,2) = -cos(dB)*cos(dL);
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
    /*CIMatrix[0] = cos(THETA.dY)*cos(THETA.dZ);
    CIMatrix[1] = cos(THETA.dY)*sin(THETA.dZ);
    CIMatrix[2] = -sin(THETA.dY);
    CIMatrix[3] = sin(THETA.dX)*sin(THETA.dY)*cos(THETA.dZ) - cos(THETA.dX)*sin(THETA.dZ);
    CIMatrix[4] = sin(THETA.dX)*sin(THETA.dY)*sin(THETA.dZ) + cos(THETA.dX)*cos(THETA.dZ);
    CIMatrix[5] = sin(THETA.dX)*cos(THETA.dY);
    CIMatrix[6] = cos(THETA.dX)*sin(THETA.dY)*cos(THETA.dZ) + sin(THETA.dX)*sin(THETA.dZ);
    CIMatrix[7] = cos(THETA.dX)*sin(THETA.dY)*sin(THETA.dZ) - sin(THETA.dX)*cos(THETA.dZ);
    CIMatrix[8] = cos(THETA.dX)*cos(THETA.dY);*/

    //seneor to image coordinate system which could be modified according to different seneor
    /*ICMatrix[0] = 0; 	ICMatrix[1] = -1;	ICMatrix[2] = 0;
    ICMatrix[3] = -1;	ICMatrix[4] = 0;	ICMatrix[5] = 0;
    ICMatrix[6] = 0;	ICMatrix[7] = 0;	ICMatrix[8] = -1;*/

    Eigen::MatrixXd IMMatrix(3,3);
    double M1[9], M2[9], M3[9];
    double pVector[] = { 0,0,0 };
    IMMatrix=EMMatrix*EGMatrix*GIMatrix*CIMatrix*ICMatrix;

    //get sensor center pos in the
    UAVXYZToLatLonWGS84 coordi;
    Vec3 cur=coordi.LatLonToXYZ(curllat(0),curllat(1),curllat(2));
    Vec3 cnt=coordi.LatLonToXYZ(centerllat(0),centerllat(1),centerllat(2));

    double dXs = (cur(0) - cnt(0))*EMMatrix(0,0) + (cur(1) - cnt(1))*EMMatrix(0,1) + (cur(2) - cnt(2))*EMMatrix(0,2);
    double dYs = (cur(0) - cnt(0))*EMMatrix(1,0) + (cur(1) - cnt(1))*EMMatrix(1,1) + (cur(2) - cnt(2))*EMMatrix(1,2);
    double dZs = (cur(0) - cnt(0))*EMMatrix(2,0) + (cur(1) - cnt(1))*EMMatrix(2,1) + (cur(2) - cnt(2))*EMMatrix(2,2);

    // calculate the placement vector
    eo(0)=dXs;
    eo(1)=dYs;
    eo(2)=dZs;

    return 0;

}

long POSProc::POSProc_POSTrans(Vec3 centerllat, Vec3 curllat, Vec3 &eo)
{
    //成图坐标系到地心坐标系
    double dL = centerllat(0);
    double dB = centerllat(1);

    Eigen::MatrixXd EMMatrix(3,3);
    EMMatrix(0,0) = -sin(dL);
    EMMatrix(0,1) = cos(dL);
    EMMatrix(0,2) = 0;

    EMMatrix(1,0) = -sin(dB)*cos(dL);
    EMMatrix(1,1) = -sin(dB)*sin(dL);
    EMMatrix(1,2) = cos(dB);

    EMMatrix(2,0) = cos(dB)*cos(dL);
    EMMatrix(2,1) = cos(dB)*sin(dL);
    EMMatrix(2,2) = sin(dB);

    //rotate matrix
    Eigen::MatrixXd EGMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd GIMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd CIMatrix=Eigen::MatrixXd::Identity(3,3);	//
    Eigen::MatrixXd ICMatrix=Eigen::MatrixXd::Identity(3,3);	//

    double dH;
    double roll, pitch, yaw;
    dB =curllat(1);
    dL =curllat(0);
    dH =curllat(2);

    roll =0;
    pitch =0;
    yaw =0;

    //WGS84 trans to local coordinate system
    EGMatrix(0,0) = -sin(dB)*cos(dL); EGMatrix(0,1) = -sin(dL); EGMatrix(0,2) = -cos(dB)*cos(dL);
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
    /*CIMatrix[0] = cos(THETA.dY)*cos(THETA.dZ);
    CIMatrix[1] = cos(THETA.dY)*sin(THETA.dZ);
    CIMatrix[2] = -sin(THETA.dY);
    CIMatrix[3] = sin(THETA.dX)*sin(THETA.dY)*cos(THETA.dZ) - cos(THETA.dX)*sin(THETA.dZ);
    CIMatrix[4] = sin(THETA.dX)*sin(THETA.dY)*sin(THETA.dZ) + cos(THETA.dX)*cos(THETA.dZ);
    CIMatrix[5] = sin(THETA.dX)*cos(THETA.dY);
    CIMatrix[6] = cos(THETA.dX)*sin(THETA.dY)*cos(THETA.dZ) + sin(THETA.dX)*sin(THETA.dZ);
    CIMatrix[7] = cos(THETA.dX)*sin(THETA.dY)*sin(THETA.dZ) - sin(THETA.dX)*cos(THETA.dZ);
    CIMatrix[8] = cos(THETA.dX)*cos(THETA.dY);*/

    //seneor to image coordinate system which could be modified according to different seneor
    ICMatrix(0,0) = 0; 	ICMatrix(0,1) = -1;	ICMatrix(0,2) = 0;
    ICMatrix(1,0) = -1;	ICMatrix(1,1) = 0;	ICMatrix(1,2) = 0;
    ICMatrix(2,0) = 0;	ICMatrix(2,1) = 0;	ICMatrix(2,2) = -1;

    Eigen::MatrixXd IMMatrix(3,3);
    double M1[9], M2[9], M3[9];
    double pVector[] = { 0,0,0 };
    IMMatrix=EMMatrix*EGMatrix*GIMatrix*CIMatrix*ICMatrix;

    //get sensor center pos in the
    UAVXYZToLatLonWGS84 coordi;
    Vec3 cur=coordi.BLHToXYZ(curllat(1),curllat(0),curllat(2));
    Vec3 cnt=coordi.BLHToXYZ(centerllat(1),centerllat(0),centerllat(2));

    double dXs = (cur(0) - cnt(0))*EMMatrix(0,0) + (cur(1) - cnt(1))*EMMatrix(0,1) + (cur(2) - cnt(2))*EMMatrix(0,2);
    double dYs = (cur(0) - cnt(0))*EMMatrix(1,0) + (cur(1) - cnt(1))*EMMatrix(1,1) + (cur(2) - cnt(2))*EMMatrix(1,2);
    double dZs = (cur(0) - cnt(0))*EMMatrix(2,0) + (cur(1) - cnt(1))*EMMatrix(2,1) + (cur(2) - cnt(2))*EMMatrix(2,2);

    // calculate the placement vector
    eo(0)=dXs;
    eo(1)=dYs;
    eo(2)=dZs;
    return 0;
}