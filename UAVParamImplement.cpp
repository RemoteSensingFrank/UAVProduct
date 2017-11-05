#include "UAVParamImplement.h"
UAVErr UAVParamPos::UAVProcParamImport(const char *pathParam) {
    return 0;
}
UAVErr UAVParamPos::UAVProcParamExport(const char *pathParam) {
    return 0;
}
bool UAVParamPos::UAVProcParamCheck() {
    if(dL*PI/180>PI||dL*PI/180<-PI||dB*PI/180>PI_D2||dB*PI/180<-0)
        return false;
    else
        return true;
}

UAVErr UAVParamPos1::UAVProcParamPos_Get(FILE *file) {
    char pLine[2048];
    fgets(pLine,2048,file);
    sscanf(pLine,"%lf%lf%lf%lf%lf%lf",&dL,&dB,&dH,&dRoll,&dPitch,&dHeading);
}

UAVErr UAVParamList::UAVProcParamImport(const char *pathParam) {
    return 0;
}
UAVErr UAVParamList::UAVProcParamExport(const char *pathParam) {
    return 0;
}
bool UAVParamList::UAVProcParamCheck() {
    return 0;
}

UAVErr UAVParamFeature::UAVProcParamImport(const char *pathParam) {
    return 0;
}
UAVErr UAVParamFeature::UAVProcParamExport(const char *pathParam) {
    return 0;
}
bool UAVParamFeature::UAVProcParamCheck() {
    return 0;
}

UAVErr UAVParamFeature::UAVParamFeatureInsert(int viewIdx, std::string image, std::string feature) {
    FeatureParam featParam;
    featParam._image_in_=image;
    featParam._feature_out_=feature;
    _param_features_.insert(std::make_pair(viewIdx,featParam));
}