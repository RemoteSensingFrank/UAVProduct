//
// Created by wuwei on 17-12-6.
//

#include "UAVProcessGCP.h"
openMVG::sfm::Landmarks UAVProcessGCPsGUI::UAVGeoGCPImport(std::string fgcp, COORDITRANSFNC coordiFunc,
                                                           CoordiListType coordiTpIn, CoordiListType coordiTpOut)
{
    //按照GUI输出的格式读取输入的GCP文件
    FILE* fgcps= nullptr;
    fgcps = fopen(fgcp.c_str(),"r+");
    openMVG::sfm::Landmarks gcps;
    char str[128];
    while (fscanf(fgcps, "%s", str) != EOF)
    {
        openMVG::sfm::Landmark gcp;
        int idLandMark,numObs;
        double Xs,Ys,Zs;
        sscanf(str,"%d %lf %lf %lf %d",&idLandMark,&Xs,&Ys,&Zs,&numObs);

        if(numObs<2)
           break;
        else
        {
            gcp.X=openMVG::Vec3(Xs,Ys,Zs);
            for(int i=0;i<numObs;++i)
            {
                fscanf(fgcps, "%s", str);
                int obsId;
                double x,y;
                sscanf(str,"%d %lf %lf",&obsId,&x,&y);
                gcp.obs[obsId].x=openMVG::Vec2(x,y);
            }
            gcps[idLandMark]=gcp;
        }
    }
    return gcps;
}