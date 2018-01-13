//
// Created by wuwei on 18-1-13.
//

#include "UAVSeamFinder.h"
#include <types.h>
#include <unistd.h>

#include "gdal_priv.h"
#include "UAVAlgorithm/UAVDistanceTransform.h"

# ifndef uchar
#define uchar unsigned char
#endif

void UAVSeamVoronoiFinder::findInPair(SIZE_IMG size1, SIZE_IMG size2, openMVG::Vec2f lefttop1, openMVG::Vec2f lefttop2,
                                      int widthoverlap, int heightoverlap, std::string strMask1,
                                      std::string stdMask2) {

    //10 pix buffer gap
    const int gap = 10;

    uchar *submask1= nullptr , *submask2= nullptr;
    uchar *mask1 = nullptr , *mask2 = nullptr;
    GDALAllRegister();

    try {
        submask1 = new uchar[(widthoverlap+2*gap)*((heightoverlap+2*gap))];
        submask2 = new uchar[(widthoverlap+2*gap)*((heightoverlap+2*gap))];

        GDALDatasetH datasetMask1,datasetMask2;
        if(access(strMask1,R_OK|W_OK)&&access(strMask2,R_OK|W_OK))
        {
            datasetMask1 = GDALOpen(strMask1,GA_Update);
            datasetMask2 = GDALOpen(strMask2,GA_Update);
            mask1 = new uchar[size1.width*size1.height];
            mask2 = new uchar[size2.width*size2.height];
            GDALRasterIO(GDALGetRasterBand(datasetMask1,1),GF_Read,0,0,size1.width,size1.height,mask1,size1.width,size1.height,GDT_Byte,0,0);
            GDALRasterIO(GDALGetRasterBand(datasetMask1,1),GF_Write,0,0,size2.width,size2.height,mask2,size2.width,size2.height,GDT_Byte,0,0);
        }
        else
        {
            throw -1;
        }

        //initalize
        memset(submask1,0,sizeof(uchar)*(widthoverlap+2*gap)*((heightoverlap+2*gap)));
        memset(submask2,0,sizeof(uchar)*(widthoverlap+2*gap)*((heightoverlap+2*gap)));

        for (int y = -gap; y < heightoverlap+gap; ++y)
        {
            for (int x = -gap; x < widthoverlap; ++x) {
                int y1 = heightoverlap - lefttop1(1)+y;
                int x1 = widthoverlap - lefttop1(0)+x;

                if (y1 >= 0 && x1 >= 0 && y1 < size1.height && x1 < size1.width)
                    submask1[(y+gap)*widthoverlap+x+gap] = mask1[y1*widthoverlap+x1];
                else
                    submask1[(y+gap)*widthoverlap+x+gap] = 0;


                int y2 = heightoverlap - lefttop2(1)+y;
                int x2 = widthoverlap - lefttop2(0)+x;

                if (y2 >= 0 && x2 >= 0 && y2 < size2.height && x2 < size2.width)
                    submask2[(y+gap)*widthoverlap+x+gap] = mask2[y2*widthoverlap+x2];
                else
                    submask2[(y+gap)*widthoverlap+x+gap] = 0;
            }
        }

        uchar *collision = new uchar[(widthoverlap+2*gap)*((heightoverlap+2*gap))];
        memset(collision,0,sizeof(uchar)*(widthoverlap+2*gap)*((heightoverlap+2*gap)));
        float *unique1 = new float[(widthoverlap+2*gap)*((heightoverlap+2*gap))];
        float *unique2 = new float[(widthoverlap+2*gap)*((heightoverlap+2*gap))];

        for(int i=0;i<(widthoverlap+2*gap)*((heightoverlap+2*gap));++i)
        {
            if(submask1[i]!=0&&submask2!=0)
                collision[i] = 1;
        }

        for(int i=0;i<(widthoverlap+2*gap)*((heightoverlap+2*gap));++i)
        {
            if(collision[i]==1)
            {
                unique1[i]=0;
                unique2[i]=0;
            } else{
                unique1[i]=submask1[i];
                unique2[i]=submask2[i];
            }
        }
        DistanceTransform(unique1,widthoverlap,heightoverlap);
        DistanceTransform(unique2,widthoverlap,heightoverlap);

        for (int j = 0; j < widthoverlap; ++j) {
            for (int k = 0; k < heightoverlap; ++k) {
                if(unique1[(k+gap)*widthoverlap+(j+gap)]<unique2[(k+gap)*widthoverlap+(j+gap)])
                {
                    mask2[(heightoverlap-lefttop2(1)+k)*size2.width+widthoverlap-lefttop2(0)+j]=0;
                } else{
                    mask1[(heightoverlap-lefttop1(1)+k)*size1.width+widthoverlap-lefttop1(0)+j]=0;
                }
            }
        }

        //clear memory
        delete []collision;collision= nullptr;
        delete []unique1 ;collision= nullptr;
        delete []unique1 ;collision= nullptr;

    }catch (std::bad_alloc e){

    }catch (int){

    }


}