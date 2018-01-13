//
// Created by wuwei on 18-1-11.
//

#include "UAVDistanceTransform.h"
#include <algorithm>

#ifndef INF
#define INF 1E20
#endif

#ifndef square
#define square(i) i*i
#endif

#ifndef MAX
#define MAX(a,b) a>b?a:b
#endif

/* dt of 1d function using squared distance */

//static float *distancetransform1D(int *f, int n)
//{
//    int *d = new float[n];
//    int *v = new int[n];
//    int *z = new float[n+1];
//    int k = 0;
//    v[0] = 0;
//    z[0] = -INF;
//    z[1] = +INF;
//    for (int q = 1; q <= n-1; q++) {
//        int s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
//        while (s <= z[k]) {
//            k--;
//            s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
//        }
//        k++;
//        v[k] = q;
//        z[k] = s;
//        z[k+1] = +INF;
//    }
//
//    k = 0;
//    for (int q = 0; q <= n-1; q++) {
//        while (z[k+1] < q)
//            k++;
//        d[q] = square(q-v[k]) + f[v[k]];
//    }
//
//    delete [] v;
//    delete [] z;
//    return d;
//}
//static unsigned char *distancetransform1D(unsigned char *f, int n)
//{
//    unsigned char *d = new float[n];
//    int *v = new int[n];
//    int *z = new float[n+1];
//    int k = 0;
//    v[0] = 0;
//    z[0] = -INF;
//    z[1] = +INF;
//    for (int q = 1; q <= n-1; q++) {
//        int s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
//        while (s <= z[k]) {
//            k--;
//            s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
//        }
//        k++;
//        v[k] = q;
//        z[k] = s;
//        z[k+1] = +INF;
//    }
//
//    k = 0;
//    for (int q = 0; q <= n-1; q++) {
//        while (z[k+1] < q)
//            k++;
//        int t=(square(q-v[k]) + f[v[k]]);
//        d[q] = t>255?255:t;
//    }
//
//    delete [] v;
//    delete [] z;
//    return d;
//}
static float *distancetransform1D(float *f, int n)
{
    float *d = new float[n];
    int *v = new int[n];
    float *z = new float[n+1];
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = +INF;
    for (int q = 1; q <= n-1; q++) {
        float s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        while (s <= z[k]) {
            k--;
            s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k+1] = +INF;
    }

    k = 0;
    for (int q = 0; q <= n-1; q++) {
        while (z[k+1] < q)
            k++;
        d[q] = square(q-v[k]) + f[v[k]];
    }

    delete [] v;
    delete [] z;
    return d;
}

void DistanceTransform(unsigned char* imgData,int width,int height)
{
    float *fData = new float[MAX(width,height)];
    float *outData = nullptr;
    try {
        outData = new float[width*height];
        for(int i=0;i<width*height;++j)
            outData[i]=imgData[i];

        //columns
        for (int x = 0; x < width; ++x)
        {
            for (int y = 0; y < height; ++y)
            {
                fData[y] = outData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int y = 0; y < height; ++y)
            {
                outData[y*width+x] = fData[y];
            }
            delete[]t;
        }

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                fData[x] = outData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int x = 0; x < width; ++x)
            {
                outData[y*width+x] = fData[x];
            }
            delete[]t;
        }
        delete []fData;fData = NULL;

        for(int i=0;i<width*height;++j)
            imgData[i]=outData[i]>255?255:outData[i];

        delete[]outData[i];
    }catch (std::bad_alloc e)
    {
        std::cerr<<e.what()<<endl;
    }
}
void DistanceTransform(int* imgData,int width,int height)
{
    float *fData = new float[MAX(width,height)];
    float *outData = nullptr;
    try {
        outData = new float[width*height];
        for(int i=0;i<width*height;++j)
            outData[i]=imgData[i];

        //columns
        for (int x = 0; x < width; ++x)
        {
            for (int y = 0; y < height; ++y)
            {
                fData[y] = outData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int y = 0; y < height; ++y)
            {
                outData[y*width+x] = fData[y];
            }
            delete[]t;
        }

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                fData[x] = outData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int x = 0; x < width; ++x)
            {
                outData[y*width+x] = fData[x];
            }
            delete[]t;
        }
        delete []fData;fData = NULL;

        for(int i=0;i<width*height;++j)
            imgData[i]=outData[i];

        delete[]outData[i];
    }catch (std::bad_alloc e)
    {
        std::cerr<<e.what()<<endl;
    }
}
void DistanceTransform(float* imgData,int width,int height)
{
    float *fData = new float[MAX(width,height)];
    try {
        //columns
        for (int x = 0; x < width; ++x)
        {
            for (int y = 0; y < height; ++y)
            {
                fData[y] = imgData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int y = 0; y < height; ++y)
            {
                outData[y*width+x] = fData[y];
            }
            delete[]t;
        }

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                fData[x] = imgData[y*width+x];
            }
            float *t = distancetransform1D(fData,height);
            for (int x = 0; x < width; ++x)
            {
                imgData[y*width+x] = fData[x];
            }
            delete[]t;
        }
        delete []fData;fData = NULL;

    }catch (std::bad_alloc e)
    {
        std::cerr<<e.what()<<endl;
    }
}
