//
// Created by wuwei on 18-1-11.
//

#ifndef UAVPRODUCT_UAVDISTANCETRANSFORM_H
#define UAVPRODUCT_UAVDISTANCETRANSFORM_H

/**
 * distance transform
 * @param imgData:影像数据
 * @param width :数据宽度
 * @param height:数据高度
 */
void DistanceTransform(unsigned char* imgData,int width,int height);
void DistanceTransform(int* imgData,int width,int height);
void DistanceTransform(float* imgData,int width,int height);

/**
 * distance transform using GPU with openGL
 * @param imgData
 * @param width
 * @param height
 */
void DistanceTransformGL(float* imgData,int width,int height);
#endif //UAVPRODUCT_UAVDISTANCETRANSFORM_H
