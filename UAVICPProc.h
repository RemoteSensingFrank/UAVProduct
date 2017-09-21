//
// Created by wuwei on 17-8-30.
//

#ifndef __UAVICPProc_H
#define __UAVICPProc_H

#include <string>
using namespace std;

//解算的类型
enum EXTRACT_TYPE {
  EXTRACT_CPU,    //CPU计算
  EXTRACT_GPU     //GPU计算
};
//根据Google瓦片数据解算相控点
class UAVICPExtract{
public:
    /*
      解算匹配并以ENVI的格式输出，以ENVI的格式输出主要是方便查看是否正确
      string img1：影像1的路径
      string img2：影像2的路径
      string match：匹配点输出路径
      EXTRACT_TYPE type=EXTRACT_CPU 解算方法，默认采用CPU进行解算
    */
    bool UAVICPExtractMatchesEnvi(string img1,string img2,string  match,EXTRACT_TYPE type=EXTRACT_CPU);

    /*
      解算得到相控点并以二进制格式输出
      string img1：影像1的路径
      string img2：影像2的路径
      string icps：相控点输出路径
      EXTRACT_TYPE type=EXTRACT_CPU 解算方法，默认采用CPU进行解算
    */
    bool UAVICPExtractICPs(string img1,string img2,string  icps,EXTRACT_TYPE type=EXTRACT_CPU);

    //导入相控点
    bool UAVICPImportICPs();

private:
    /**
     * 从文件中导入像控点
     * @param pathICPs:像控点文件
     * @param pathSFM :SFM文件
     * @return
     */
    bool UAVICPImportICPs(string pathICPs,string pathSFM);
};


#endif
