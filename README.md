# UAVProduct处理说明
处理无人机影像的代码，使用了OpenMVG，openMVS以及GDAL库等，实现影像几何校正，影像拼接，点云生成等功能

[![Build Status](https://travis-ci.org/RemoteSensingFrank/UAVProduct.svg?branch=master)](https://travis-ci.org/RemoteSensingFrank/UAVProduct)
[![Waffle.io](http://progressed.io/bar/80?title=done )](https://travis-ci.org/RemoteSensingFrank/UAVProduct)
## 1 处理流程  
* 构建全局影像参数
* 影像列表的构建以及各个文件夹的生成
* 影像特征点的解算
* 影像特征点的匹配
* 影像光束法平差
* 影像拼接得到DOM和点云数据

处理流程图：
![无人机影像处理流程](https://lh3.googleusercontent.com/-5X7qRpvaKXw/WYsi216jygI/AAAAAAAACTc/qQSY-BTdO8AwNEAl532NOIlUFHFM7hTkACLcBGAs/s0/%25E6%2597%25A0%25E4%25BA%25BA%25E6%259C%25BA%25E5%25A4%2584%25E7%2590%2586%25E6%25B5%2581%25E7%25A8%258B.png "无人机处理流程.png")  

## 2处理流程详细说明  

### 2.1 影像列表的构建  
这一步主要是进行一些估计和一些预处理的操作，实际上包括构建影像列表，解析POS数据得到处理影像大小得到影像的内参数据等，
在这一步中要获取影像的基本信息，并构建列表方便解算，另外在存在POS的情况下可以通过焦距和航高以及POS数据估算影像实际
的地面范围，然后将地面范围以KML数据的形式输出方便查看，同时估算拼接后影像数据大小

### 2.2 影像特征点的解算  
影像特征点的解算主要是通过SIFT特征进行解算，SIFT特征具有旋转不变，尺度不变光照不变等特点，由于其具有以上特征使得SIFT算子在特征点提取方面存在巨大的优势，SIFT特征点提取流程为：
* 1.尺度空间的建立
* 2.检测DoG空间极值点
* 3.空间极值点的精化处理
* 4.特征点方向计算
* 5.特征点方向特征描述子生成

下面详细介绍一下各个处理流程，首先是尺度空间的建立过程，对于输入影像建立影像金字塔，这个过程分为两个部分第一部分为影像尺度空间的降采样，每次对影像进行/2的降采样，得到的影像长和宽均为原影像的一般，第二部分为同一尺度空间的高斯模糊，通过不同尺度的高斯低通滤波算子对影像进行滤波，得到不同程度模糊的影像；第二个步骤为DoG空间极值点的计算，DoG空间极值即为高斯差分的空间极值，比较本空间和相邻两个尺度空间的DoG极值，取最大值，实际上DoG空间极值就是像素灰度变化最大的值，对影像来说角点，边缘点都可能具有较大的DoG值，实际上我们最需要的是角点信息，DoG局部曲率非常不对称的点，因此需要对步骤2检测到的空间极值点进行精化处理，精化处理的方法为：通过拟和三维二次函数以精确确定关键点的位置和尺度（达到亚像素精度），同时去除低对比度的关键点和不稳定的边缘响应点(因为DoG算子会产生较强的边缘响应)，以增强匹配稳定性、提高抗噪声能力。具体介绍参看：
进行精化后就可以得到影像特征点，在此情况下需要确定特征点的主方向，特征点的主方向通过梯度的模值和角度来确定主方向，获取主方向之后选取8*8窗口大小的窗口分成4*4的大小，则每一个窗口内有2*2个像素，计算像素的梯度则每一个2*2像素有八个方向的梯度，最后得到128维的特征描述子，最后对特征描述子进行归一化即可得到特征描述：
[SIFT参考](http://blog.csdn.net/abcjennifer/article/details/7639681/)

### 2.3 影像特征点匹配  
实际上通过SIFT进行特征点匹配的过程中只要找到了特征点，则特征匹配都不是什么太大的问题，但是有一点需要注意，一般来说我们得到了一个128维的特征，在此情况下通过特征匹配得到最佳匹配可以认为是匹配点，匹配的过程可以通过KNN算法进行加速实现，但是这样可能出现一个问题，就是重复匹配问题，即多个点都匹配到同一个点带来匹配误差，在此情况下需要进行修正，纠正的方法很多，通过仿射变换矩阵，单应矩阵，基础矩阵等都可以进行修正，openMVG提供了多种修正方式可以选取，在代码中默认使用单应矩阵进行修正，修正后能够使匹配效果得到吉他的提高．

### 2.4 光束法平差的SFM过程  
关于这个恐怕是整个过程中最重要的部分了，说它最重要不一定是说有多难，而是说明很多搞摄影测量的人把摄影测量和CV结合起来的时候有这个光束法的概念，但是并不是真的懂了光束法平差的整个过程，我觉得对于一个专业人员来说只了解概念用用软件是没有什么用的，所以这一块需要好好的研究，肤浅的了解了一点然后胡吹这样总是不对的。  
好了吐槽完毕之后我们讲干货：光束法平差的过程：  
实际上在光束法平差之前我们需要对所有影像进行相对定向，在相对定向的基础上才能够进行光束法平差，也就是我们摄影测量中所说的空三加密的过程，而通过CV进行相对定向主要有两种方法，第一种为全局法第二种为序列方法，[全局法的介绍](http://wuweiblog.com/2017/07/17/global-rotation-average/)，在以上链接中有全局法较为详细的介绍，在这里就不详细说明了，另外就是序列法，这个方法对于用摄影测量的术语应该叫做[连续法相对定向](http://www.docin.com/p-766375704.html)，但是在CV中有更加严密的求解方法，并不通过泰勒展开进行迭代，而是在已知内参的情况下通过矩阵分解对本质矩阵进行分解得到R和T，通过图像一张张的添加得到相对变换的R和T然后通过最小生成树选择参考影像得到全局变换的R和T[本质矩阵的矩阵分解参考](http://www.360doc.com/content/14/0205/15/10724725_349965748.shtml)。下面分别介绍两种方法的光束法平差，实际上对于全局求解方法，直接得到全局数据后根据全局的R和T可以通过前方交会求解相对坐标系下的三维坐标，每一个三维坐标和其对应的二维影像坐标以及其旋转和平移矩阵可以列出共线条件方程，在此情况下可以列出光束法平差的条件方程，实际上就是一个求解一个巨大的稀疏矩阵的过程，求解的工具主要包括[SBA](http://users.ics.forth.gr/~lourakis/sba/)和[Ceres Solcver](http://www.ceres-solver.org/)等。而连续法的求解方法相当于一个逐步添加的过程，求出两个影像相对关系后计算三维点坐标，进行光束法平差，然后添加一幅影像继续以上过程，直到所有影像都添加完毕，当然过程中还涉及到一些细节在这里并不进行详细说明了。

### 2.5 影像拼接  

### 2.6 密集点云的生成  

## 3 源码使用
这一章理论上是不用介绍的，因为代码量比较少大家看看就明白了，但是鉴于注释不太清楚而且测试用例不全，所以还是介绍一下，首先需要初始化参数，全局变量中提供的所有哦参数都需要进行初始化，初始化之后调用UAVList_CreateSFMList函数创建需要进行处理的影像集，得到一些参数，如果想要粗略的显示影像位置可以调用UAVList_CreateImageRange函数进行显示，结果会以KML文件的形式给出在 Google Earth上导入可以直接看，次此功能必须在具有POS数据的情况才能够使用，在缺少POS数据的情况下无法使用，得到影像list之后调用特征点提取和解算函数进行特征点的提取和解算，在这里特征点提取和解算可以选取CPU计算也可以选用GPU进行计算，如果存在像控点则可以在这一步添加像控点，最后对结果进行光束法平差．
## 4 安装与编译  
[编译参考](http://wuweiblog.com/2017/08/24/%E7%A8%8B%E5%BA%8F%E7%8E%AF%E5%A2%83%E9%85%8D%E7%BD%AE/)

# How to Compile the UAV Product

------
Show How to compile the UAV Product step by step under Ubuntu16.04:
 1. openMVG
 2. openMVS
 3. DevIl
 4. gTest
 5. other third party library
 6. UAV Product


1.openMVG
---

compile the openMVG and install

    git clone https://github.com/openMVG/openMVG.git
    git submodule update -i
    cd ./src
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install
2.openMVS
---
be careful the verison of the eigen used by openMVS must be 3.2;
and make sure you build the release version

    git clone https://github.com/cdcseacave/openMVS.git
    mkdir buildOpenMVS
    cd buildOpenMVS
    cmake ..
    make -j4
    sudo make install

3.DevIL
---

    git clone https://github.com/DentonW/DevIL.git
    mkdir build
    cmake ..
    make 
    sudo make install

4.gTest
---
    sudo apt-get install libgtest-dev
    cd /usr/src/gtest
    sudo cmake .
    sudo make
    sudo cp libgtest*.a /usr/local/lib
    
5.other third party library
---
    cgal:`sudo apt-get install libcgal-qt5-dev`
    opencv:`sudo apt-get install libopencv-dev`
    gdal:`sudo apt-get install libgdal-dev`
    boost:`sudo apt-get install libboost-all-dev`
    libkml:`sudo apt-get install libkml-dev`
    glew
6.UAV
---
    set openMVG_DIR
    set openMVS_DIR
    set SIFT_LIBRARY
    set EXIF_LIBRARY


    

 

