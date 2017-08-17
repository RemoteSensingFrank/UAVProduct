#UAVProduct  
处理无人机影像的代码，使用了OpenMVG，openMVS以及GDAL库等，实现影像几何校正，影像拼接，点云生成等功能  
##1 处理流程  
* 构建全局影像参数
* 影像列表的构建以及各个文件夹的生成
* 影像特征点的解算
* 影像特征点的匹配
* 影像光束法平差
* 影像拼接得到DOM和点云数据

处理流程图：
![enter image description here](https://lh3.googleusercontent.com/-5X7qRpvaKXw/WYsi216jygI/AAAAAAAACTc/qQSY-BTdO8AwNEAl532NOIlUFHFM7hTkACLcBGAs/s0/%25E6%2597%25A0%25E4%25BA%25BA%25E6%259C%25BA%25E5%25A4%2584%25E7%2590%2586%25E6%25B5%2581%25E7%25A8%258B.png "无人机处理流程.png")  

##2 处理流程详细说明  

###2.1 影像列表的构建  

###2.2 影像特征点的解算  

###2.3 影像特征点匹配  

###2.4 光束法平差的SFM过程  

###2.5 影像拼接  

###2.6 密集点云的生成  

##3 安装与编译  