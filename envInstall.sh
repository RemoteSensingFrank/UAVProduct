#！/bin/bash
mkdir env && cd env
sudo apt-get install git
git clone –recursive https://github.com/openMVG/openMVG
 
#openMVG
mkdir openMVG_Build && cd openMVG_Build
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/
make &&sudo make install
 
#openMVS
sudo apt-get update -qq && sudo apt-get install -qq
sudo apt-get -y install git mercurial cmake libpng-dev libjpeg-dev libtiff-d    ev libglu1-mesa-dev

cur_path=`pwd`
echo $cur_path
#Eigen (Required)
hg clone https://bitbucket.org/eigen/eigen#3.2
mkdir eigen_build && cd eigen_build
cmake . ../eigen
make && sudo make install
cd ..

#Boost (Required)
sudo apt-get -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

#OpenCV (Required)
sudo apt-get -y install libopencv-dev

#CGAL (Required)
sudo apt-get -y install libcgal-dev libcgal-qt5-dev

#VCGLib (Required)
git clone https://github.com/cdcseacave/VCG.git vcglib

#Ceres (Required)
sudo apt-get -y install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
mkdir ceres_build && cd ceres_build
cmake . ../ceres-solver/ -DMINIGLOG=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j2 && sudo make install
cd ..

#GLFW3 (Optional)
sudo apt-get -y install freeglut3-dev libglew-dev libglfw3-dev

#OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
mkdir openMVS_build && cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_DIR="$main_path/vcglib"

#If you want to use OpenMVS as shared library, add to the CMake command:
-DBUILD_SHARED_LIBS=ON

#Install OpenMVS library (optional):
make -j2 && sudo make install

#GDAL
sudo apt-get install libgdal-dev

#gtest
sudo apt-get install libgtest-dev

#UAVProduct
git clone https://github.com/RemoteSensingFrank/UAVProduct
cd UAVProduct
make -j2
