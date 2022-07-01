#!/bin/bash


cd Thirdparty


# build eigen
cd eigen
git clone https://gitlab.com/libeigen/eigen.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../eigen
time make -j$(nproc)
make install


# build pangolin
cd ../../pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../Pangolin
time make -j$(nproc)
make install


# build opencv
cd ../../opencv
git clone https://github.com/opencv/opencv.git -b 4.4.0
mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=../install \
        -DWITH_OPENGL=ON \
        -DWITH_GRK=ON \
        -OPENCV_GENERATE_PKGCONFIG ../opencv-4.4.0
time make -j$(nproc)
make install


# build ceres
cd ../../ceres
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../ceres-solver
time make -j$(nproc)
make install
