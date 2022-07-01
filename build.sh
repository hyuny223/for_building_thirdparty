#!/bin/bash


cd Thirdparty && mkdir eigen pangolin opencv ceres


# build eigen
cd eigen
git clone https://github.com/libigl/eigen.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../eigen
time make -j$(nproc)
make install


# build pangolin
cd ../../pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../Pangolin
time make -j$(nproc)
make install


# build opencv
cd ../../opencv
git clone https://github.com/opencv/opencv.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_E=RELEASE \
        -DCMAKE_INSTALL_PREFIX=../install \
        -DWITH_OPENGL=ON \
        -DWITH_GRK=ON \
        -OPENCV_GENERATE_PKGCONFIG ../opencv
time make -j$(nproc)
make install


# build ceres
cd ../../ceres
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../ceres-solver
time make -j$(nproc)
make install
