#!/bin/bash


cd Thirdparty && mkdir eigen pangolin opencv ceres spdlog


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
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip
unzip opencv.zip && rm opencv.zip
mkdir build install && cd build
cmake -DCMAKE_BUILD_E=RELEASE \
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

# build spdlog
cd ../../spdlog
git clone https://github.com/gabime/spdlog.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../spdlog
time make -j$(nproc)
make install
