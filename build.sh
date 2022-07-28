#!/bin/bash


mkdir Thirdparty
cd Thirdparty
mkdir eigen pangolin opencv ceres spdlog googletest


# build eigen
cd eigen
git clone https://github.com/libigl/eigen.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../eigen
time make -j$(nproc)
make install


# build pangolin
cd ../../pangolin
git clone -b v0.6 https://github.com/stevenlovegrove/Pangolin.git
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
        -DBUILD_opencv_python3=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TEST=OFF \
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
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../spdlog
time make -j$(nproc)
make install

# build PCL
cd ../../pcl
git clone https://github.com/PointCloudLibrary/pcl.git
mkdir build install && build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../pcl
time make -j$(nproc)
make install

# build googletest
cd ../../googletest
git clone https://github.com/google/googletest.git
mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../googletest
time make -j$(nproc)
make install
