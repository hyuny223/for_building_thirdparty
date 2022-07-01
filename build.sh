#!/bin/bash


cd Thirdparty


# build eigen
cd eigen && mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../eigen-3.4.0
time make -j$(nproc)
make install


# build pangolin
cd ../../pangolin && mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../Pangolin
time make -j$(nproc)
make install


# build opencv
cd ../../opencv && mkdir build install && cd build
cmake -DCMAKE_BUILD_TPYE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=../install \
        -DWITH_OPENGL=ON \
        -DWITH_GRK=ON \
        -OPENCV_GENERATE_PKGCONFIG ../opencv-4.4.0
time make -j$(nproc)
make install


# build ceres
cd ../../ceres && mkdir build install && cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../install ../ceres-solver
time make -j$(nproc)
make install
