cmake_minimum_required(VERSION 3.16)
project(module4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE4_SOURCE_FILES
        src/optimization.cpp
        )

find_package(OpenCV 4.4.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/install/lib/cmake/opencv4)
find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/eigen/install/share/eigen3/cmake)
find_package(Ceres REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/ceres/install/lib/cmake/Ceres)
find_package(spdlog 1.10.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/lib/cmake/spdlog)

add_library(${PROJECT_NAME} SHARED
        ${MODULE4_SOURCE_FILES}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${Eigen3_LIBRARIES}
        ${CERES_LIBRARIES}
        spdlog::spdlog
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIR}
        ${spdlog_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/modules/module1/include
        ${CMAKE_SOURCE_DIR}/modules/module2/include
        ${CMAKE_SOURCE_DIR}/modules/module3/include
        )


