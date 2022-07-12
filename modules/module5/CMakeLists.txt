cmake_minimum_required(VERSION 3.16)
project(module5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE5_SOURCE_FILES
        src/viewer.cpp
        
        )

find_package(OpenCV 4.4.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/install/lib/cmake/opencv5)

add_library(${PROJECT_NAME} SHARED
        ${MODULE5_SOURCE_FILES}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${Eigen3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIR}

        )


target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/modules/module5/include
        )