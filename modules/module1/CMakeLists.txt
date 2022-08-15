cmake_minimum_required(VERSION 3.16)
project(module1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE1_SOURCE_FILES
        src/frame.cpp
        src/framepoint.cpp
        src/keyframe.cpp
        src/localmap.cpp
        src/globalmap.cpp
        src/keyframeVec.cpp
        src/loadImage.cpp
        )

find_package(OpenCV 4.4.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/install/lib/cmake/opencv4)
find_package(spdlog 1.10.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/lib/cmake/spdlog)


add_library(${PROJECT_NAME} SHARED
        ${MODULE1_SOURCE_FILES}
        )


target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${spdlog_INCLUDE_DIR}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        spdlog::spdlog
        )
