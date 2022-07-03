cmake_minimum_required(VERSION 3.16)
project(module1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE1_SOURCE_FILES
        src/datatype.cpp
        )

find_package(OpenCV REQUIRED HINTS ../../Thirdparty/opencv/install/lib/cmake/opencv4)

add_library(${PROJECT_NAME} SHARED
        ${MODULE1_SOURCE_FILES}
        )


target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        )
