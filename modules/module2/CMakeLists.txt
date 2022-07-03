cmake_minimum_required(VERSION 3.16)
project(module2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE2_SOURCE_FILES
        src/tracking.cpp
        )

find_package(OpenCV REQUIRED HINTS ../../Thirdparty/opencv/install/lib/cmake/opencv4)

add_library(${PROJECT_NAME} SHARED
        ${MODULE2_SOURCE_FILES}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${CMAKE_SOURCE_DIR}/modules/module1/build/libmodule1.so
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/modules/module1/include
        )


