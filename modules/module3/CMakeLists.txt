cmake_minimum_required(VERSION 3.16)
project(module3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE3_SOURCE_FILES
        src/motionEstimation.cpp
        src/similarity.cpp
        )

find_package(OpenCV REQUIRED HINTS ../../Thirdparty/opencv/install/lib/cmake/opencv4)

add_library(${PROJECT_NAME} SHARED
        ${MODULE3_SOURCE_FILES}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        # ${CMAKE_SOURCE_DIR}/modules/module1/build/libmodule1.so
        # ${CMAKE_SOURCE_DIR}/modules/module2/build/libmodule2.so
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/modules/module1/include
        ${CMAKE_SOURCE_DIR}/modules/module2/include
        )


