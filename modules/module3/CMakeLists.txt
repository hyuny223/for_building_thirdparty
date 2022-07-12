cmake_minimum_required(VERSION 3.16)
project(module3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MODULE3_SOURCE_FILES
        src/similarity.cpp
        src/projection.cpp
        )

find_package(OpenCV 4.4.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/install/lib/cmake/opencv4)
find_package(spdlog 1.10.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/lib/cmake/spdlog)

if (spdlog_FOUND)
    set(spdlog_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/include)
    set(spdlog_LIBRARIES ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/lib)
    message(STATUS "spdlog FOUND - ${spdlog_DIR}")
endif()

add_library(${PROJECT_NAME} SHARED
        ${MODULE3_SOURCE_FILES}
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${spdlog_LIBRARIES}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/modules/module1/include
        ${CMAKE_SOURCE_DIR}/modules/module2/include
        ${spdlog_INCLUDE_DIR}
        )


