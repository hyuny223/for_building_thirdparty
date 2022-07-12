cmake_minimum_required(VERSION 3.16)
project(untitled)

set(CMAKE_CXX_STANDARD 17)


############################ Thirdparty Building ############################

find_package(OpenCV REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/opencv/install/lib/cmake/opencv4)
find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/eigen/install/share/eigen3/cmake)
find_package(Pangolin REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/pangolin/install/lib/cmake/Pangolin)
find_package(Ceres REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/ceres/install/lib/cmake/Ceres)
find_package(spdlog 1.10.0 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/Thirdparty/spdlog/install/lib/cmake/spdlog)

if (OpenCV_FOUND)
    set(OpenCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV FOUND - ${OpenCV_DIR}")
endif()
if (Eigen3_FOUND)
    set(Eigen3_LIBRARIES Eigen3::Eigen)
    set(Eigen3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
    message(STATUS "Eigen3 FOUND - ${Eigen3_INCLUDE_DIR}")
endif()
if (Pangolin_FOUND)
    set(Pangolin_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/pangolin/install/include)
    set(Pangolin_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/pangolin/install/lib/libpango_core.so)
    message(STATUS "Pangolin FOUND - ${Pangolin_DIR}")
endif()
if (Ceres_FOUND)
    set(Ceres_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/ceres/install/include)
    message(STATUS "Ceres FOUND - ${Ceres_DIR}")
endif()
if (spdlog_FOUND)
    set(spdlog_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/spdlog/install/include)
    set(spdlog_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/spdlog/install/lib)
    message(STATUS "spdlog FOUND - ${spdlog_DIR}")
endif()



############################ Modules Building ############################

add_subdirectory(modules)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIR}
        ${Ceres_INCLUDE_DIR}
        ${spdlog_INCLUDE_DIR}

        module1/include
        module2/include
        module3/include
        module4/include
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${spdlog_LIBRARIES}
        module1
        module2
        module3
        module4
        )
