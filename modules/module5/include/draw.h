#pragma once

#include <cmath>

#include "opencv2/opencv.hpp"

#include "pangolin/pangolin.h"
#include "Eigen/Core"
#include "Eigen/Geometry"


void drawCurrentFrame(size_t i, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses);
void drawKeytFrame(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> keyFrame);
void drawCoordinate(float scale);
void drawLine(size_t i,std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses, bool drawLine);
