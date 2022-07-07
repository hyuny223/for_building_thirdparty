#include "opencv2/opencv.hpp"

#include "projection.h"
#include "tracking.h"

void doProjection(std::shared_ptr<Data::KeyFrame> currKeyFrame, std::vector<cv::Point2d>& newFramePoint2d)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3,CV_64FC1,k);

    auto point3D = currKeyFrame->getFramePoint3d();

    cv::Mat transformMat = currKeyFrame->getTransformMat();
    cv::Mat rotationMat, translationMat;
    Frontend::splitTransformMat(transformMat, rotationMat, translationMat);

    double d[] = {0,0,0,0};
    cv::Mat distCoeffs(4, 1, CV_64F, d);

    cv::projectPoints(point3D, rotationMat, transformMat, K, distCoeffs, newFramePoint2d);

}
