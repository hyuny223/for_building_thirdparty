#include "opencv2/opencv.hpp"

#include "projection.h"
#include "tracking.h"

void doProjection(std::shared_ptr<Data::KeyFrame> currKeyFrame)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3,CV_64FC1,k);

    auto point3D = currKeyFrame->getFramePoint3d();

    cv::Mat transformMat = currKeyFrame->getTransformMat();
    cv::Mat rotationMat, translationMat;
    Frontend::splitTransformMat(transformMat, rotationMat, translationMat);

    double d[] = {0,0,0,0};
    cv::Mat distCoeffs(4, 1, CV_64F, d);

    std::vector<cv::Point2d> newFramePoint2d;
    cv::projectPoints(point3D, rotationMat, transformMat, K, distCoeffs, newFramePoint2d);

    auto reprojPoints = currKeyFrame->getReprojPoints();
    for(int i = 0; i < newFramePoint2d.size(); ++i)
    {
        reprojPoints.push_back(cv::KeyPoint(newFramePoint2d[i], 1.0));
    }

    currKeyFrame->setReprojPoints(reprojPoints);
}
