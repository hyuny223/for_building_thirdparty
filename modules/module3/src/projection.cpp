#include "opencv2/opencv.hpp"

#include "projection.h"
#include "tracking.h"
#include "spdlog/spdlog.h"

void doProjection(std::shared_ptr<Data::KeyFrame> prevKeyFrame,
                std::shared_ptr<Data::KeyFrame> currKeyFrame)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3,CV_64FC1,k);

    auto point3D = prevKeyFrame->getFramePoint3d();

    double d[] = {0,0,0,0};
    cv::Mat distCoeffs(4, 1, CV_64F, d);

    std::vector<cv::Point2d> newFramePoint2d;
    cv::projectPoints(point3D, prevKeyFrame->getRotationMat(), prevKeyFrame->getTranslationMat(), K, distCoeffs, newFramePoint2d);
    spdlog::info("- projectPoints complete");

    std::vector<cv::KeyPoint> reprojPoints;

    for(int i = 0; i < newFramePoint2d.size(); ++i)
    {
        reprojPoints.push_back(cv::KeyPoint(newFramePoint2d[i], 1.0));

#if 0
        std::cout << "origin (x,y) = (" << currKeyFrame->getGoodMatches()[i].x << ", " << currKeyFrame->getGoodMatches()[i].y << ")" << std::endl;
        std::cout << "projected (x,y) = (" << reprojPoints[i].pt.x << ", " << reprojPoints[i].pt.y << ")" << std::endl;
        std::cout << "\n\n";
#endif
    }

    currKeyFrame->setReprojPoints(reprojPoints);
    spdlog::info("- setReprojPoints complete");
}
