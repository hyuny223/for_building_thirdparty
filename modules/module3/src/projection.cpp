#include "opencv2/opencv.hpp"

#include "projection.h"

void Projection::doProjection(std::shared_ptr<Data::KeyFrame> keyFrame)
{
    double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};
    cv::Mat K(3,3,CV_64FC1,k);

    std::shared_ptr<Data::KeyFrame> prevKeyFrame = keyFrame->getPrevKeyFrame();



    // std::vector<cv::Point3d> vFramePoint3d = keyFrame->getFramePoint3d();
    // std::vector<cv::KeyPoint> vReprojectPoints = keyFrame->getReprojPoints();

    // std::cout << vFramePoint3d.size() << std::endl;

    // vReprojectPoints.reserve(vFramePoint3d.size());

    // for (int i = 0; i < vFramePoint3d.size(); ++i)
    // {

    //     double x = K.ptr<double>(0)[0] * (vFramePoint3d[i].x / vFramePoint3d[i].z) + K.ptr<double>(0)[2];
    //     double y = K.ptr<double>(1)[1] * (vFramePoint3d[i].y / vFramePoint3d[i].z) + K.ptr<double>(1)[2];

    //     vReprojectPoints[i].pt = {x, y};

    //     std::vector<cv::Point2d> goodMatches = keyFrame->getGoodMatches();

    //     for (int i = 0; i < goodMatches.size(); ++i)
    //     {
    //         // std::cout << "Origin : ("<< goodMatches[i].x << ", " << goodMatches[i].y << ")\n";
    //         // std::cout << "Reproj : ("<< vReprojectPoints[i].pt.x << ", " << vReprojectPoints[i].pt.y << ")\n";
    //         // std::cout << "\n\n";
    //     }
    // }
}
