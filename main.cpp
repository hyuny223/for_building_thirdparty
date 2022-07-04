#include <iostream>

#include "opencv2/opencv.hpp"
#include "framepoint.h"
#include "frame.h"
#include "keyframe.h"
#include "tracking.h"
#include "similarity.h"


int main()
{
    cv::Mat image_L = cv::imread("/root/dataset/sequences/00/image_0/000000.png");
    cv::Mat image_R = cv::imread("/root/dataset/sequences/00/image_0/000001.png");

    std::shared_ptr<Data::Frame> frame_L = std::make_shared<Data::Frame>(image_L);
    std::shared_ptr<Data::Frame> frame_R = std::make_shared<Data::Frame>(image_R);

    auto tracking = std::make_shared<Frontend::Tracking>(frame_L, frame_R);

    int nFeatures = 500;
    tracking->detectFeatures(nFeatures);
    tracking->matchFeatures();
    tracking->computeEssentialMatrix();

    for(int i = 0; i < frame_L->getGoodMatches().size(); ++i)
    {
        // std::cout << "point num = " << i << std::endl;
        tracking->computeTriangulation(frame_L->getGoodMatches()[i], frame_R->getGoodMatches()[i]);
        std::cout << "next\n";
    }

    cv::imshow("image_L", image_L);
    cv::imshow("image_R", image_R);
    cv::waitKey(0);

    return 0;
}
