#include <iostream>
#include "opencv2/opencv.hpp"
#include "datatype.h"
#include "tracking.h"


int main()
{
    cv::Mat image_L = cv::imread("/root/dataset/sequences/00/image_0/000000.png");
    cv::Mat image_R = cv::imread("/root/dataset/sequences/00/image_0/000001.png");

    auto tracking = std::make_shared<Frontend::Tracking>();

    int nFeatures = 500;
    tracking->detectFeatures(image_L, image_R, nFeatures);
    tracking->matchFeatures(image_L,image_R);
    tracking->computeFundamentalMatrix();

    for(int i = 0; i < tracking->framePoint->getGoodMatches_L().size(); ++i)
    {
        std::cout << "point num = " << i << std::endl;
        tracking->computeTriangulation(tracking->framePoint->getGoodMatches_L()[i], tracking->framePoint->getGoodMatches_R()[i]);
        std::cout << "next\n";
    }

    // cv::imshow("image_L", image_L);
    // cv::imshow("image_R", image_R);
    // cv::waitKey(0);

    return 0;
}
