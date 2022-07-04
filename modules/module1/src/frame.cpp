#include "opencv2/opencv.hpp"
#include "frame.h"

namespace Data
{
    Frame::Frame(const cv::Mat image)
    :mmFrame(image){};

    void Frame::setFrame(const cv::Mat& image)
    {
        mmFrame = image;
    }
    cv::Mat Frame::getFrame()
    {
        return mmFrame;
    }

    void Frame::setDescriptors(const cv::Mat& descriptors)
    {
        mDescriptors = descriptors;
    }

    cv::Mat Frame::getDescriptors()
    {
        return mDescriptors;
    }

    // void Frame::setMatches(const std::vector<cv::DMatch>& match)
    // {
    //     mMatches = match;
    // }

    // std::vector<cv::DMatch> Frame::getMatches()
    // {
    //     return mMatches;
    // }

    void Frame::setGoodMatches(const std::vector<cv::Point2f>& goodMatch)
    {
        mvGoodMatches = goodMatch;
    }

    std::vector<cv::Point2f> Frame::getGoodMatches()
    {
        return mvGoodMatches;
    }

    void Frame::setFundamentalMatrix(const cv::Mat& F)
    {
        mmFundamentalMatrix = F;
    }

    void Frame::setEssentialMatrix(const cv::Mat& E)
    {
        mmEssentialMatrix = E;
    }

    void Frame::setRotationMatrix(const cv::Mat& R)
    {
        mmRoationMatrix = R;
    }

    void Frame::setTranslationMatrix(const cv::Mat& t)
    {
        mmTranslationMatrix = t;
    }

    cv::Mat Frame::getFundamentalMatrix()
    {
        return mmFundamentalMatrix;
    }

    cv::Mat Frame::getEssentialMatrix()
    {
        return mmEssentialMatrix;
    }

    cv::Mat Frame::getRotationMatrix()
    {
        return mmRoationMatrix;
    }

    cv::Mat Frame::getTranslationMatrix()
    {
        return mmTranslationMatrix;
    }

}
