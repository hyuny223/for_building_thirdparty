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

    void Frame::setGoodMatches(std::vector<cv::Point2d> goodMatch)
    {
        mvGoodMatches = goodMatch;
    }

    std::vector<cv::Point2d> Frame::getGoodMatches()
    {
        return mvGoodMatches;
    }

    void Frame::setFundamentalMat(const cv::Mat& F)
    {
        mmFundamentalMat = F;
    }

    void Frame::setEssentialMat(const cv::Mat& E)
    {
        mmEssentialMat = E;
    }

    void Frame::setRotationMat(const cv::Mat& R)
    {
        mmRotaionMat = R;
    }

    void Frame::setTranslationMat(const cv::Mat& t)
    {
        mmTranslationMat = t;
    }

    cv::Mat Frame::getFundamentalMat()
    {
        return mmFundamentalMat;
    }

    cv::Mat Frame::getEssentialMat()
    {
        return mmEssentialMat;
    }

    cv::Mat Frame::getRotationMat()
    {
        return mmRotaionMat;
    }

    cv::Mat Frame::getTranslationMat()
    {
        return mmTranslationMat;
    }


    void Frame::setTransformMat(const cv::Mat& T)
    {
        mmTransformMat = T;
    }

    cv::Mat Frame::getTransformMat()
    {
        return mmTransformMat;
    }

    void Frame::setScale(const double& scale)
    {
        mdScale = scale;
    }
    double Frame::getScale()
    {
        return mdScale;
    }
}
