#include "datatype.h"

namespace Data
{

    void FramePoint::setFramePoint3d_L(const std::vector<cv::Point3f>& framePoints)
    {
        mvFramePoints3d_L = framePoints;
    }

    void FramePoint::setFramePoint3d_R(const std::vector<cv::Point3f>& framePoints)
    {
        mvFramePoints3d_R = framePoints;
    }

    std::vector<cv::Point3f> FramePoint::getFramePoint3d_L()
    {
        return mvFramePoints3d_L;
    }

    std::vector<cv::Point3f> FramePoint::getFramePoint3d_R()
    {
        return mvFramePoints3d_R;
    }

    void FramePoint::setFramePoint2d_L(const std::vector<cv::KeyPoint>& framePoints)
    {
        mvFramePoints2d_L = framePoints;
    }

    void FramePoint::setFramePoint2d_R(const std::vector<cv::KeyPoint>& framePoints)
    {
        mvFramePoints2d_R = framePoints;
    }

    std::vector<cv::KeyPoint> FramePoint::getFramePoint2d_L()
    {
        return mvFramePoints2d_L;
    }

    std::vector<cv::KeyPoint> FramePoint::getFramePoint2d_R()
    {
        return mvFramePoints2d_R;
    }


    void FramePoint::setDescriptors_L(const cv::Mat& descriptors)
    {
        mDescriptors_L = descriptors;
    }

    void FramePoint::setDescriptors_R(const cv::Mat& descriptors)
    {
        mDescriptors_R = descriptors;
    }


    cv::Mat FramePoint::getDescriptors_L()
    {
        return mDescriptors_L;
    }

    cv::Mat FramePoint::getDescriptors_R()
    {
        return mDescriptors_R;
    }


    void FramePoint::setMatches(const std::vector<cv::DMatch>& match)
    {
        mMatches = match;
    }

    std::vector<cv::DMatch> FramePoint::getMatches()
    {
        return mMatches;
    }


    void FramePoint::setGoodMatches_L(const std::vector<cv::Point2f>& goodMatch)
    {
        mvGoodMatches_L = goodMatch;
    }
    void FramePoint::setGoodMatches_R(const std::vector<cv::Point2f>& goodMatch)
    {
        mvGoodMatches_R = goodMatch;
    }

    std::vector<cv::Point2f> FramePoint::getGoodMatches_L()
    {
        return mvGoodMatches_L;
    }
    std::vector<cv::Point2f> FramePoint::getGoodMatches_R()
    {
        return mvGoodMatches_R;
    }

    void FramePoint::setFundamentalMatrix(const cv::Mat& F)
    {
        mmFundamentalMatrix = F;
    }
    void FramePoint::setEssentialMatrix(const cv::Mat& E)
    {
        mmEssentialMatrix = E;
    }
    void FramePoint::setRotationMatrix(const cv::Mat& R)
    {
        mmRoationMatrix = R;
    }
    void FramePoint::setTranslationMatrix(const cv::Mat& t)
    {
        mmTranslationMatrix = t;
    }

    cv::Mat FramePoint::getFundamentalMatrix()
    {
        return mmFundamentalMatrix;
    }

    cv::Mat FramePoint::getEssentialMatrix()
    {
        return mmEssentialMatrix;
    }

    cv::Mat FramePoint::getRotationMatrix()
    {
        return mmRoationMatrix;
    }
    cv::Mat FramePoint::getTranslationMatrix()
    {
        return mmTranslationMatrix;
    }

}
