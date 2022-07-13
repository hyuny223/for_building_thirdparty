#include "keyframe.h"


namespace Data
{

    KeyFrame::KeyFrame(std::shared_ptr<Data::Frame> frame)
    :mpFrame(frame)
    {
        this->setDescriptors(frame->getDescriptors());
        this->setEssentialMat(frame->getEssentialMat());
        this->setFrame(frame->getFrame());
        this->setFramePoint2d(frame->getFramePoint2d());
        this->setFramePoint3d(frame->getFramePoint3d());
        this->setFundamentalMat(frame->getFundamentalMat());
        this->setGoodMatches(frame->getGoodMatches());
        this->setRotationMat(frame->getRotationMat());
        this->setScale(frame->getScale());
        this->setTranslationMat(frame->getTranslationMat());
    };


    std::shared_ptr<Data::Frame> KeyFrame::getKeyFrame()
    {
        return mpFrame;
    }


    // void KeyFrame::setWorldPosition(cv::Point3d worldPosition)
    // {
    //     mWorldPosition = worldPosition;
    // }

    // cv::Point3d KeyFrame::getWorldPosition()
    // {
    //     return mWorldPosition;
    // }


    void KeyFrame::setReprojPoints(std::vector<cv::KeyPoint> reprojPoints)
    {
        mvReprojPoints = reprojPoints;
    }

    std::vector<cv::KeyPoint> KeyFrame::getReprojPoints()
    {
        return mvReprojPoints;
    }

    // void KeyFrame::setW2CRotationMat(cv::Mat rotation)
    // {
    //     mRcw = rotation;
    // }

    // cv::Mat KeyFrame::getW2CRotationMat()
    // {
    //     return mRcw;
    // }

    // void KeyFrame::setW2CTranslationMat(cv::Mat translation)
    // {
    //     mTcw = translation;
    // }

    // cv::Mat KeyFrame::getW2CTranslationMat()
    // {
    //     return mTcw;
    // }

    void KeyFrame::setw2c(cv::Mat w2c)
    {
        mw2c = w2c;
    }

    cv::Mat KeyFrame::getw2c()
    {
        return mw2c;
    }


    void KeyFrame::setQuaternion(cv::Mat q)
    {
        mQuaternion = q;
    }
    cv::Mat KeyFrame::getQuaternion()
    {
        return mQuaternion;
    }
}
