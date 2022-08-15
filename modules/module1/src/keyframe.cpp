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

    void KeyFrame::setReprojPoints(std::vector<cv::KeyPoint> reprojPoints)
    {
        mvReprojPoints = reprojPoints;
    }

    std::vector<cv::KeyPoint> KeyFrame::getReprojPoints()
    {
        return mvReprojPoints;
    }

    void KeyFrame::setQuaternion(cv::Mat q)
    {
        mQuaternion = q;
    }
    cv::Mat KeyFrame::getQuaternion()
    {
        return mQuaternion;
    }

    void KeyFrame::setWorldPosition(cv::Point3d position)
    {
        mWorldPosition = position;
    }
    cv::Point3d KeyFrame::getWorldPosition()
    {
        return mWorldPosition;
    }

    void KeyFrame::setWorld2CamTransformMat(cv::Mat transform)
    {
        mWorld2CamTransform = transform;
    }
    cv::Mat KeyFrame::getWorld2CamTransformMat()
    {
        return mWorld2CamTransform;
    }
}
