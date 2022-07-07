#include "keyframe.h"


std::vector<std::shared_ptr<Data::KeyFrame>> Data::KeyFrameVec::mvKeyFrameVec = {};


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
        this->miKeyFrameNum = mvKeyFrameVec.size();
    };


    std::shared_ptr<Data::Frame> KeyFrame::getKeyFrame()
    {
        return mpFrame;
    }


    void KeyFrame::setWorldPosition(cv::Point3d worldPosition)
    {
        mWorldPosition = worldPosition;
    }

    cv::Point3d KeyFrame::getWorldPosition()
    {
        return mWorldPosition;
    }


    void KeyFrame::setReprojPoints(std::vector<cv::KeyPoint> reprojPoints)
    {
        mvReprojPoints = reprojPoints;
    }

    std::vector<cv::KeyPoint> KeyFrame::getReprojPoints()
    {
        return mvReprojPoints;
    }

    void KeyFrame::setW2CRotationMat(cv::Mat rotation)
    {
        mRcw = rotation;
    }

    cv::Mat KeyFrame::getW2CRotationMat()
    {
        return mRcw;
    }

    void KeyFrame::setW2CTranslationMat(cv::Mat translation)
    {
        mTcw = translation;
    }

    cv::Mat KeyFrame::getW2CTranslationMat()
    {
        return mTcw;
    }

    int KeyFrame::getKeyFrameNum()
    {
        return miKeyFrameNum;
    }

    std::shared_ptr<Data::KeyFrame> KeyFrame::getPrevKeyFrame()
    {
        return mvKeyFrameVec[miKeyFrameNum-1];
    }
    std::shared_ptr<Data::KeyFrame> KeyFrame::getWorldKeyFrame()
    {
        return mvKeyFrameVec[0];
    }

}
