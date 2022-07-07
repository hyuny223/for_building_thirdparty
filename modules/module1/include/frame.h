#pragma once

#include "framepoint.h"

namespace Data
{
    class Frame: public FramePoint
    {
        protected:
            cv::Mat mmFrame;
            std::shared_ptr<Data::FramePoint> mpFramePoint;

            cv::Mat mDescriptors;

            std::vector<cv::DMatch> mMatches;

            std::vector<cv::Point2d> mvGoodMatches;

            cv::Mat mmFundamentalMat;
            cv::Mat mmEssentialMat;
            cv::Mat mmRoationMat;
            cv::Mat mmTranslationMat;
            cv::Mat mmTransformMat;
            cv::Mat mmw2c = cv::Mat::eye(4, 4, CV_32F);

            double mdScale{1000};

        public:
            Frame() = default;
            Frame(const cv::Mat image);
            virtual ~Frame() = default;

            void setFrame(const cv::Mat& image);
            cv::Mat getFrame();

            void setDescriptors(const cv::Mat& descriptors);
            cv::Mat getDescriptors();

            // void setMatches(const std::vector<cv::DMatch>& match);
            // std::vector<cv::DMatch> getMatches();

            void setGoodMatches(const std::vector<cv::Point2d>& goodMatch);
            std::vector<cv::Point2d> getGoodMatches();

            void setFundamentalMat(const cv::Mat& F);
            void setEssentialMat(const cv::Mat& E);
            void setRotationMat(const cv::Mat& R);
            void setTranslationMat(const cv::Mat& t);
            void setTransformMat(const cv::Mat& T);

            cv::Mat getFundamentalMat();
            cv::Mat getEssentialMat();
            cv::Mat getRotationMat();
            cv::Mat getTranslationMat();
            cv::Mat getTransformMat();

            void setScale(const double& scale);
            double getScale();
    };
}
