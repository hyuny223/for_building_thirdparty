#pragma once

#include "opencv2/opencv.hpp"
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

            cv::Mat mmFundamentalMatrix;
            cv::Mat mmEssentialMatrix;
            cv::Mat mmRoationMatrix;
            cv::Mat mmTranslationMatrix;

            double mdScale{1000};

        public:
            Frame() = default;
            Frame(const cv::Mat image);
            // ~Frame();

            void setFrame(const cv::Mat& image);
            cv::Mat getFrame();

            void setDescriptors(const cv::Mat& descriptors);
            cv::Mat getDescriptors();

            // void setMatches(const std::vector<cv::DMatch>& match);
            // std::vector<cv::DMatch> getMatches();

            void setGoodMatches(const std::vector<cv::Point2d>& goodMatch);
            std::vector<cv::Point2d> getGoodMatches();

            void setFundamentalMatrix(const cv::Mat& F);
            void setEssentialMatrix(const cv::Mat& E);
            void setRotationMatrix(const cv::Mat& R);
            void setTranslationMatrix(const cv::Mat& t);

            cv::Mat getFundamentalMatrix();
            cv::Mat getEssentialMatrix();
            cv::Mat getRotationMatrix();
            cv::Mat getTranslationMatrix();

            void setScale(const double& scale);
            double getScale();
    };
}
