#pragma once

#include <iostream>
#include <vector>
#include "keyframe.h"

namespace Data
{
    class KeyFrameVec
    {
        private:
            std::vector<std::shared_ptr<Data::KeyFrame>> mvKeyFrameVec = {};

        public:
            static KeyFrameVec& GetInstance()
            {
                static KeyFrameVec* keyframeVec = new KeyFrameVec();
                return *keyframeVec;
            }

            std::vector<std::shared_ptr<Data::KeyFrame>> getKeyFrameVec();
            void setKeyFrameVec(std::shared_ptr<Data::KeyFrame> keyframe);

        private:
            KeyFrameVec() = default;

            // Delete copy/move so extra instances can't be created/moved.
            KeyFrameVec(const KeyFrameVec&) = delete;
            KeyFrameVec& operator=(const KeyFrameVec&) = delete;
            KeyFrameVec(KeyFrameVec&&) = delete;
            KeyFrameVec& operator=(KeyFrameVec&&) = delete;

    };
}
