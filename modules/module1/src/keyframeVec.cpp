#include <iostream>
#include <vector>

#include "keyframeVec.h"
#include "keyframe.h"

namespace Data
{
    std::vector<std::shared_ptr<Data::KeyFrame>> KeyFrameVec::getKeyFrameVec()
    {
        return mvKeyFrameVec;
    }

    void KeyFrameVec::setKeyFrameVec(std::shared_ptr<Data::KeyFrame> keyframe)
    {
        mvKeyFrameVec.push_back(keyframe);
    }
}
