#include <iostream>
#include <vector>
#include "keyframe.h"
#include "localmap.h"

namespace Data
{
    void LocalMap::setLocalMap(Data::KeyFrame keyFrame)
    {
        mvLocalMap.push_back(keyFrame);

        if(mvLocalMap.size() == 1)
        {
            // 포즈 값
        }
    }

    std::vector<Data::KeyFrame> LocalMap::getLocalMap()
    {
        return mvLocalMap;
    }

    cv::Point3d LocalMap::getLocalPose()
    {
        return mLocalPose;
    }

    void LocalMap::resetLocalMap()
    {
        mvLocalMap.clear();
    }
}
