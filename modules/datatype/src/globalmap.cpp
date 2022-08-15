
#include <vector>
#include "globalmap.h"
#include "localmap.h"

namespace Data
{
    std::vector<std::shared_ptr<Data::LocalMap>> GlobalMap::getGlobalMap()
    {
        return mvGlobalMap;
    }
    void GlobalMap::setGlobalMap(std::shared_ptr<Data::LocalMap> localMap)
    {
        mvGlobalMap.push_back(localMap);
    }
}

