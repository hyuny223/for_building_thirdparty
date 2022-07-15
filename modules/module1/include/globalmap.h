#pragma once

#include <iostream>
#include <vector>
#include "localmap.h"

namespace Data
{
    class GlobalMap
    {
        private:
            std::vector<std::shared_ptr<Data::LocalMap>> mvGlobalMap;

        public:
            static GlobalMap& GetInstance()
            {
                static GlobalMap* globalMap = new GlobalMap();
                return *globalMap;
            }

            std::vector<std::shared_ptr<Data::LocalMap>> getGlobalMap();
            void setGlobalMap(std::shared_ptr<Data::LocalMap> localMap);

        private:
            GlobalMap() = default;

            // Delete copy/move so extra instances can't be created/moved.
            GlobalMap(const GlobalMap&) = delete;
            GlobalMap& operator=(const GlobalMap&) = delete;
            GlobalMap(GlobalMap&&) = delete;
            GlobalMap& operator=(GlobalMap&&) = delete;

    };
}
