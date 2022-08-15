
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>


std::pair<std::vector<std::string>, std::vector<std::string>> argparse(const std::string& p)
{
    std::string p0 = p + "image_0";
    std::string p1 = p + "image_1";

    std::filesystem::path path0(p0);
    std::filesystem::path path1(p1);

    std::filesystem::directory_iterator itr0(std::filesystem::absolute(path0));
    std::filesystem::directory_iterator itr1(std::filesystem::absolute(path1));

	std::vector<std::string> leftFiles;
	std::vector<std::string> rightFiles;

    while(itr0 != std::filesystem::end(itr0))
    {
        const std::filesystem::directory_entry& entry0 = *itr0;
        const std::filesystem::directory_entry& entry1 = *itr1;

        leftFiles.emplace_back(entry0.path());
        rightFiles.emplace_back(entry1.path());

        ++itr0; ++itr1;
    }

    std::sort(leftFiles.begin(), leftFiles.end());
    std::sort(rightFiles.begin(), rightFiles.end());

    return std::make_pair(leftFiles, rightFiles);
}
