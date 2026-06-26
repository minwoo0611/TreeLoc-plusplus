#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "treelocpp/types.h"

namespace treelocpp {

std::vector<Pose> ReadTrajectory(const std::filesystem::path& path);
std::vector<Tree> ReadTreeCsv(const std::filesystem::path& path);
bool HasFrameCsv(const std::filesystem::path& root, int index);
std::vector<int> DiscoverFrameIndices(const std::filesystem::path& root, int max_frames);
std::vector<std::string> TreeCsvSchemaErrors(const std::filesystem::path& root,
                                             int max_frames,
                                             const std::string& label);
std::string DatasetName(const std::filesystem::path& root);

}  // namespace treelocpp
