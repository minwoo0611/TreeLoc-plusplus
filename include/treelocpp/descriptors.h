#pragma once

#include <filesystem>
#include <vector>

#include "treelocpp/config.h"
#include "treelocpp/types.h"

namespace treelocpp {

Dataset LoadDataset(const std::filesystem::path& root,
                    const Config& config,
                    bool past_only);
std::vector<Tree> SelectTrees(const std::vector<Tree>& trees, const Config& config);
Eigen::MatrixXd ComputeTDH(const std::vector<Tree>& trees,
                           const std::vector<RangeBin>& spatial_bins,
                           const std::vector<RangeBin>& radius_bins);
Eigen::VectorXd ComputePDH(const std::vector<Tree>& trees, const Config& config);
TriangleSet ComputeKnnTriangles(const std::vector<Eigen::Vector2d>& centers,
                                int k,
                                double min_dist,
                                double max_dist);
std::vector<std::pair<long long, int>> TriangleHashes(const TriangleSet& triangles,
                                                      const std::vector<Eigen::Vector2d>& centers,
                                                      double delta_l,
                                                      long long rho,
                                                      long long hash_modulus);
double ChiSquared(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs);
double ChiSquared(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs);

}  // namespace treelocpp
