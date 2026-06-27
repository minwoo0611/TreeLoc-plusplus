#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "treelocpp/types.h"

namespace treelocpp {

struct Config {
    std::string mode = "intra";
    std::filesystem::path dataset_root = "data/Wild_V02";
    std::filesystem::path query_root = "data/Wild_V03";
    std::filesystem::path database_root = "data/Wild_V02";
    std::vector<std::filesystem::path> query_roots;
    std::vector<std::filesystem::path> database_roots;
    std::vector<std::string> query_labels;
    std::vector<std::string> database_labels;

    int max_frames = 0;
    double spatial_threshold = 5.0;
    bool use_test_polygons = false;
    std::string test_polygon_family = "auto";
    int temporal_min_separation = 50;
    int recall_k = 1;
    int histogram_k = 100;
    int rerank_k = 10;

    int knn_k = 10;
    double min_dist = 1.0;
    double max_dist = 30.0;
    double delta_l = 0.1;
    long long rho = 100007;
    long long hash_modulus = 50000000;

    int number_of_cluster = 30;
    double local_radius = 30.0;
    double tree_score_min = 0.1;
    bool neighbor_augment = true;
    bool neighbor_past_only = true;
    int neighbor_max_scenes = 5;
    double neighbor_radius = 5.0;
    int min_reconstructed_per_frame = 15;
    double dedup_distance = 0.4;
    bool tree_axis_alignment_enabled = true;
    double dataset_yaw_deg = 0.0;
    double query_yaw_deg = 0.0;
    double database_yaw_deg = 0.0;

    double min_radius = 0.0;
    double max_radius = 0.8;
    int total_section = 8;
    double bin_width = 0.15;
    double spatial_bin_interval = 6.0;
    double spatial_bin_padding = 1.0;
    int spatial_bin_count = 5;
    double spatial_bin_min = 0.0;
    double spatial_bin_max = 30.0;
    std::vector<RangeBin> spatial_range_bins;

    bool tdh_use_rec_only = false;
    bool pairwise_use_rec_only = true;
    double pairwise_weight = 0.5;
    double pairwise_min_dist = 0.0;
    double pairwise_max_dist = 10.0;
    int pairwise_bins = 40;
    int pairwise_max_pairs = 5000;
    bool pairwise_soft_binning = false;

    bool use_t_aware_overlap = true;
    double t_aware_tau = 5.0;
    double t_aware_power = 2.0;
    std::string t_aware_mode = "exp";

    bool use_yaw_voting = true;
    double yaw_bin_deg = 5.0;
    double yaw_inlier_tol_deg = 10.0;
    int yaw_min_tri_inliers = 2;

    bool use_dbh_triangle_match = true;
    double dbh_diff_tol = 0.2;
    double match_distance_tol = 0.4;

    bool use_vertical = true;
    int vertical_ransac_iters = 200;
    int vertical_min_sample = 4;
    double z_inlier_tol = 0.1;
    double z_inlier_ratio = 0.5;

    bool save_pose_edges = false;
    std::filesystem::path pose_edge_output_dir = "results/pose_edges";
    std::string pose_edge_prefix = "pose";
};

std::filesystem::path DefaultConfigPath(const std::string& mode);
bool LoadConfig(const std::filesystem::path& path, Config& config, std::string* error);
void RefreshDerivedConfig(Config& config);
bool ValidateConfig(const Config& config, std::string* error);
std::vector<RangeBin> BuildRadiusBins(const Config& config);

}  // namespace treelocpp
