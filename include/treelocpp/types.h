#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace treelocpp {

using RangeBin = std::pair<double, double>;

struct Pose {
    double stamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

struct Tree {
    Eigen::Matrix3d axis = Eigen::Matrix3d::Identity();
    bool has_axis = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double dbh = 0.0;
    double dbh_approximation = 0.0;
    double score = 1.0;
    int reconstructed = 1;
    int number_clusters = 3;
};

struct TriangleSet {
    std::vector<std::array<int, 3>> simplices;
};

struct FrameData {
    int index = -1;
    Pose pose;
    Eigen::Matrix4d alignment_transform = Eigen::Matrix4d::Identity();
    std::vector<Tree> trees;
    std::vector<Eigen::Vector2d> centers;
    Eigen::MatrixXd tdh;
    Eigen::VectorXd pairwise;
    TriangleSet triangles;
    std::vector<std::pair<long long, int>> hashes;
    std::unordered_map<long long, int> hash_counts;
};

struct Dataset {
    std::filesystem::path root;
    std::vector<Pose> trajectory;
    std::vector<FrameData> frames;
    std::unordered_map<int, size_t> frame_to_slot;
};

struct MatchPair {
    int query = -1;
    int candidate = -1;
};

struct Transform2D {
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    double overlap = 0.0;
    std::vector<MatchPair> pairs;
    bool ok = false;
};

struct PoseCorrection {
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double axis_roll = 0.0;
    double axis_pitch = 0.0;
    int inliers = 0;
};

struct CandidateResult {
    int query_index = -1;
    int candidate_index = -1;
    double retrieval_score = 0.0;
    int hash_score = 0;
    Transform2D transform;
    PoseCorrection vertical;
    double spatial_error = 0.0;
    bool true_neighbor = false;
};

struct EvaluationSummary {
    int queries = 0;
    int evaluated = 0;
    int true_positive = 0;
    double recall = 0.0;
    double mean_spatial_error = 0.0;
    double mean_overlap = 0.0;
    double mean_z = 0.0;
    double mean_roll_deg = 0.0;
    double mean_pitch_deg = 0.0;
    int zero_gt_top_histogram = 0;
    int zero_gt_top_hash = 0;
    double mean_gt_top_histogram = 0.0;
    int localization_2d_recall_success = 0;
    int localization_2d_true_success = 0;
    int localization_6d_recall_success = 0;
    int localization_6d_true_success = 0;
    double localization_txy_sum = 0.0;
    double localization_yaw_sum = 0.0;
    double localization_txyz_sum = 0.0;
    double localization_z_sum = 0.0;
    double localization_roll_sum = 0.0;
    double localization_pitch_sum = 0.0;
    double localization_yaw6_sum = 0.0;
    double localization_rot_norm_sum = 0.0;
};

}  // namespace treelocpp
