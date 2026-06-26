#include "treelocpp/config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace treelocpp {

namespace {

std::string Trim(const std::string& text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string StripComment(const std::string& line) {
    bool single = false;
    bool dbl = false;
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '\'' && !dbl) single = !single;
        if (line[i] == '"' && !single) dbl = !dbl;
        if (line[i] == '#' && !single && !dbl) return line.substr(0, i);
    }
    return line;
}

std::string ParseString(const std::string& value) {
    std::string out = Trim(value);
    if (out.size() >= 2 &&
        ((out.front() == '"' && out.back() == '"') ||
         (out.front() == '\'' && out.back() == '\''))) {
        out = out.substr(1, out.size() - 2);
    }
    return out;
}

std::string Lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

std::string NormalizeKey(std::string text) {
    text = Lower(Trim(text));
    for (char& ch : text) {
        if (ch == '-') ch = '_';
    }
    return text;
}

bool IsKey(const std::string& key, std::initializer_list<const char*> aliases) {
    return std::any_of(aliases.begin(), aliases.end(),
                       [&](const char* alias) { return key == alias; });
}

template <typename T>
T ParseNumber(const std::string& value, const std::string& key, size_t line_no) {
    std::stringstream ss(value);
    T parsed;
    ss >> parsed;
    if (!ss || !ss.eof()) {
        throw std::runtime_error("invalid value for " + key + " on line " +
                                 std::to_string(line_no));
    }
    return parsed;
}

bool ParseBool(const std::string& value, const std::string& key, size_t line_no) {
    const std::string v = Lower(ParseString(value));
    if (v == "true" || v == "1" || v == "yes") return true;
    if (v == "false" || v == "0" || v == "no") return false;
    throw std::runtime_error("invalid boolean for " + key + " on line " +
                             std::to_string(line_no));
}

void Assign(const std::string& key, const std::string& value, Config& config, size_t line_no) {
    const std::string normalized = NormalizeKey(key);
    if (IsKey(normalized, {"mode", "dataset.mode"})) config.mode = ParseString(value);
    else if (IsKey(normalized, {"dataset_root", "dataset.root", "dataset.dataset_root"})) config.dataset_root = ParseString(value);
    else if (IsKey(normalized, {"query_root", "dataset.query_root"})) config.query_root = ParseString(value);
    else if (IsKey(normalized, {"database_root", "map_root", "dataset.map_root", "dataset.database_root"})) config.database_root = ParseString(value);
    else if (IsKey(normalized, {"max_frames", "frame_limit", "dataset.frame_limit"})) config.max_frames = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_threshold", "gt_radius_m", "evaluation.gt_radius_m"})) config.spatial_threshold = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"use_test_polygons", "evaluation.use_test_regions"})) config.use_test_polygons = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"test_polygon_family", "evaluation.test_region_family"})) config.test_polygon_family = Lower(ParseString(value));
    else if (IsKey(normalized, {"temporal_min_separation", "evaluation.intra_exclusion_frames"})) config.temporal_min_separation = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"recall_k", "retrieval.recall_at_k"})) config.recall_k = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"histogram_k", "retrieval.tdh_top_k"})) config.histogram_k = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"rerank_k", "retrieval.hash_rerank_top_k"})) config.rerank_k = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"knn_k", "triangle_descriptor.knn_neighbors"})) config.knn_k = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"min_dist", "triangle_descriptor.min_edge_m"})) config.min_dist = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"max_dist", "triangle_descriptor.max_edge_m"})) config.max_dist = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"delta_l", "triangle_descriptor.edge_quantization_m"})) config.delta_l = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"rho", "triangle_descriptor.hash_base"})) config.rho = ParseNumber<long long>(value, normalized, line_no);
    else if (IsKey(normalized, {"hash_modulus", "triangle_descriptor.hash_modulus"})) config.hash_modulus = ParseNumber<long long>(value, normalized, line_no);
    else if (IsKey(normalized, {"number_of_cluster", "tree_selection.target_tree_count"})) config.number_of_cluster = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"local_radius", "tree_selection.local_radius_m"})) config.local_radius = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"tree_score_min", "tree_selection.supplemental_score_min"})) config.tree_score_min = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"neighbor_augment", "neighbor_augmentation.enabled"})) config.neighbor_augment = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"neighbor_past_only", "neighbor_augmentation.past_only"})) config.neighbor_past_only = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"neighbor_max_scenes", "neighbor_augmentation.max_neighbor_frames"})) config.neighbor_max_scenes = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"neighbor_radius", "neighbor_augmentation.search_radius_m"})) config.neighbor_radius = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"min_reconstructed_per_frame", "neighbor_augmentation.min_reconstructed_trees"})) config.min_reconstructed_per_frame = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"dedup_distance", "tree_selection.dedup_radius_m"})) config.dedup_distance = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"apply_axis_alignment", "tree_axis_alignment.enabled"})) config.tree_axis_alignment_enabled = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"dataset_yaw_deg", "yaw_offsets.dataset_deg"})) config.dataset_yaw_deg = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"query_yaw_deg", "yaw_offsets.query_deg"})) config.query_yaw_deg = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"database_yaw_deg", "yaw_offsets.map_deg", "yaw_offsets.database_deg"})) config.database_yaw_deg = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"min_radius", "tdh.dbh_min_m"})) config.min_radius = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"max_radius", "tdh.dbh_max_m"})) config.max_radius = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"total_section", "tdh.dbh_sections"})) config.total_section = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"bin_width", "tdh.dbh_bin_width_m"})) config.bin_width = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_bin_interval", "tdh.spatial_bin_interval_m"})) config.spatial_bin_interval = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_bin_padding", "tdh.spatial_bin_padding_m"})) config.spatial_bin_padding = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_bin_count", "tdh.spatial_bin_count"})) config.spatial_bin_count = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_bin_min", "tdh.spatial_min_m"})) config.spatial_bin_min = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"spatial_bin_max", "tdh.spatial_max_m"})) config.spatial_bin_max = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"tdh_use_rec_only", "tdh.use_reconstructed_only"})) config.tdh_use_rec_only = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_use_rec_only", "pairwise_context.use_reconstructed_only"})) config.pairwise_use_rec_only = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_weight", "pairwise_context.weight"})) config.pairwise_weight = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_min_dist", "pairwise_context.min_distance_m"})) config.pairwise_min_dist = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_max_dist", "pairwise_context.max_distance_m"})) config.pairwise_max_dist = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_bins", "pairwise_context.bin_count"})) config.pairwise_bins = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_max_pairs", "pairwise_context.max_sampled_pairs"})) config.pairwise_max_pairs = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"pairwise_soft_binning", "pairwise_context.soft_binning"})) config.pairwise_soft_binning = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"use_t_aware_overlap", "translation_aware_overlap.enabled"})) config.use_t_aware_overlap = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"t_aware_tau", "translation_aware_overlap.tau_m"})) config.t_aware_tau = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"t_aware_power", "translation_aware_overlap.power"})) config.t_aware_power = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"t_aware_mode", "translation_aware_overlap.mode"})) config.t_aware_mode = ParseString(value);
    else if (IsKey(normalized, {"use_yaw_voting", "yaw_voting.enabled"})) config.use_yaw_voting = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"yaw_bin_deg", "yaw_voting.bin_deg"})) config.yaw_bin_deg = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"yaw_inlier_tol_deg", "yaw_voting.inlier_tolerance_deg"})) config.yaw_inlier_tol_deg = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"yaw_min_tri_inliers", "yaw_voting.min_triangle_inliers"})) config.yaw_min_tri_inliers = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"use_dbh_triangle_match", "dbh_matching.enabled"})) config.use_dbh_triangle_match = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"dbh_diff_tol", "dbh_matching.difference_tolerance_m"})) config.dbh_diff_tol = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"match_distance_tol", "dbh_matching.match_distance_tolerance_m"})) config.match_distance_tol = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"use_vertical", "pose_refinement.enabled"})) config.use_vertical = ParseBool(value, normalized, line_no);
    else if (IsKey(normalized, {"vertical_ransac_iters", "pose_refinement.ransac_iterations"})) config.vertical_ransac_iters = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"vertical_min_sample", "pose_refinement.ransac_min_sample"})) config.vertical_min_sample = ParseNumber<int>(value, normalized, line_no);
    else if (IsKey(normalized, {"z_inlier_tol", "pose_refinement.z_inlier_tolerance_m"})) config.z_inlier_tol = ParseNumber<double>(value, normalized, line_no);
    else if (IsKey(normalized, {"z_inlier_ratio", "pose_refinement.z_inlier_ratio"})) config.z_inlier_ratio = ParseNumber<double>(value, normalized, line_no);
    else throw std::runtime_error("unknown config key " + normalized + " on line " + std::to_string(line_no));
}

}  // namespace

std::filesystem::path DefaultConfigPath(const std::string& mode) {
    return std::filesystem::path("config") / (mode == "inter" ? "inter_v02_v03.yaml" : "full_v02.yaml");
}

bool LoadConfig(const std::filesystem::path& path, Config& config, std::string* error) {
    try {
        std::ifstream in(path);
        if (!in) throw std::runtime_error("could not open config: " + path.string());
        std::string line;
        std::string section;
        size_t line_no = 0;
        while (std::getline(in, line)) {
            ++line_no;
            const std::string without_comment = StripComment(line);
            const std::string trimmed = Trim(without_comment);
            if (trimmed.empty()) continue;
            const size_t colon = trimmed.find(':');
            if (colon == std::string::npos) {
                throw std::runtime_error("expected key: value on line " + std::to_string(line_no));
            }
            const std::string key = Trim(trimmed.substr(0, colon));
            const std::string value = Trim(trimmed.substr(colon + 1));
            const size_t indent = without_comment.find_first_not_of(" \t\r\n");
            const bool indented = indent != std::string::npos && indent > 0;
            if (value.empty()) {
                section = NormalizeKey(key);
                continue;
            }
            const std::string qualified_key =
                indented && !section.empty() ? section + "." + key : key;
            if (!indented) section.clear();
            Assign(qualified_key, value, config, line_no);
        }
        RefreshDerivedConfig(config);
        return ValidateConfig(config, error);
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}

void RefreshDerivedConfig(Config& config) {
    config.spatial_range_bins.clear();
    for (int i = 0; i < config.spatial_bin_count; ++i) {
        const double start0 = config.spatial_bin_min + i * config.spatial_bin_interval;
        const double end0 = start0 + config.spatial_bin_interval;
        const double lo = std::max(config.spatial_bin_min, start0 - config.spatial_bin_padding);
        const double padded_end = (i == 0) ? end0 : end0 + config.spatial_bin_padding;
        const double hi = std::min(config.spatial_bin_max, padded_end);
        if (hi > lo) config.spatial_range_bins.emplace_back(lo, hi);
    }
}

bool ValidateConfig(const Config& config, std::string* error) {
    auto fail = [&](const std::string& msg) {
        if (error) *error = msg;
        return false;
    };
    if (config.max_frames < 0) return fail("max_frames must be non-negative");
    if (config.spatial_threshold <= 0.0) return fail("spatial_threshold must be positive");
    if (config.histogram_k <= 0 || config.rerank_k <= 0) return fail("candidate counts must be positive");
    if (config.knn_k <= 0) return fail("knn_k must be positive");
    if (config.max_dist <= config.min_dist) return fail("max_dist must be greater than min_dist");
    if (config.delta_l <= 0.0) return fail("delta_l must be positive");
    if (config.number_of_cluster <= 0) return fail("number_of_cluster must be positive");
    if (config.local_radius <= 0.0) return fail("local_radius must be positive");
    if (config.total_section <= 0 || config.bin_width <= 0.0) return fail("radius bins are invalid");
    if (config.pairwise_bins <= 0) return fail("pairwise_bins must be positive");
    if (config.pairwise_max_dist <= config.pairwise_min_dist) return fail("pairwise distance range is invalid");
    if (config.spatial_range_bins.empty()) return fail("spatial bins are empty");
    return true;
}

std::vector<RangeBin> BuildRadiusBins(const Config& config) {
    std::vector<RangeBin> bins;
    if (config.total_section == 1) {
        bins.emplace_back(0.0, std::numeric_limits<double>::infinity());
        return bins;
    }
    const double step = (config.max_radius - config.min_radius) / (config.total_section + 1);
    for (int i = 0; i < config.total_section - 1; ++i) {
        const double lo = config.min_radius + i * step;
        bins.emplace_back(lo, lo + config.bin_width);
    }
    bins.emplace_back(bins.back().first + step, std::numeric_limits<double>::infinity());
    return bins;
}

}  // namespace treelocpp
