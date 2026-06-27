#include "treelocpp/eval.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include "treelocpp/descriptors.h"
#include "treelocpp/geometry.h"
#include "treelocpp/io.h"
#include "treelocpp/matching.h"

namespace treelocpp {

namespace {

using PolygonSet = std::vector<std::vector<Eigen::Vector2d>>;

struct LocalizationDetail {
    bool valid = false;
    bool ok_2d = false;
    bool ok_6d = false;
    double txy = 0.0;
    double txyz = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double rot_norm = 0.0;
};

std::string SanitizeLabel(std::string label) {
    if (label.empty()) return "dataset";
    for (char& ch : label) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (!std::isalnum(uch) && ch != '-' && ch != '_') ch = '_';
    }
    return label;
}

Eigen::Matrix4d PredictedRelativeTransform(const FrameData& qf,
                                           const FrameData& cf,
                                           const CandidateResult& best) {
    Eigen::Matrix3d Rz_q2c = Eigen::Matrix3d::Identity();
    Rz_q2c.block<2, 2>(0, 0) = best.transform.R;
    const Eigen::Matrix3d R_rp_q2c =
        Eigen::AngleAxisd(best.vertical.axis_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(best.vertical.axis_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d R_q2c = Rz_q2c * R_rp_q2c;
    const Eigen::Vector3d t_q2c(best.transform.t.x(), best.transform.t.y(), 0.0);

    Eigen::Matrix3d R_c2q = R_q2c.transpose();
    Eigen::Vector3d t_c2q = -R_c2q * t_q2c;
    const Eigen::Matrix3d delta_r_c2q =
        Eigen::AngleAxisd(best.vertical.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(best.vertical.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    R_c2q = delta_r_c2q * R_c2q;
    t_c2q += Eigen::Vector3d(0.0, 0.0, best.vertical.z);

    Eigen::Matrix4d T_pred_aligned = Eigen::Matrix4d::Identity();
    T_pred_aligned.block<3, 3>(0, 0) = R_c2q;
    T_pred_aligned.block<3, 1>(0, 3) = t_c2q;

    return qf.alignment_transform.inverse() *
        T_pred_aligned *
        cf.alignment_transform;
}

std::filesystem::path PoseEdgePath(const Config& config, const std::string& name) {
    return config.pose_edge_output_dir /
        (config.pose_edge_prefix + "_" + SanitizeLabel(name) + ".txt");
}

std::ofstream OpenPoseEdgeFile(const Config& config, const std::string& name) {
    std::ofstream out;
    if (!config.save_pose_edges) return out;
    std::filesystem::create_directories(config.pose_edge_output_dir);
    out.open(PoseEdgePath(config, name));
    if (!out) {
        throw std::runtime_error("could not open pose edge output: " +
                                 PoseEdgePath(config, name).string());
    }
    out << std::fixed << std::setprecision(9);
    return out;
}

void WritePoseEdge(std::ostream& out,
                   const FrameData& qf,
                   const FrameData& cf,
                   const CandidateResult& result) {
    if (!out || !result.transform.ok) return;
    const Eigen::Matrix4d T = PredictedRelativeTransform(qf, cf, result);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    EulerZYX(T.block<3, 3>(0, 0), roll, pitch, yaw);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    out << qf.index << " " << cf.index << " " << result.transform.overlap
        << " " << t.x() << " " << t.y() << " " << t.z()
        << " " << roll << " " << pitch << " " << yaw << "\n";
}

std::vector<std::filesystem::path> QueryRoots(const Config& config) {
    return config.query_roots.empty()
        ? std::vector<std::filesystem::path>{config.query_root}
        : config.query_roots;
}

std::vector<std::filesystem::path> DatabaseRoots(const Config& config) {
    return config.database_roots.empty()
        ? std::vector<std::filesystem::path>{config.database_root}
        : config.database_roots;
}

std::string RootLabel(const std::filesystem::path& root,
                      const std::vector<std::string>& labels,
                      size_t index) {
    if (index < labels.size() && !labels[index].empty()) return labels[index];
    return DatasetName(root);
}

std::string JoinLines(const std::vector<std::string>& lines) {
    std::ostringstream out;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (i > 0) out << "\n";
        out << lines[i];
    }
    return out.str();
}

void ThrowIfSchemaErrors(const std::vector<std::string>& lines) {
    if (!lines.empty()) throw std::runtime_error(JoinLines(lines));
}

std::string LowerAscii(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

bool HasSequenceMarker(const std::string& component, char marker) {
    for (size_t pos = component.find(marker); pos != std::string::npos;
         pos = component.find(marker, pos + 1)) {
        const bool left_ok = pos == 0 || component[pos - 1] == '_' ||
            component[pos - 1] == '-' || component[pos - 1] == '/';
        if (!left_ok || pos + 1 >= component.size()) continue;
        const char next = component[pos + 1];
        if (next == '-' || next == '_' || std::isdigit(static_cast<unsigned char>(next))) return true;
    }
    return false;
}

bool LooksLikeWildV(const std::filesystem::path& root) {
    for (const auto& part : root) {
        const std::string component = LowerAscii(part.string());
        if (component == "wild_v" || component == "wild-v" ||
            component == "wildv" || component == "venman") {
            return true;
        }
        if (HasSequenceMarker(component, 'v')) return true;
    }
    return false;
}

bool LooksLikeWildK(const std::filesystem::path& root) {
    for (const auto& part : root) {
        const std::string component = LowerAscii(part.string());
        if (component == "wild_k" || component == "wild-k" ||
            component == "wildk" || component == "karawatha") {
            return true;
        }
        if (HasSequenceMarker(component, 'k')) return true;
    }
    return false;
}

const PolygonSet& WildVTestPolygons() {
    static const PolygonSet polygons = {
        {{-468.0, -82.0}, {-468.0, 44.0}, {-314.0, 44.0},
         {-305.0, 12.0}, {-192.0, 44.0}, {-192.0, -82.0}},
        {{-78.0, -171.0}, {-78.0, -215.0}, {-305.0, -215.0}, {-305.0, -171.0}},
        {{-62.0, 70.0}, {95.0, 70.0}, {142.0, 0.0}, {140.0, -142.0}, {-62.0, -142.0}},
    };
    return polygons;
}

const PolygonSet& WildKTestPolygons() {
    static const PolygonSet polygons = {
        {{-150.0, 8.0}, {300.0, 8.0}, {300.0, -210.0}, {-150.0, -210.0}},
        {{-215.0, 618.0}, {-74.0, 618.0}, {-74.0, 423.0}, {-215.0, 423.0}},
        {{-513.0, 300.0}, {-513.0, 37.0}, {-321.0, 37.0}, {-321.0, 300.0}},
    };
    return polygons;
}

const PolygonSet& SelectTestPolygons(const Config& config, const std::filesystem::path& root) {
    const std::string family = LowerAscii(config.test_polygon_family);
    if (family == "v" || family == "wild_v" || family == "wild-v" || family == "venman") {
        return WildVTestPolygons();
    }
    if (family == "k" || family == "wild_k" || family == "wild-k" || family == "karawatha") {
        return WildKTestPolygons();
    }
    if (family != "auto" && !family.empty()) {
        throw std::runtime_error("unknown test_polygon_family: " + config.test_polygon_family);
    }

    const bool is_v = LooksLikeWildV(root);
    const bool is_k = LooksLikeWildK(root);
    if (is_v && !is_k) return WildVTestPolygons();
    if (is_k && !is_v) return WildKTestPolygons();
    if (is_v && is_k) {
        throw std::runtime_error("ambiguous test polygon family for query_root: " + root.string());
    }
    throw std::runtime_error("could not infer test polygon family from query_root: " + root.string() +
                             " (set test_polygon_family: wild_v or wild_k)");
}

bool PointInPolygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& poly) {
    bool inside = false;
    for (int i = 0, j = static_cast<int>(poly.size()) - 1; i < static_cast<int>(poly.size()); j = i++) {
        const auto& pi = poly[i];
        const auto& pj = poly[j];
        const bool crosses = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
            (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y() + 1e-12) + pi.x());
        if (crosses) inside = !inside;
    }
    return inside;
}

bool InTestPolygons(const Pose& pose, const PolygonSet& polygons) {
    const Eigen::Vector2d p(pose.x, pose.y);
    for (const auto& poly : polygons) {
        if (PointInPolygon(p, poly)) return true;
    }
    return false;
}

std::vector<size_t> IntraCandidates(const Dataset& dataset, size_t query_slot, const Config& config) {
    std::vector<size_t> out;
    const int qi = dataset.frames[query_slot].index;
    for (size_t i = 0; i < dataset.frames.size(); ++i) {
        const int ci = dataset.frames[i].index;
        if (ci >= qi) continue;
        if (std::abs(ci - qi) <= config.temporal_min_separation) continue;
        out.push_back(i);
    }
    return out;
}

std::unordered_set<int> GroundTruth(const Dataset& qset,
                                    const Dataset& dset,
                                    size_t query_slot,
                                    const std::vector<size_t>& candidate_slots,
                                    const Config& config) {
    std::unordered_set<int> gt;
    const Pose& qp = qset.frames[query_slot].pose;
    for (size_t slot : candidate_slots) {
        const FrameData& cf = dset.frames[slot];
        if (PoseDistanceXY(qp, cf.pose) <= config.spatial_threshold) gt.insert(cf.index);
    }
    return gt;
}

void Accumulate(const CandidateResult& best, EvaluationSummary& summary) {
    summary.mean_spatial_error += best.spatial_error;
    summary.mean_overlap += best.transform.overlap;
    summary.mean_z += std::abs(best.vertical.z);
    summary.mean_roll_deg += std::abs(best.vertical.roll) * 180.0 / M_PI;
    summary.mean_pitch_deg += std::abs(best.vertical.pitch) * 180.0 / M_PI;
}

LocalizationDetail AccumulateLocalization(const FrameData& qf,
                                          const FrameData& cf,
                                          const CandidateResult& best,
                                          bool is_true_neighbor,
                                          EvaluationSummary& summary) {
    LocalizationDetail detail;
    if (!best.transform.ok) return detail;

    const Eigen::Matrix4d Tq = PoseToTransform(qf.pose);
    const Eigen::Matrix4d Tc = PoseToTransform(cf.pose);
    const Eigen::Matrix4d T_rel_gt = Tq.inverse() * Tc;
    const Eigen::Vector3d pos_rel_gt = T_rel_gt.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_rel_gt = T_rel_gt.block<3, 3>(0, 0);
    const Eigen::Matrix4d T_rel_pred = PredictedRelativeTransform(qf, cf, best);
    const Eigen::Vector3d pos_rel_pred = T_rel_pred.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_rel_pred = T_rel_pred.block<3, 3>(0, 0);

    double gt_roll = 0.0;
    double gt_pitch = 0.0;
    double gt_yaw = 0.0;
    double pred_roll = 0.0;
    double pred_pitch = 0.0;
    double pred_yaw = 0.0;
    EulerZYX(R_rel_gt, gt_roll, gt_pitch, gt_yaw);
    EulerZYX(R_rel_pred, pred_roll, pred_pitch, pred_yaw);

    const double txy_err = (pos_rel_gt.head<2>() - pos_rel_pred.head<2>()).norm();
    const double txyz_err = (pos_rel_gt - pos_rel_pred).norm();
    const double z_err = std::abs(pos_rel_gt.z() - pos_rel_pred.z());
    const double roll_err = std::abs(WrapAngle(gt_roll - pred_roll));
    const double pitch_err = std::abs(WrapAngle(gt_pitch - pred_pitch));
    const double yaw_err = std::abs(WrapAngle(gt_yaw - pred_yaw));
    const double rot_norm = std::sqrt(roll_err * roll_err + pitch_err * pitch_err + yaw_err * yaw_err);

    const bool ok_2d = txy_err <= 0.5 && yaw_err * 180.0 / M_PI <= 5.0;
    const bool ok_6d = txyz_err <= 0.5 && rot_norm * 180.0 / M_PI <= 5.0;

    detail.valid = true;
    detail.ok_2d = ok_2d;
    detail.ok_6d = ok_6d;
    detail.txy = txy_err;
    detail.txyz = txyz_err;
    detail.z = z_err;
    detail.roll = roll_err;
    detail.pitch = pitch_err;
    detail.yaw = yaw_err;
    detail.rot_norm = rot_norm;

    if (is_true_neighbor && ok_2d) {
        ++summary.localization_2d_recall_success;
        ++summary.localization_2d_true_success;
        summary.localization_txy_sum += txy_err;
        summary.localization_yaw_sum += yaw_err;
    }
    if (is_true_neighbor && ok_6d) {
        ++summary.localization_6d_recall_success;
        ++summary.localization_6d_true_success;
        summary.localization_txyz_sum += txyz_err;
        summary.localization_z_sum += z_err;
        summary.localization_roll_sum += roll_err;
        summary.localization_pitch_sum += pitch_err;
        summary.localization_yaw6_sum += yaw_err;
        summary.localization_rot_norm_sum += rot_norm;
    }
    return detail;
}

void Finalize(EvaluationSummary& summary) {
    summary.recall = summary.evaluated > 0
        ? static_cast<double>(summary.true_positive) / summary.evaluated
        : 0.0;
    if (summary.evaluated > 0) {
        summary.mean_spatial_error /= summary.evaluated;
        summary.mean_overlap /= summary.evaluated;
        summary.mean_z /= summary.evaluated;
        summary.mean_roll_deg /= summary.evaluated;
        summary.mean_pitch_deg /= summary.evaluated;
        summary.mean_gt_top_histogram /= summary.evaluated;
    }
}

int HashIntersectionEval(const FrameData& qf, const FrameData& cf) {
    int score = 0;
    for (const auto& kv : qf.hash_counts) {
        auto it = cf.hash_counts.find(kv.first);
        if (it != cf.hash_counts.end()) score += std::min(kv.second, it->second);
    }
    return score;
}

std::pair<int, int> TopCandidateGtCounts(const Dataset& qset,
                                         const Dataset& dset,
                                         size_t query_slot,
                                         const std::vector<size_t>& candidate_slots,
                                         const std::unordered_set<int>& gt,
                                         const Config& config) {
    struct Score {
        size_t slot;
        double tdh;
        double pw;
        double combined;
        int hash = 0;
    };
    const FrameData& qf = qset.frames[query_slot];
    std::vector<Score> scores;
    scores.reserve(candidate_slots.size());
    for (size_t slot : candidate_slots) {
        const FrameData& cf = dset.frames[slot];
        scores.push_back({slot, ChiSquared(qf.tdh, cf.tdh), ChiSquared(qf.pairwise, cf.pairwise), 0.0, 0});
    }
    if (scores.empty()) return {0, 0};
    double tdh_min = std::numeric_limits<double>::infinity();
    double tdh_max = -std::numeric_limits<double>::infinity();
    double pw_min = std::numeric_limits<double>::infinity();
    double pw_max = -std::numeric_limits<double>::infinity();
    for (const auto& s : scores) {
        tdh_min = std::min(tdh_min, s.tdh);
        tdh_max = std::max(tdh_max, s.tdh);
        pw_min = std::min(pw_min, s.pw);
        pw_max = std::max(pw_max, s.pw);
    }
    const double tdh_den = std::max(tdh_max - tdh_min, 1e-12);
    const double pw_den = std::max(pw_max - pw_min, 1e-12);
    for (auto& s : scores) {
        const double tdh = (s.tdh - tdh_min) / tdh_den;
        const double pw = (s.pw - pw_min) / pw_den;
        s.combined = (1.0 - config.pairwise_weight) * tdh + config.pairwise_weight * pw;
    }
    const int top_hist = std::min(config.histogram_k, static_cast<int>(scores.size()));
    auto by_combined = [](const Score& a, const Score& b) {
        return a.combined < b.combined;
    };
    if (top_hist < static_cast<int>(scores.size())) {
        std::nth_element(scores.begin(), scores.begin() + top_hist, scores.end(), by_combined);
        scores.resize(top_hist);
    }
    std::sort(scores.begin(), scores.end(), by_combined);
    int gt_hist = 0;
    for (auto& s : scores) {
        if (gt.count(dset.frames[s.slot].index)) ++gt_hist;
        s.hash = HashIntersectionEval(qf, dset.frames[s.slot]);
    }
    std::sort(scores.begin(), scores.end(), [](const Score& a, const Score& b) {
        return a.hash > b.hash;
    });
    const int top_hash = std::min(config.rerank_k, static_cast<int>(scores.size()));
    int gt_hash = 0;
    for (int i = 0; i < top_hash; ++i) {
        if (gt.count(dset.frames[scores[i].slot].index)) ++gt_hash;
    }
    return {gt_hist, gt_hash};
}

void RunInterPair(const Config& config,
                  const std::filesystem::path& query_root,
                  const std::filesystem::path& database_root,
                  const std::string& query_label,
                  const std::string& database_label,
                  EvaluationSummary& summary) {
    std::vector<std::string> schema_errors =
        TreeCsvSchemaErrors(query_root, config.max_frames, "query_root");
    const std::vector<std::string> database_errors =
        TreeCsvSchemaErrors(database_root, config.max_frames, "database_root");
    schema_errors.insert(schema_errors.end(), database_errors.begin(), database_errors.end());
    ThrowIfSchemaErrors(schema_errors);

    Dataset queries = LoadDataset(query_root, config, config.neighbor_past_only, config.query_yaw_deg);
    Dataset database = LoadDataset(database_root, config, config.neighbor_past_only, config.database_yaw_deg);
    const PolygonSet* test_polygons = nullptr;
    if (config.use_test_polygons) test_polygons = &SelectTestPolygons(config, query_root);
    summary.queries += static_cast<int>(queries.frames.size());

    std::ofstream edge_out = OpenPoseEdgeFile(
        config, SanitizeLabel(database_label) + "_vs_" + SanitizeLabel(query_label));

    std::ofstream debug;
    if (const char* path = std::getenv("TREELOCPP_DEBUG_CSV")) {
        debug.open(path, std::ios::app);
        if (debug && debug.tellp() == 0) {
            debug << "mode,query,candidate,true_neighbor,spatial_error,overlap,"
                  << "txy,txyz,z_err,roll_err_deg,pitch_err_deg,yaw_err_deg,rot_norm_deg,"
                  << "ok_2d,ok_6d,z_offset,plane_roll_deg,plane_pitch_deg,"
                  << "axis_roll_deg,axis_pitch_deg,match_count\n";
        }
    }
    std::vector<size_t> all_db(database.frames.size());
    std::iota(all_db.begin(), all_db.end(), 0);
    for (size_t q = 0; q < queries.frames.size(); ++q) {
        if (test_polygons && !InTestPolygons(queries.frames[q].pose, *test_polygons)) continue;
        const auto gt = GroundTruth(queries, database, q, all_db, config);
        if (gt.empty()) continue;
        auto ranked = RankCandidates(queries, database, q, all_db, config);
        if (ranked.empty()) continue;
        for (const auto& result : ranked) {
            const FrameData& ranked_cf =
                database.frames.at(database.frame_to_slot.at(result.candidate_index));
            WritePoseEdge(edge_out, queries.frames[q], ranked_cf, result);
        }
        const auto [gt_hist, gt_hash] = TopCandidateGtCounts(queries, database, q, all_db, gt, config);
        CandidateResult best = ranked.front();
        best.true_neighbor = gt.count(best.candidate_index) > 0;
        const FrameData& cf = database.frames.at(database.frame_to_slot.at(best.candidate_index));
        ++summary.evaluated;
        if (gt_hist == 0) ++summary.zero_gt_top_histogram;
        if (gt_hash == 0) ++summary.zero_gt_top_hash;
        summary.mean_gt_top_histogram += gt_hist;
        if (best.true_neighbor) ++summary.true_positive;
        Accumulate(best, summary);
        const LocalizationDetail detail =
            AccumulateLocalization(queries.frames[q], cf, best, best.true_neighbor, summary);
        if (debug && detail.valid) {
            debug << "inter," << queries.frames[q].index << "," << best.candidate_index << ","
                  << (best.true_neighbor ? 1 : 0) << "," << best.spatial_error << ","
                  << best.transform.overlap << "," << detail.txy << "," << detail.txyz << ","
                  << detail.z << "," << detail.roll * 180.0 / M_PI << ","
                  << detail.pitch * 180.0 / M_PI << "," << detail.yaw * 180.0 / M_PI << ","
                  << detail.rot_norm * 180.0 / M_PI << "," << (detail.ok_2d ? 1 : 0) << ","
                  << (detail.ok_6d ? 1 : 0) << "," << best.vertical.z << ","
                  << best.vertical.roll * 180.0 / M_PI << ","
                  << best.vertical.pitch * 180.0 / M_PI << ","
                  << best.vertical.axis_roll * 180.0 / M_PI << ","
                  << best.vertical.axis_pitch * 180.0 / M_PI << ","
                  << best.transform.pairs.size() << "\n";
        }
    }
}

}  // namespace

EvaluationSummary RunIntraSession(const Config& config) {
    ThrowIfSchemaErrors(TreeCsvSchemaErrors(config.dataset_root, config.max_frames, "dataset_root"));
    Dataset dataset = LoadDataset(config.dataset_root, config, config.neighbor_past_only, config.dataset_yaw_deg);
    EvaluationSummary summary;
    summary.queries = static_cast<int>(dataset.frames.size());
    std::ofstream debug;
    if (const char* path = std::getenv("TREELOCPP_DEBUG_CSV")) {
        debug.open(path);
        if (debug) {
            debug << "mode,query,candidate,true_neighbor,spatial_error,overlap,"
                  << "txy,txyz,z_err,roll_err_deg,pitch_err_deg,yaw_err_deg,rot_norm_deg,"
                  << "ok_2d,ok_6d,z_offset,plane_roll_deg,plane_pitch_deg,"
                  << "axis_roll_deg,axis_pitch_deg,match_count\n";
        }
    }
    std::ofstream edge_out = OpenPoseEdgeFile(config, DatasetName(config.dataset_root));
    for (size_t q = 0; q < dataset.frames.size(); ++q) {
        const auto candidates = IntraCandidates(dataset, q, config);
        if (candidates.empty()) continue;
        const auto gt = GroundTruth(dataset, dataset, q, candidates, config);
        if (gt.empty()) continue;
        auto ranked = RankCandidates(dataset, dataset, q, candidates, config);
        if (ranked.empty()) continue;
        for (const auto& result : ranked) {
            const FrameData& ranked_cf =
                dataset.frames.at(dataset.frame_to_slot.at(result.candidate_index));
            WritePoseEdge(edge_out, dataset.frames[q], ranked_cf, result);
        }
        CandidateResult best = ranked.front();
        best.true_neighbor = gt.count(best.candidate_index) > 0;
        const FrameData& cf = dataset.frames.at(dataset.frame_to_slot.at(best.candidate_index));
        ++summary.evaluated;
        if (best.true_neighbor) ++summary.true_positive;
        Accumulate(best, summary);
        const LocalizationDetail detail =
            AccumulateLocalization(dataset.frames[q], cf, best, best.true_neighbor, summary);
        if (debug && detail.valid) {
            debug << "intra," << dataset.frames[q].index << "," << best.candidate_index << ","
                  << (best.true_neighbor ? 1 : 0) << "," << best.spatial_error << ","
                  << best.transform.overlap << "," << detail.txy << "," << detail.txyz << ","
                  << detail.z << "," << detail.roll * 180.0 / M_PI << ","
                  << detail.pitch * 180.0 / M_PI << "," << detail.yaw * 180.0 / M_PI << ","
                  << detail.rot_norm * 180.0 / M_PI << "," << (detail.ok_2d ? 1 : 0) << ","
                  << (detail.ok_6d ? 1 : 0) << "," << best.vertical.z << ","
                  << best.vertical.roll * 180.0 / M_PI << ","
                  << best.vertical.pitch * 180.0 / M_PI << ","
                  << best.vertical.axis_roll * 180.0 / M_PI << ","
                  << best.vertical.axis_pitch * 180.0 / M_PI << ","
                  << best.transform.pairs.size() << "\n";
        }
    }
    Finalize(summary);
    return summary;
}

EvaluationSummary RunInterSession(const Config& config) {
    EvaluationSummary summary;
    const auto query_roots = QueryRoots(config);
    const auto database_roots = DatabaseRoots(config);
    for (size_t qi = 0; qi < query_roots.size(); ++qi) {
        const std::string q_label = RootLabel(query_roots[qi], config.query_labels, qi);
        for (size_t di = 0; di < database_roots.size(); ++di) {
            const std::string d_label = RootLabel(database_roots[di], config.database_labels, di);
            RunInterPair(config, query_roots[qi], database_roots[di], q_label, d_label, summary);
        }
    }
    Finalize(summary);
    return summary;
}

void PrintSummary(const EvaluationSummary& summary, std::ostream& out) {
    out << std::fixed << std::setprecision(4);
    out << "Queries: " << summary.queries << "\n";
    out << "Evaluated queries with GT: " << summary.evaluated << "\n";
    out << "True positives: " << summary.true_positive << "\n";
    out << "Recall@1: " << summary.recall << "\n";
    out << "Mean best spatial error: " << summary.mean_spatial_error << " m\n";
    out << "Mean best overlap: " << summary.mean_overlap << "\n";
    out << "Mean |z offset|: " << summary.mean_z << " m\n";
    out << "Mean |roll|: " << summary.mean_roll_deg << " deg\n";
    out << "Mean |pitch|: " << summary.mean_pitch_deg << " deg\n";
    out << "Localization Success@true-pairs 2D: "
        << summary.localization_2d_true_success << "/" << summary.true_positive
        << " (" << (summary.true_positive > 0
            ? 100.0 * summary.localization_2d_true_success / summary.true_positive
            : 0.0) << "%)\n";
    out << "Mean localization |t_xy| (succ only): "
        << (summary.localization_2d_true_success > 0
            ? summary.localization_txy_sum / summary.localization_2d_true_success
            : 0.0) << " m\n";
    out << "Mean localization |yaw_err| (succ only): "
        << (summary.localization_2d_true_success > 0
            ? summary.localization_yaw_sum / summary.localization_2d_true_success * 180.0 / M_PI
            : 0.0) << " deg\n";
    out << "Localization Success@true-pairs 6DoF: "
        << summary.localization_6d_true_success << "/" << summary.true_positive
        << " (" << (summary.true_positive > 0
            ? 100.0 * summary.localization_6d_true_success / summary.true_positive
            : 0.0) << "%)\n";
    out << "Mean localization |t_xyz| (succ only): "
        << (summary.localization_6d_true_success > 0
            ? summary.localization_txyz_sum / summary.localization_6d_true_success
            : 0.0) << " m\n";
    out << "Mean localization |z| (succ only): "
        << (summary.localization_6d_true_success > 0
            ? summary.localization_z_sum / summary.localization_6d_true_success
            : 0.0) << " m\n";
    out << "Mean localization angles (succ only): roll="
        << (summary.localization_6d_true_success > 0
            ? summary.localization_roll_sum / summary.localization_6d_true_success * 180.0 / M_PI
            : 0.0)
        << " deg, pitch="
        << (summary.localization_6d_true_success > 0
            ? summary.localization_pitch_sum / summary.localization_6d_true_success * 180.0 / M_PI
            : 0.0)
        << " deg, yaw="
        << (summary.localization_6d_true_success > 0
            ? summary.localization_yaw6_sum / summary.localization_6d_true_success * 180.0 / M_PI
            : 0.0)
        << " deg, norm="
        << (summary.localization_6d_true_success > 0
            ? summary.localization_rot_norm_sum / summary.localization_6d_true_success * 180.0 / M_PI
            : 0.0) << " deg\n";
    if (summary.evaluated > 0 && (summary.zero_gt_top_histogram > 0 || summary.mean_gt_top_histogram > 0.0)) {
        out << "0 GT in Hybrid Top100: " << summary.zero_gt_top_histogram << "/" << summary.evaluated << "\n";
        out << "0 GT in Hash Top10: " << summary.zero_gt_top_hash << "/" << summary.evaluated << "\n";
        out << "Average GT in Hybrid Top100: " << summary.mean_gt_top_histogram << "\n";
    }
}

}  // namespace treelocpp
