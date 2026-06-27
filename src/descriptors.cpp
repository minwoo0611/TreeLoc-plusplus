#include "treelocpp/descriptors.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <unordered_set>

#include "treelocpp/geometry.h"
#include "treelocpp/io.h"

namespace treelocpp {

namespace {

double Radius(const Tree& tree) {
    return std::isfinite(tree.dbh) ? tree.dbh : tree.dbh_approximation;
}

bool InLocalBox(const Tree& tree, const Config& config) {
    return std::abs(tree.x) <= config.local_radius && std::abs(tree.y) <= config.local_radius;
}

std::vector<int> NeighborFrames(int idx,
                                const std::vector<Pose>& trajectory,
                                const Config& config,
                                bool past_only) {
    std::vector<std::pair<double, int>> scored;
    if (idx < 0 || idx >= static_cast<int>(trajectory.size())) return {};
    for (int j = 0; j < static_cast<int>(trajectory.size()); ++j) {
        if (j == idx) continue;
        if (past_only && j >= idx) continue;
        const double d = PoseDistanceXY(trajectory[idx], trajectory[j]);
        if (d <= config.neighbor_radius) scored.emplace_back(d, j);
    }
    std::sort(scored.begin(), scored.end());
    std::vector<int> out;
    for (const auto& item : scored) {
        if (static_cast<int>(out.size()) >= config.neighbor_max_scenes) break;
        out.push_back(item.second);
    }
    return out;
}

std::vector<Tree> Deduplicate(const std::vector<Tree>& trees, double tol) {
    if (tol <= 0.0) return trees;
    const double tol2 = tol * tol;
    std::vector<Tree> out;
    std::vector<char> used(trees.size(), 0);
    for (size_t i = 0; i < trees.size(); ++i) {
        if (used[i]) continue;
        out.push_back(trees[i]);
        for (size_t j = i + 1; j < trees.size(); ++j) {
            const double dx = trees[i].x - trees[j].x;
            const double dy = trees[i].y - trees[j].y;
            if (dx * dx + dy * dy <= tol2) used[j] = 1;
        }
    }
    return out;
}

void ApplyTransform(std::vector<Tree>& trees, const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    for (auto& tree : trees) {
        const Eigen::Vector4d p(tree.x, tree.y, tree.z, 1.0);
        const Eigen::Vector4d pt = T * p;
        tree.x = pt.x();
        tree.y = pt.y();
        tree.z = pt.z();
        if (tree.has_axis) tree.axis = R * tree.axis;
    }
}

Eigen::Matrix4d AxisAlignment(const std::vector<Tree>& trees) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int count = 0;
    for (const auto& tree : trees) {
        if (!tree.has_axis) continue;
        Eigen::Vector3d axis(tree.axis(0, 2), tree.axis(1, 2), tree.axis(2, 2));
        if (!axis.allFinite() || axis.norm() < 1e-6) continue;
        axis.normalize();
        mean += axis;
        ++count;
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (count == 0 || mean.norm() < 1e-6) return T;
    mean.normalize();
    if (mean.z() < 0.0) mean = -mean;
    const Eigen::Vector3d target(0.0, 0.0, 1.0);
    const double c = std::clamp(mean.dot(target), -1.0, 1.0);
    if (std::abs(c - 1.0) < 1e-6) return T;
    if (std::abs(c + 1.0) < 1e-6) {
        T(0, 0) = -1.0;
        T(2, 2) = -1.0;
        return T;
    }
    const Eigen::Vector3d v = mean.cross(target);
    const double s = v.norm();
    Eigen::Matrix3d vx;
    vx << 0.0, -v.z(), v.y(),
          v.z(), 0.0, -v.x(),
          -v.y(), v.x(), 0.0;
    T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() + vx + vx * vx * ((1.0 - c) / (s * s));
    return T;
}

Eigen::Matrix4d ApplyFrameAlignment(std::vector<Tree>& trees,
                                    const std::vector<Tree>& current,
                                    const Config& config) {
    Eigen::Matrix4d total = Eigen::Matrix4d::Identity();
    std::vector<Tree> align_rows;
    align_rows.reserve(current.size());
    for (const auto& tree : current) {
        if (tree.number_clusters > 2) align_rows.push_back(tree);
    }
    if (config.tree_axis_alignment_enabled && align_rows.size() >= 3) {
        Eigen::Matrix4d T = AxisAlignment(align_rows);
        ApplyTransform(trees, T);
        total = T * total;
    }
    return total;
}

FrameData BuildFrame(const std::filesystem::path& root,
                     int idx,
                     const std::vector<Pose>& trajectory,
                     const Config& config,
                     bool past_only) {
    FrameData frame;
    frame.index = idx;
    if (idx >= 0 && idx < static_cast<int>(trajectory.size())) {
        frame.pose = trajectory[idx];
    }

    std::vector<Tree> raw = ReadTreeCsv(root / ("TreeManagerState_" + std::to_string(idx) + ".csv"));
    std::vector<Tree> current;
    current.reserve(raw.size());
    for (const auto& tree : raw) {
        if (InLocalBox(tree, config)) current.push_back(tree);
    }

    std::vector<Tree> reconstructed;
    std::vector<Tree> supplemental;
    for (const auto& tree : current) {
        if (tree.reconstructed == 1) reconstructed.push_back(tree);
        else if (tree.score > config.tree_score_min) supplemental.push_back(tree);
    }

    if (config.neighbor_augment &&
        static_cast<int>(reconstructed.size()) < config.min_reconstructed_per_frame) {
        for (int neighbor : NeighborFrames(idx, trajectory, config, past_only)) {
            if (!HasFrameCsv(root, neighbor)) continue;
            std::vector<Tree> nb = ReadTreeCsv(root / ("TreeManagerState_" + std::to_string(neighbor) + ".csv"));
            std::vector<Eigen::Vector2d> nb_pts;
            std::vector<Tree> nb_rec;
            for (const auto& tree : nb) {
                if (tree.reconstructed != 1 || !InLocalBox(tree, config)) continue;
                nb_rec.push_back(tree);
                nb_pts.emplace_back(tree.x, tree.y);
            }
            if (nb_rec.empty() || neighbor >= static_cast<int>(trajectory.size())) continue;
            Eigen::MatrixXd moved = LocalToLocal2D(nb_pts, trajectory[neighbor], trajectory[idx]);
            for (int i = 0; i < static_cast<int>(nb_rec.size()); ++i) {
                Tree t = nb_rec[i];
                t.x = moved(i, 0);
                t.y = moved(i, 1);
                reconstructed.push_back(t);
            }
            if (static_cast<int>(reconstructed.size()) >= config.min_reconstructed_per_frame) break;
        }
        reconstructed = Deduplicate(reconstructed, config.dedup_distance);
    }

    frame.trees = reconstructed;
    const int need = config.number_of_cluster - static_cast<int>(frame.trees.size());
    if (need > 0) {
        std::sort(supplemental.begin(), supplemental.end(),
                  [](const Tree& a, const Tree& b) { return a.score > b.score; });
        for (int i = 0; i < need && i < static_cast<int>(supplemental.size()); ++i) {
            frame.trees.push_back(supplemental[i]);
        }
    }

    frame.alignment_transform = ApplyFrameAlignment(frame.trees, current, config);

    frame.centers.reserve(frame.trees.size());
    for (const auto& tree : frame.trees) frame.centers.emplace_back(tree.x, tree.y);

    const std::vector<RangeBin> radius_bins = BuildRadiusBins(config);
    std::vector<Tree> rec_only;
    for (const auto& tree : frame.trees) {
        if (tree.reconstructed == 1) rec_only.push_back(tree);
    }
    const std::vector<Tree>& tdh_trees = config.tdh_use_rec_only ? rec_only : frame.trees;
    const std::vector<Tree>& pdh_trees = config.pdh_use_rec_only ? rec_only : frame.trees;

    frame.tdh = ComputeTDH(tdh_trees, config.spatial_range_bins, radius_bins);
    frame.pdh = ComputePDH(pdh_trees, config);
    frame.triangles = ComputeKnnTriangles(frame.centers, config.knn_k, config.min_dist, config.max_dist);
    frame.hashes = TriangleHashes(frame.triangles, frame.centers,
                                  config.delta_l, config.rho, config.hash_modulus);
    for (const auto& item : frame.hashes) ++frame.hash_counts[item.first];
    return frame;
}

long long TriangleHash(double a, double b, double c, double delta_l, long long rho, long long mod) {
    const long long e1 = static_cast<long long>(std::llround(a / delta_l));
    const long long e2 = static_cast<long long>(std::llround(b / delta_l));
    const long long e3 = static_cast<long long>(std::llround(c / delta_l));
    long long h = (e3 * rho + e2) % mod;
    h = (h * rho + e1) % mod;
    return h;
}

std::array<int, 3> OrderedTriangle(const std::vector<Eigen::Vector2d>& centers,
                                   int i,
                                   int j,
                                   int k) {
    std::array<int, 3> tri{i, j, k};
    std::sort(tri.begin(), tri.end());
    (void)centers;
    return tri;
}

}  // namespace

Dataset LoadDataset(const std::filesystem::path& root,
                    const Config& config,
                    bool past_only) {
    Dataset dataset;
    dataset.root = root;
    dataset.trajectory = ReadTrajectory(root / "trajectory.txt");
    const auto indices = DiscoverFrameIndices(root, config.max_frames);
    dataset.frames.reserve(indices.size());
    for (int idx : indices) {
        if (idx >= static_cast<int>(dataset.trajectory.size())) continue;
        FrameData frame = BuildFrame(root, idx, dataset.trajectory, config, past_only);
        if (frame.trees.size() >= 3 && !frame.hashes.empty()) {
            dataset.frame_to_slot[frame.index] = dataset.frames.size();
            dataset.frames.push_back(std::move(frame));
        }
    }
    return dataset;
}

std::vector<Tree> SelectTrees(const std::vector<Tree>& trees, const Config& config) {
    std::vector<Tree> selected;
    std::vector<Tree> supplemental;
    for (const auto& tree : trees) {
        if (tree.reconstructed == 1) selected.push_back(tree);
        else if (tree.score > config.tree_score_min) supplemental.push_back(tree);
    }
    if (static_cast<int>(selected.size()) < config.number_of_cluster) {
        std::sort(supplemental.begin(), supplemental.end(),
                  [](const Tree& a, const Tree& b) { return a.score > b.score; });
        const int need = config.number_of_cluster - static_cast<int>(selected.size());
        for (int i = 0; i < need && i < static_cast<int>(supplemental.size()); ++i) {
            selected.push_back(supplemental[i]);
        }
    }
    return selected;
}

Eigen::MatrixXd ComputeTDH(const std::vector<Tree>& trees,
                           const std::vector<RangeBin>& spatial_bins,
                           const std::vector<RangeBin>& radius_bins) {
    Eigen::MatrixXd hist = Eigen::MatrixXd::Zero(spatial_bins.size(), radius_bins.size());
    for (const auto& tree : trees) {
        const double d = std::hypot(tree.x, tree.y);
        int sbin = -1;
        for (int i = 0; i < static_cast<int>(spatial_bins.size()); ++i) {
            if (spatial_bins[i].first < d && d <= spatial_bins[i].second) {
                sbin = i;
                break;
            }
        }
        if (sbin < 0) continue;
        const double r = Radius(tree);
        for (int j = 0; j < static_cast<int>(radius_bins.size()); ++j) {
            if ((radius_bins[j].first < r && r <= radius_bins[j].second) ||
                (j == static_cast<int>(radius_bins.size()) - 1 && r > radius_bins[j].first)) {
                hist(sbin, j) += 1.0;
                break;
            }
        }
    }

    Eigen::MatrixXd smoothed = Eigen::MatrixXd::Zero(hist.rows(), hist.cols());
    for (int i = 0; i < hist.rows(); ++i) {
        for (int j = 0; j < hist.cols(); ++j) {
            for (int di = 0; di < 2; ++di) {
                for (int dj = 0; dj < 2; ++dj) {
                    const int ni = i + di;
                    const int nj = j + dj;
                    if (ni < hist.rows() && nj < hist.cols()) smoothed(i, j) += hist(ni, nj);
                }
            }
        }
    }
    return smoothed;
}

Eigen::VectorXd ComputePDH(const std::vector<Tree>& trees, const Config& config) {
    Eigen::VectorXd hist = Eigen::VectorXd::Zero(config.pdh_bins);
    if (trees.size() < 2) return hist;
    const int n = static_cast<int>(trees.size());
    const double range = config.pdh_max_dist - config.pdh_min_dist;
    std::vector<double> distances;
    const int64_t total_pairs = static_cast<int64_t>(n) * (n - 1) / 2;
    if (config.pdh_max_pairs > 0 && total_pairs > config.pdh_max_pairs) {
        std::mt19937_64 rng(0);
        std::uniform_int_distribution<int> dist_i(0, n - 2);
        distances.reserve(config.pdh_max_pairs);
        for (int sample = 0; sample < config.pdh_max_pairs; ++sample) {
            const int i = dist_i(rng);
            std::uniform_int_distribution<int> dist_j(i + 1, n - 1);
            const int j = dist_j(rng);
            const double d = std::hypot(trees[i].x - trees[j].x, trees[i].y - trees[j].y);
            if (d >= config.pdh_min_dist && d <= config.pdh_max_dist) distances.push_back(d);
        }
    } else {
        distances.reserve(static_cast<size_t>(std::min<int64_t>(total_pairs, 10000)));
        for (int i = 0; i + 1 < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const double d = std::hypot(trees[i].x - trees[j].x, trees[i].y - trees[j].y);
                if (d >= config.pdh_min_dist && d <= config.pdh_max_dist) distances.push_back(d);
            }
        }
    }
    if (!config.pdh_soft_binning) {
        for (double d : distances) {
            if (d < config.pdh_min_dist || d > config.pdh_max_dist) continue;
            int b = static_cast<int>(std::floor((d - config.pdh_min_dist) * config.pdh_bins / range));
            if (b < 0) b = 0;
            if (b >= config.pdh_bins) b = config.pdh_bins - 1;
            hist[b] += 1.0;
        }
        return hist;
    }

    std::vector<double> centers(config.pdh_bins);
    const double width = range / config.pdh_bins;
    for (int i = 0; i < config.pdh_bins; ++i) {
        centers[i] = config.pdh_min_dist + width * (static_cast<double>(i) + 0.5);
    }
    for (double d : distances) {
        int ci = 0;
        while (ci < config.pdh_bins && centers[ci] < d) ++ci;
        if (ci >= config.pdh_bins) ci = config.pdh_bins - 1;
        const double t = std::abs(d - centers[ci]) / (width + 1e-12);
        if (t <= 1.0) {
            hist[ci] += 1.0 - t;
            const double spill = t;
            if (d < centers[ci] && ci - 1 >= 0) hist[ci - 1] += spill;
            else if (d > centers[ci] && ci + 1 < config.pdh_bins) hist[ci + 1] += spill;
        } else {
            hist[ci] += 1.0;
        }
    }
    return hist;
}

TriangleSet ComputeKnnTriangles(const std::vector<Eigen::Vector2d>& centers,
                                int k,
                                double min_dist,
                                double max_dist) {
    TriangleSet out;
    if (centers.size() < 3) return out;
    std::set<std::array<int, 3>> triples;
    const double min2 = min_dist * min_dist;
    const double max2 = max_dist * max_dist;
    for (int i = 0; i < static_cast<int>(centers.size()); ++i) {
        std::vector<std::pair<double, int>> distances;
        for (int j = 0; j < static_cast<int>(centers.size()); ++j) {
            distances.emplace_back((centers[i] - centers[j]).squaredNorm(), j);
        }
        std::sort(distances.begin(), distances.end());
        const int found = std::min(static_cast<int>(distances.size()), k + 1);
        if (found < 3) continue;
        std::vector<int> valid;
        valid.reserve(found);
        for (int t = 1; t < found; ++t) {
            if (distances[t].first >= min2 && distances[t].first <= max2) valid.push_back(distances[t].second);
        }
        for (int a = 0; a < static_cast<int>(valid.size()); ++a) {
            for (int b = a + 1; b < static_cast<int>(valid.size()); ++b) {
                const int j = valid[a];
                const int l = valid[b];
                const double djl2 = (centers[j] - centers[l]).squaredNorm();
                if (djl2 < min2 || djl2 > max2) continue;
                const Eigen::Vector2d v1 = centers[j] - centers[i];
                const Eigen::Vector2d v2 = centers[l] - centers[i];
                const double area = 0.5 * std::abs(v1.x() * v2.y() - v1.y() * v2.x());
                if (area < 1e-4) continue;
                triples.insert(OrderedTriangle(centers, i, j, l));
            }
        }
    }
    out.simplices.assign(triples.begin(), triples.end());
    return out;
}

std::vector<std::pair<long long, int>> TriangleHashes(const TriangleSet& triangles,
                                                      const std::vector<Eigen::Vector2d>& centers,
                                                      double delta_l,
                                                      long long rho,
                                                      long long hash_modulus) {
    std::vector<std::pair<long long, int>> hashes;
    hashes.reserve(triangles.simplices.size());
    for (int i = 0; i < static_cast<int>(triangles.simplices.size()); ++i) {
        const auto& tri = triangles.simplices[i];
        std::array<double, 3> sides = {
            (centers[tri[0]] - centers[tri[1]]).norm(),
            (centers[tri[1]] - centers[tri[2]]).norm(),
            (centers[tri[2]] - centers[tri[0]]).norm()
        };
        std::sort(sides.begin(), sides.end());
        const double s = (sides[0] + sides[1] + sides[2]) * 0.5;
        const double area = std::sqrt(std::max(0.0, s * (s - sides[0]) * (s - sides[1]) * (s - sides[2])));
        long long h = TriangleHash(sides[0], sides[1], sides[2], delta_l, rho, hash_modulus);
        h = (h * rho + static_cast<long long>(std::llround(area / delta_l))) % hash_modulus;
        hashes.emplace_back(h, i);
    }
    return hashes;
}

double ChiSquared(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs) {
    double sum = 0.0;
    for (int i = 0; i < lhs.rows(); ++i) {
        for (int j = 0; j < lhs.cols(); ++j) {
            const double diff = lhs(i, j) - rhs(i, j);
            sum += diff * diff / (lhs(i, j) + rhs(i, j) + 1e-10);
        }
    }
    return sum;
}

double ChiSquared(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    double sum = 0.0;
    const int n = std::min(lhs.size(), rhs.size());
    for (int i = 0; i < n; ++i) {
        const double diff = lhs[i] - rhs[i];
        sum += diff * diff / (lhs[i] + rhs[i] + 1e-10);
    }
    return sum;
}

}  // namespace treelocpp
