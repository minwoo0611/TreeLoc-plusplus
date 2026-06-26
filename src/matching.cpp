#include "treelocpp/matching.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "treelocpp/descriptors.h"
#include "treelocpp/geometry.h"

namespace treelocpp {

namespace {

constexpr std::array<std::array<int, 3>, 6> kPerms{{
    {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
    {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}
}};

double TreeRadius(const Tree& tree) {
    return std::isfinite(tree.dbh) ? tree.dbh : tree.dbh_approximation;
}

Eigen::Matrix3d ProjectSO3(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d out = U * V.transpose();
    if (out.determinant() < 0.0) {
        U.col(2) *= -1.0;
        out = U * V.transpose();
    }
    return out;
}

struct PlaneCorrection {
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    int inliers = 0;
};

PlaneCorrection EstimatePlaneCorrection(const std::vector<double>& delta_z,
                                        const std::vector<Eigen::Vector2d>& candidate_xy,
                                        const Config& config) {
    PlaneCorrection out;
    const int n = static_cast<int>(delta_z.size());
    if (n == 0 || candidate_xy.size() != delta_z.size()) return out;
    if (n < 3) {
        out.z = std::accumulate(delta_z.begin(), delta_z.end(), 0.0) / n;
        out.inliers = n;
        return out;
    }

    Eigen::MatrixXd A_full(n, 3);
    Eigen::VectorXd b_full(n);
    for (int i = 0; i < n; ++i) {
        A_full(i, 0) = 1.0;
        A_full(i, 1) = candidate_xy[i].y();
        A_full(i, 2) = -candidate_xy[i].x();
        b_full(i) = delta_z[i];
    }

    const int sample_size = std::min(std::max(3, config.vertical_min_sample), n);
    std::mt19937_64 rng(1234);
    int best_count = -1;
    std::vector<char> best_inlier;
    Eigen::Vector3d best_beta = Eigen::Vector3d::Zero();

    for (int iter = 0; iter < std::max(1, config.vertical_ransac_iters); ++iter) {
        std::vector<int> sample;
        sample.reserve(sample_size);
        if (sample_size == n) {
            sample.resize(n);
            std::iota(sample.begin(), sample.end(), 0);
        } else {
            std::unordered_set<int> used;
            while (static_cast<int>(sample.size()) < sample_size) {
                const int idx = static_cast<int>(rng() % static_cast<uint64_t>(n));
                if (used.insert(idx).second) sample.push_back(idx);
            }
        }

        Eigen::MatrixXd A(sample_size, 3);
        Eigen::VectorXd b(sample_size);
        for (int i = 0; i < sample_size; ++i) {
            A.row(i) = A_full.row(sample[i]);
            b(i) = b_full(sample[i]);
        }
        const Eigen::Vector3d beta = A.colPivHouseholderQr().solve(b);
        const Eigen::VectorXd residual = (A_full * beta - b_full).cwiseAbs();

        std::vector<char> inlier(n, 0);
        int count = 0;
        for (int i = 0; i < n; ++i) {
            if (residual(i) <= config.z_inlier_tol) {
                inlier[i] = 1;
                ++count;
            }
        }
        if (count > best_count) {
            best_count = count;
            best_inlier = std::move(inlier);
            best_beta = beta;
        }
    }

    if (best_count < 3) {
        out.z = std::accumulate(delta_z.begin(), delta_z.end(), 0.0) / n;
        out.inliers = n;
        return out;
    }

    Eigen::MatrixXd A_inlier(best_count, 3);
    Eigen::VectorXd b_inlier(best_count);
    int row = 0;
    for (int i = 0; i < n; ++i) {
        if (!best_inlier[i]) continue;
        A_inlier.row(row) = A_full.row(i);
        b_inlier(row) = b_full(i);
        ++row;
    }
    best_beta = A_inlier.colPivHouseholderQr().solve(b_inlier);
    out.z = best_beta(0);
    out.roll = best_beta(1);
    out.pitch = best_beta(2);
    out.inliers = best_count;
    return out;
}

Eigen::Matrix3d EstimateAxisRollPitch(const std::vector<Eigen::Vector3d>& query_up,
                                      const std::vector<Eigen::Vector3d>& candidate_up,
                                      const Eigen::Matrix3d& Rz_q2c,
                                      int ransac_iters = 200,
                                      double inlier_angle_deg = 2.5,
                                      double min_inlier_ratio = 0.5) {
    std::vector<Eigen::Vector3d> uq;
    std::vector<Eigen::Vector3d> uc_in_q;
    uq.reserve(query_up.size());
    uc_in_q.reserve(candidate_up.size());
    for (size_t i = 0; i < query_up.size() && i < candidate_up.size(); ++i) {
        Eigen::Vector3d q = query_up[i];
        Eigen::Vector3d c = Rz_q2c.transpose() * candidate_up[i];
        if (!q.allFinite() || !c.allFinite() || q.norm() < 1e-9 || c.norm() < 1e-9) continue;
        q.normalize();
        c.normalize();
        if (q.dot(c) < 0.0) c = -c;
        uq.push_back(q);
        uc_in_q.push_back(c);
    }
    const int n = static_cast<int>(uq.size());
    if (n == 0) return Eigen::Matrix3d::Identity();

    auto fit_kabsch = [&](const std::vector<int>& ids) {
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (int id : ids) M += uc_in_q[id] * uq[id].transpose();
        return ProjectSO3(M);
    };

    const double threshold = inlier_angle_deg * M_PI / 180.0;
    auto is_inlier = [&](const Eigen::Matrix3d& R, int idx) {
        double c = (R * uq[idx]).dot(uc_in_q[idx]);
        c = std::clamp(c, -1.0, 1.0);
        return std::acos(c) <= threshold;
    };

    int best_count = 0;
    std::vector<int> best_set;
    Eigen::Matrix3d best_R = Eigen::Matrix3d::Identity();
    std::mt19937_64 rng(1234);
    std::uniform_int_distribution<int> uni(0, n - 1);

    for (int iter = 0; iter < std::max(1, ransac_iters); ++iter) {
        std::vector<int> seed;
        if (n == 1) {
            seed = {0};
        } else {
            int i0 = uni(rng);
            int i1 = uni(rng);
            while (i1 == i0 && n > 1) i1 = uni(rng);
            seed = {i0, i1};
        }
        const Eigen::Matrix3d R = fit_kabsch(seed);
        std::vector<int> inliers;
        for (int i = 0; i < n; ++i) {
            if (is_inlier(R, i)) inliers.push_back(i);
        }
        if (static_cast<int>(inliers.size()) > best_count) {
            best_count = static_cast<int>(inliers.size());
            best_set = std::move(inliers);
            best_R = R;
        }
    }

    if (best_count < 5 || best_count < static_cast<int>(std::ceil(min_inlier_ratio * n))) {
        return Eigen::Matrix3d::Identity();
    }

    best_R = fit_kabsch(best_set);
    const double yaw = std::atan2(best_R(1, 0), best_R(0, 0));
    const Eigen::Matrix3d R_rp =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * best_R;
    return ProjectSO3(R_rp);
}

bool Rigid2D(const std::vector<Eigen::Vector2d>& q,
             const std::vector<Eigen::Vector2d>& c,
             Eigen::Matrix2d& R,
             Eigen::Vector2d& t) {
    if (q.size() != c.size() || q.size() < 2) return false;
    Eigen::MatrixXd Q(q.size(), 2);
    Eigen::MatrixXd C(c.size(), 2);
    for (size_t i = 0; i < q.size(); ++i) {
        Q.row(i) = q[i];
        C.row(i) = c[i];
    }
    const Eigen::Vector2d qc = Q.colwise().mean();
    const Eigen::Vector2d cc = C.colwise().mean();
    Q.rowwise() -= qc.transpose();
    C.rowwise() -= cc.transpose();
    const Eigen::Matrix2d H = Q.transpose() * C;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0.0) {
        Eigen::Matrix2d V = svd.matrixV();
        V.col(1) *= -1.0;
        R = V * svd.matrixU().transpose();
    }
    t = cc - R * qc;
    return true;
}

double TriangleResidual(const std::array<Eigen::Vector2d, 3>& q,
                        const std::array<Eigen::Vector2d, 3>& c) {
    std::vector<Eigen::Vector2d> qv(q.begin(), q.end());
    std::vector<Eigen::Vector2d> cv(c.begin(), c.end());
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    if (!Rigid2D(qv, cv, R, t)) return std::numeric_limits<double>::infinity();
    double err = 0.0;
    for (int i = 0; i < 3; ++i) err += (R * q[i] + t - c[i]).norm();
    return err / 3.0;
}

bool BestTrianglePermutation(const FrameData& qf,
                             const FrameData& cf,
                             const std::array<int, 3>& qt,
                             const std::array<int, 3>& ct,
                             const Config& config,
                             std::array<int, 3>& best_perm) {
    std::array<Eigen::Vector2d, 3> qpts;
    std::array<Eigen::Vector2d, 3> csrc;
    for (int i = 0; i < 3; ++i) {
        qpts[i] = qf.centers[qt[i]];
        csrc[i] = cf.centers[ct[i]];
    }
    double best = std::numeric_limits<double>::infinity();
    bool ok = false;
    for (const auto& perm : kPerms) {
        if (config.use_dbh_triangle_match) {
            bool dbh_ok = true;
            for (int i = 0; i < 3; ++i) {
                const double dq = TreeRadius(qf.trees[qt[i]]);
                const double dc = TreeRadius(cf.trees[ct[perm[i]]]);
                if (!std::isfinite(dq) || !std::isfinite(dc) ||
                    std::abs(dq - dc) >= config.dbh_diff_tol) {
                    dbh_ok = false;
                    break;
                }
            }
            if (!dbh_ok) continue;
        }
        std::array<Eigen::Vector2d, 3> cpts{csrc[perm[0]], csrc[perm[1]], csrc[perm[2]]};
        const double err = TriangleResidual(qpts, cpts);
        if (err < best) {
            best = err;
            best_perm = perm;
            ok = true;
        }
    }
    return ok;
}

std::unordered_map<long long, std::vector<int>> HashGroups(const std::vector<std::pair<long long, int>>& hashes) {
    std::unordered_map<long long, std::vector<int>> out;
    for (const auto& item : hashes) out[item.first].push_back(item.second);
    return out;
}

std::vector<MatchPair> NearestMatches(const FrameData& qf,
                                      const FrameData& cf,
                                      const Eigen::Matrix2d& R,
                                      const Eigen::Vector2d& t,
                                      const Config& config) {
    std::vector<MatchPair> matches;
    std::vector<char> q_used(qf.trees.size(), 0);
    for (int ci = 0; ci < static_cast<int>(cf.trees.size()); ++ci) {
        int best = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        const Eigen::Vector2d cp(cf.trees[ci].x, cf.trees[ci].y);
        for (int qi = 0; qi < static_cast<int>(qf.trees.size()); ++qi) {
            if (q_used[qi]) continue;
            const double dr = std::abs(TreeRadius(qf.trees[qi]) - TreeRadius(cf.trees[ci]));
            if (dr >= std::max(0.4, config.dbh_diff_tol * 2.0)) continue;
            const Eigen::Vector2d qp = R * Eigen::Vector2d(qf.trees[qi].x, qf.trees[qi].y) + t;
            const double d2 = (qp - cp).squaredNorm();
            if (d2 < best_d2) {
                best_d2 = d2;
                best = qi;
            }
        }
        if (best >= 0 && std::sqrt(best_d2) <= config.match_distance_tol) {
            q_used[best] = 1;
            matches.push_back({best, ci});
        }
    }
    return matches;
}

double OverlapScore(const FrameData& qf, const FrameData& cf, const std::vector<MatchPair>& matches) {
    const int inter = static_cast<int>(matches.size());
    const int uni = static_cast<int>(qf.trees.size() + cf.trees.size() - inter);
    return uni > 0 ? static_cast<double>(inter) / uni : 0.0;
}

double TPenalty(const Eigen::Vector2d& t, const Config& config) {
    if (!config.use_t_aware_overlap) return 1.0;
    const double norm = t.norm();
    if (config.t_aware_mode == "inv") {
        return 1.0 / (1.0 + std::pow(norm / std::max(config.t_aware_tau, 1e-6), config.t_aware_power));
    }
    return std::exp(-norm / std::max(config.t_aware_tau, 1e-6));
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d> VoteYawAndFit(
    const std::vector<Eigen::Vector2d>& q_centers,
    const std::vector<Eigen::Vector2d>& c_centers,
    const std::vector<double>& yaws,
    const Config& config) {
    std::vector<int> keep;
    if (config.use_yaw_voting && static_cast<int>(yaws.size()) >= config.yaw_min_tri_inliers) {
        const int bins = std::max(1, static_cast<int>(std::llround(360.0 / config.yaw_bin_deg)));
        std::vector<int> counts(bins, 0);
        for (double yaw : yaws) {
            int b = static_cast<int>(std::floor((WrapAngle(yaw) + M_PI) / (2.0 * M_PI) * bins));
            b = std::clamp(b, 0, bins - 1);
            ++counts[b];
        }
        const int mode = static_cast<int>(std::max_element(counts.begin(), counts.end()) - counts.begin());
        const double center = -M_PI + (mode + 0.5) * (2.0 * M_PI / bins);
        const double tol = config.yaw_inlier_tol_deg * M_PI / 180.0;
        for (int i = 0; i < static_cast<int>(yaws.size()); ++i) {
            if (std::abs(WrapAngle(yaws[i] - center)) <= tol) keep.push_back(i);
        }
    }
    if (keep.size() < 2) {
        keep.resize(q_centers.size());
        std::iota(keep.begin(), keep.end(), 0);
    }
    std::vector<Eigen::Vector2d> q;
    std::vector<Eigen::Vector2d> c;
    for (int idx : keep) {
        q.push_back(q_centers[idx]);
        c.push_back(c_centers[idx]);
    }
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    Rigid2D(q, c, R, t);
    return {R, t};
}

void WeightedRigid2D(const Eigen::MatrixXd& q,
                     const Eigen::MatrixXd& c,
                     const Eigen::VectorXd& weights,
                     Eigen::Matrix2d& R,
                     Eigen::Vector2d& t) {
    Eigen::VectorXd w = weights.cwiseMax(1e-6);
    w /= w.sum();
    const Eigen::Vector2d qc = (q.array().colwise() * w.array()).colwise().sum();
    const Eigen::Vector2d cc = (c.array().colwise() * w.array()).colwise().sum();
    Eigen::MatrixXd q0 = q.rowwise() - qc.transpose();
    Eigen::MatrixXd c0 = c.rowwise() - cc.transpose();
    const Eigen::Matrix2d H = (q0.array().colwise() * w.array()).matrix().transpose() * c0;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d V = svd.matrixV();
    R = V * svd.matrixU().transpose();
    if (R.determinant() < 0.0) {
        V.col(1) *= -1.0;
        R = V * svd.matrixU().transpose();
    }
    t = cc - R * qc;
}

std::vector<int> DedupReps(const std::vector<Eigen::Vector2d>& pts) {
    std::unordered_map<std::string, int> reps;
    std::vector<int> out(pts.size());
    reps.reserve(pts.size() * 2);
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << pts[i].x() << "," << pts[i].y();
        const std::string key = ss.str();
        auto it = reps.find(key);
        if (it == reps.end()) {
            reps[key] = i;
            out[i] = i;
        } else {
            out[i] = it->second;
        }
    }
    return out;
}

void HungarianMinCost(const Eigen::MatrixXd& cost,
                      std::vector<int>& row_ind,
                      std::vector<int>& col_ind) {
    const int n = static_cast<int>(cost.rows());
    const int m = static_cast<int>(cost.cols());
    const int N = std::max(n, m);
    row_ind.clear();
    col_ind.clear();
    if (N == 0) return;
    const double big = 1e9;
    Eigen::MatrixXd a = Eigen::MatrixXd::Constant(N, N, big);
    a.block(0, 0, n, m) = cost;
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            if (!std::isfinite(a(r, c))) a(r, c) = big;
        }
    }
    std::vector<double> u(N + 1, 0.0), v(N + 1, 0.0), minv(N + 1);
    std::vector<int> p(N + 1, 0), way(N + 1, 0);
    std::vector<char> used(N + 1);
    for (int i = 1; i <= N; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), big);
        std::fill(used.begin(), used.end(), false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            int j1 = 0;
            double delta = big;
            for (int j = 1; j <= N; ++j) {
                if (used[j]) continue;
                double cur = a(i0 - 1, j - 1) - u[i0] - v[j];
                if (!std::isfinite(cur)) cur = big;
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            if (j1 == 0 || !std::isfinite(delta) || delta >= big * 0.5) {
                row_ind.clear();
                col_ind.clear();
                return;
            }
            for (int j = 0; j <= N; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    std::vector<int> assignment(N, -1);
    for (int j = 1; j <= N; ++j) {
        if (p[j] > 0) assignment[p[j] - 1] = j - 1;
    }
    for (int i = 0; i < n; ++i) {
        const int j = assignment[i];
        if (0 <= j && j < m) {
            row_ind.push_back(i);
            col_ind.push_back(j);
        }
    }
}

int NearestQuery(const FrameData& qf, const Eigen::Matrix2d& R, const Eigen::Vector2d& t,
                 const Eigen::Vector2d& cp, double max_dist, double* best_dist) {
    int best = -1;
    double best_d2 = max_dist * max_dist;
    for (int qi = 0; qi < static_cast<int>(qf.trees.size()); ++qi) {
        const Eigen::Vector2d qp = R * Eigen::Vector2d(qf.trees[qi].x, qf.trees[qi].y) + t;
        const double d2 = (qp - cp).squaredNorm();
        if (d2 < best_d2) {
            best_d2 = d2;
            best = qi;
        }
    }
    if (best_dist) *best_dist = std::sqrt(best_d2);
    return best;
}

int HashIntersection(const FrameData& qf, const FrameData& cf) {
    int score = 0;
    for (const auto& kv : qf.hash_counts) {
        auto it = cf.hash_counts.find(kv.first);
        if (it != cf.hash_counts.end()) score += std::min(kv.second, it->second);
    }
    return score;
}

}  // namespace

Transform2D EstimateTransform2D(const FrameData& query,
                                const FrameData& candidate,
                                const Config& config) {
    Transform2D out;
    const auto q_groups = HashGroups(query.hashes);
    const auto c_groups = HashGroups(candidate.hashes);

    std::vector<Eigen::Vector2d> q_centers;
    std::vector<Eigen::Vector2d> c_centers;
    std::vector<double> tri_yaws;
    std::vector<std::pair<int, int>> tri_pairs;

    auto best_perm_residual = [&](const std::array<int, 3>& qt,
                                  const std::array<int, 3>& ct,
                                  std::array<int, 3>& best_perm) {
        std::array<Eigen::Vector2d, 3> qpts;
        std::array<Eigen::Vector2d, 3> csrc;
        for (int i = 0; i < 3; ++i) {
            qpts[i] = query.centers[qt[i]];
            csrc[i] = candidate.centers[ct[i]];
        }
        double best = std::numeric_limits<double>::infinity();
        for (const auto& perm : kPerms) {
            std::array<Eigen::Vector2d, 3> cpts{csrc[perm[0]], csrc[perm[1]], csrc[perm[2]]};
            const double err = TriangleResidual(qpts, cpts);
            if (err < best) {
                best = err;
                best_perm = perm;
            }
        }
        return std::isfinite(best);
    };

    auto dbh_perm_cost = [&](const std::array<int, 3>& qt,
                             const std::array<int, 3>& ct,
                             std::array<int, 3>& best_perm) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto& perm : kPerms) {
            double cost = 0.0;
            bool ok = true;
            for (int i = 0; i < 3; ++i) {
                const double dq = TreeRadius(query.trees[qt[i]]);
                const double dc = TreeRadius(candidate.trees[ct[perm[i]]]);
                const double diff = std::abs(dq - dc);
                if (!std::isfinite(dq) || !std::isfinite(dc) || diff >= config.dbh_diff_tol) {
                    ok = false;
                    break;
                }
                cost += diff;
            }
            if (ok && cost < best) {
                best = cost;
                best_perm = perm;
            }
        }
        return best;
    };

    auto dbh_permutation_ok = [&](const std::array<int, 3>& qt,
                                  const std::array<int, 3>& ct,
                                  const std::array<int, 3>& perm) {
        for (int i = 0; i < 3; ++i) {
            const double dq = TreeRadius(query.trees[qt[i]]);
            const double dc = TreeRadius(candidate.trees[ct[perm[i]]]);
            if (!std::isfinite(dq) || !std::isfinite(dc) || std::abs(dq - dc) >= config.dbh_diff_tol) {
                return false;
            }
        }
        return true;
    };

    auto add_triangle = [&](int qti, int cti, const std::array<int, 3>& perm) {
        const auto& qt = query.triangles.simplices[qti];
        const auto& ct = candidate.triangles.simplices[cti];
        std::vector<Eigen::Vector2d> qv;
        std::vector<Eigen::Vector2d> cv;
        qv.reserve(3);
        cv.reserve(3);
        for (int k = 0; k < 3; ++k) {
            qv.push_back(query.centers[qt[k]]);
            cv.push_back(candidate.centers[ct[perm[k]]]);
            tri_pairs.push_back({qt[k], ct[perm[k]]});
        }
        Eigen::Matrix2d R;
        Eigen::Vector2d t;
        if (Rigid2D(qv, cv, R, t)) {
            tri_yaws.push_back(RotationYaw(R));
            q_centers.push_back((qv[0] + qv[1] + qv[2]) / 3.0);
            c_centers.push_back((cv[0] + cv[1] + cv[2]) / 3.0);
        }
    };

    for (const auto& kv : q_groups) {
        auto cit = c_groups.find(kv.first);
        if (cit == c_groups.end()) continue;
        const auto& q_list = kv.second;
        const auto& c_list = cit->second;
        if (config.use_dbh_triangle_match) {
            if (q_list.size() == 1 && c_list.size() == 1) {
                std::array<int, 3> perm{0, 1, 2};
                if (best_perm_residual(query.triangles.simplices[q_list[0]],
                                       candidate.triangles.simplices[c_list[0]],
                                       perm) &&
                    dbh_permutation_ok(query.triangles.simplices[q_list[0]],
                                       candidate.triangles.simplices[c_list[0]],
                                       perm)) {
                    add_triangle(q_list[0], c_list[0], perm);
                }
                continue;
            }

            Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(q_list.size(), c_list.size(), 1e9);
            std::map<std::pair<int, int>, std::array<int, 3>> best_perm;
            for (int qi = 0; qi < static_cast<int>(q_list.size()); ++qi) {
                for (int ci = 0; ci < static_cast<int>(c_list.size()); ++ci) {
                    std::array<int, 3> perm{0, 1, 2};
                    const double c = dbh_perm_cost(query.triangles.simplices[q_list[qi]],
                                                   candidate.triangles.simplices[c_list[ci]],
                                                   perm);
                    if (std::isfinite(c)) {
                        cost(qi, ci) = c;
                        best_perm[{qi, ci}] = perm;
                    }
                }
            }
            std::vector<int> rows;
            std::vector<int> cols;
            HungarianMinCost(cost, rows, cols);
            for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
                const int r = rows[i];
                const int c = cols[i];
                if (cost(r, c) >= 1e9) continue;
                auto it = best_perm.find({r, c});
                if (it == best_perm.end()) continue;
                add_triangle(q_list[r], c_list[c], it->second);
            }
        } else {
            for (int qti : q_list) {
                for (int cti : c_list) {
                    std::array<int, 3> perm{0, 1, 2};
                    if (best_perm_residual(query.triangles.simplices[qti], candidate.triangles.simplices[cti], perm)) {
                        add_triangle(qti, cti, perm);
                    }
                }
            }
        }
    }

    const auto q_reps = DedupReps(query.centers);
    const auto c_reps = DedupReps(candidate.centers);
    std::set<std::pair<int, int>> pair_set;
    for (const auto& pair : tri_pairs) pair_set.insert({q_reps[pair.first], c_reps[pair.second]});
    if (pair_set.size() < 2) return out;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    if (config.use_yaw_voting && static_cast<int>(tri_yaws.size()) >= config.yaw_min_tri_inliers) {
        const int nbin = static_cast<int>(std::llround(360.0 / std::max(config.yaw_bin_deg, 1e-6))) + 1;
        std::vector<int> hist(nbin - 1, 0);
        std::vector<double> edges(nbin);
        for (int i = 0; i < nbin; ++i) {
            edges[i] = -M_PI + (2.0 * M_PI) * static_cast<double>(i) / static_cast<double>(nbin - 1);
        }
        for (double yaw : tri_yaws) {
            int bin = 0;
            while (bin < nbin - 1 && !(edges[bin] <= yaw && yaw < edges[bin + 1])) ++bin;
            if (bin >= nbin - 1) bin = nbin - 2;
            ++hist[bin];
        }
        const int mode = static_cast<int>(std::max_element(hist.begin(), hist.end()) - hist.begin());
        const double center = 0.5 * (edges[mode] + edges[mode + 1]);
        const double tol = config.yaw_inlier_tol_deg * M_PI / 180.0;
        std::vector<Eigen::Vector2d> q_init;
        std::vector<Eigen::Vector2d> c_init;
        for (int i = 0; i < static_cast<int>(tri_yaws.size()); ++i) {
            if (std::abs(WrapAngle(tri_yaws[i] - center)) <= tol) {
                q_init.push_back(q_centers[i]);
                c_init.push_back(c_centers[i]);
            }
        }
        if (q_init.size() >= 2) {
            Rigid2D(q_init, c_init, R, t);
        }
    }

    Eigen::MatrixXd q_pairs(pair_set.size(), 2);
    Eigen::MatrixXd c_pairs(pair_set.size(), 2);
    int row = 0;
    for (const auto& pair : pair_set) {
        q_pairs.row(row) = query.centers[pair.first];
        c_pairs.row(row) = candidate.centers[pair.second];
        ++row;
    }
    Eigen::MatrixXd pred = ((q_pairs * R.transpose()).rowwise() + t.transpose()).eval();
    Eigen::VectorXd residual = (pred - c_pairs).rowwise().norm();
    Eigen::VectorXd weights = residual.unaryExpr([](double r) {
        return r <= 0.4 ? 1.0 : 0.4 / (r + 1e-9);
    });
    for (int iter = 0; iter < 5; ++iter) {
        WeightedRigid2D(q_pairs, c_pairs, weights, R, t);
        pred = ((q_pairs * R.transpose()).rowwise() + t.transpose()).eval();
        residual = (pred - c_pairs).rowwise().norm();
        weights = residual.unaryExpr([](double r) {
            return r <= 0.4 ? 1.0 : 0.4 / (r + 1e-9);
        });
    }

    std::vector<Eigen::Vector2d> refine_q;
    std::vector<Eigen::Vector2d> refine_c;
    for (int ci = 0; ci < static_cast<int>(candidate.trees.size()); ++ci) {
        const Eigen::Vector2d cp(candidate.trees[ci].x, candidate.trees[ci].y);
        double dist = 0.0;
        const int qi = NearestQuery(query, R, t, cp, config.match_distance_tol, &dist);
        if (qi >= 0) {
            refine_q.emplace_back(query.trees[qi].x, query.trees[qi].y);
            refine_c.push_back(cp);
        }
    }
    if (refine_q.size() >= 2) {
        Rigid2D(refine_q, refine_c, R, t);
    }

    std::vector<MatchPair> matches;
    for (int ci = 0; ci < static_cast<int>(candidate.trees.size()); ++ci) {
        const Eigen::Vector2d cp(candidate.trees[ci].x, candidate.trees[ci].y);
        double dist = 0.0;
        const int qi = NearestQuery(query, R, t, cp, config.match_distance_tol, &dist);
        if (qi >= 0) matches.push_back({qi, ci});
    }

    out.R = R;
    out.t = t;
    out.pairs = std::move(matches);
    out.overlap = OverlapScore(query, candidate, out.pairs) * TPenalty(t, config);
    out.ok = out.pairs.size() >= 2;
    return out;
}

PoseCorrection EstimateVerticalCorrection(const FrameData& query,
                                          const FrameData& candidate,
                                          const Transform2D& transform,
                                          const Config& config) {
    PoseCorrection out;
    if (!config.use_vertical || transform.pairs.empty()) return out;

    std::vector<Eigen::Vector3d> uq;
    std::vector<Eigen::Vector3d> uc;
    uq.reserve(transform.pairs.size());
    uc.reserve(transform.pairs.size());
    for (const auto& pair : transform.pairs) {
        const Tree& qt = query.trees[pair.query];
        const Tree& ct = candidate.trees[pair.candidate];
        uq.push_back(TreeUp(qt));
        uc.push_back(TreeUp(ct));
    }

    Eigen::Matrix3d R_rp_q2c = Eigen::Matrix3d::Identity();
    if (uq.size() >= 2) {
        Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
        Rz.block<2, 2>(0, 0) = transform.R;
        R_rp_q2c = EstimateAxisRollPitch(uq, uc, Rz);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw_rp = 0.0;
        EulerZYX(R_rp_q2c, roll, pitch, yaw_rp);
        out.axis_roll = roll;
        out.axis_pitch = pitch;
    }

    Eigen::Matrix3d Rz_q2c = Eigen::Matrix3d::Identity();
    Rz_q2c.block<2, 2>(0, 0) = transform.R;
    const Eigen::Matrix3d R_q2c = Rz_q2c * R_rp_q2c;
    const Eigen::Vector3d t_q2c(transform.t.x(), transform.t.y(), 0.0);
    const Eigen::Matrix3d R_c2q = R_q2c.transpose();
    const Eigen::Vector3d t_c2q = -R_c2q * t_q2c;

    std::vector<double> dz;
    std::vector<Eigen::Vector2d> candidate_xy;
    dz.reserve(transform.pairs.size());
    candidate_xy.reserve(transform.pairs.size());
    for (const auto& pair : transform.pairs) {
        const Tree& qt = query.trees[pair.query];
        const Tree& ct = candidate.trees[pair.candidate];
        if (!std::isfinite(qt.z) || !std::isfinite(ct.z)) continue;
        const Eigen::Vector3d c_in_q =
            R_c2q * Eigen::Vector3d(ct.x, ct.y, ct.z) + t_c2q;
        if (!c_in_q.allFinite()) continue;
        dz.push_back(qt.z - c_in_q.z());
        candidate_xy.emplace_back(c_in_q.x(), c_in_q.y());
    }
    const PlaneCorrection plane = EstimatePlaneCorrection(dz, candidate_xy, config);
    if (plane.inliers > 0) {
        out.z = plane.z;
        out.roll = plane.roll;
        out.pitch = plane.pitch;
        out.inliers = plane.inliers;
    }
    return out;
}

std::vector<CandidateResult> RankCandidates(const Dataset& query_set,
                                            const Dataset& database_set,
                                            size_t query_slot,
                                            const std::vector<size_t>& candidate_slots,
                                            const Config& config) {
    const FrameData& qf = query_set.frames[query_slot];
    struct Score {
        size_t slot;
        double tdh;
        double pw;
        double combined;
        int hash;
    };
    std::vector<Score> scores;
    scores.reserve(candidate_slots.size());
    for (size_t slot : candidate_slots) {
        const FrameData& cf = database_set.frames[slot];
        scores.push_back({slot, ChiSquared(qf.tdh, cf.tdh), ChiSquared(qf.pairwise, cf.pairwise), 0.0, 0});
    }
    if (scores.empty()) return {};

    auto minmax = [](const std::vector<Score>& v, auto member) {
        double mn = std::numeric_limits<double>::infinity();
        double mx = -std::numeric_limits<double>::infinity();
        for (const auto& s : v) {
            const double x = member(s);
            mn = std::min(mn, x);
            mx = std::max(mx, x);
        }
        return std::pair<double, double>{mn, mx};
    };
    const auto [tdh_min, tdh_max] = minmax(scores, [](const Score& s) { return s.tdh; });
    const auto [pw_min, pw_max] = minmax(scores, [](const Score& s) { return s.pw; });
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
    for (auto& s : scores) {
        s.hash = HashIntersection(qf, database_set.frames[s.slot]);
    }
    std::sort(scores.begin(), scores.end(), [](const Score& a, const Score& b) {
        if (a.hash != b.hash) return a.hash > b.hash;
        return a.combined < b.combined;
    });
    if (static_cast<int>(scores.size()) > config.rerank_k) scores.resize(config.rerank_k);

    std::vector<CandidateResult> results;
    results.reserve(scores.size());
    for (const auto& score : scores) {
        const FrameData& cf = database_set.frames[score.slot];
        CandidateResult result;
        result.query_index = qf.index;
        result.candidate_index = cf.index;
        result.retrieval_score = score.combined;
        result.hash_score = score.hash;
        result.transform = EstimateTransform2D(qf, cf, config);
        result.vertical = EstimateVerticalCorrection(qf, cf, result.transform, config);
        result.spatial_error = PoseDistanceXY(qf.pose, cf.pose);
        results.push_back(std::move(result));
    }
    std::sort(results.begin(), results.end(), [](const CandidateResult& a, const CandidateResult& b) {
        if (a.transform.overlap != b.transform.overlap) return a.transform.overlap > b.transform.overlap;
        if (a.hash_score != b.hash_score) return a.hash_score > b.hash_score;
        return a.retrieval_score < b.retrieval_score;
    });
    return results;
}

}  // namespace treelocpp
