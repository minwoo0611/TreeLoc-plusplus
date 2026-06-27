#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace fs = std::filesystem;

struct CrossEdge {
    size_t q_idx = 0;
    size_t db_idx = 0;
    double overlap = 0.0;
    gtsam::Vector6 meas = gtsam::Vector6::Zero();
};

struct SessionMeta {
    std::string label;
    char key = 'A';
    fs::path slam_csv;
};

std::string Trim(const std::string& s) {
    const size_t begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) out.push_back(Trim(cell));
    return out;
}

std::vector<SessionMeta> LoadSessions(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open session list: " + path.string());
    std::vector<SessionMeta> out;
    std::string line;
    while (std::getline(in, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto cells = SplitCsv(line);
        if (cells.size() != 2 && cells.size() != 3) {
            throw std::runtime_error("session line must be label,slam_csv or label,key,slam_csv: " + line);
        }
        SessionMeta meta;
        meta.label = cells[0];
        if (cells.size() == 2) {
            meta.key = static_cast<char>('A' + out.size());
            meta.slam_csv = cells[1];
        } else {
            if (cells[1].size() != 1) throw std::runtime_error("session key must be one character: " + line);
            meta.key = cells[1][0];
            meta.slam_csv = cells[2];
        }
        out.push_back(std::move(meta));
    }
    return out;
}

std::vector<gtsam::Pose3> LoadSlamCsv(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open slam CSV: " + path.string());
    std::vector<gtsam::Pose3> out;
    std::string line;
    while (std::getline(in, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string token;
        if (!std::getline(ss, token, ',')) continue;
        if (!std::getline(ss, token, ',')) continue;
        if (!std::getline(ss, token, ',')) continue;
        double x = 0.0, y = 0.0, z = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        if (!std::getline(ss, token, ',')) continue; x = std::stod(token);
        if (!std::getline(ss, token, ',')) continue; y = std::stod(token);
        if (!std::getline(ss, token, ',')) continue; z = std::stod(token);
        if (!std::getline(ss, token, ',')) continue; qx = std::stod(token);
        if (!std::getline(ss, token, ',')) continue; qy = std::stod(token);
        if (!std::getline(ss, token, ',')) continue; qz = std::stod(token);
        if (!std::getline(ss, token)) continue; qw = std::stod(token);
        out.emplace_back(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x, y, z));
    }
    return out;
}

std::vector<CrossEdge> LoadCrossPose(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("could not open pose edge file: " + path.string());
    std::vector<CrossEdge> out;
    std::string line;
    while (std::getline(in, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        CrossEdge edge;
        ss >> edge.q_idx >> edge.db_idx >> edge.overlap;
        for (int i = 0; i < 6; ++i) ss >> edge.meas(i);
        if (!ss.fail()) out.push_back(edge);
    }
    return out;
}

std::string StripTxt(std::string name) {
    if (name.size() > 4 && name.substr(name.size() - 4) == ".txt") name.resize(name.size() - 4);
    return name;
}

bool ParsePoseFilename(const fs::path& path, std::string& db_label, std::string& query_label) {
    std::string name = StripTxt(path.filename().string());
    if (name.rfind("pose_", 0) != 0) return false;
    name = name.substr(5);
    const size_t pos = name.find("_vs_");
    if (pos == std::string::npos) return false;
    db_label = name.substr(0, pos);
    query_label = name.substr(pos + 4);
    return !db_label.empty() && !query_label.empty();
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <sessions.csv> <pose_edge_dir> <output_dir>\n"
                  << "sessions.csv rows: label,slam_csv or label,key,slam_csv\n";
        return 1;
    }

    try {
        const auto sessions = LoadSessions(argv[1]);
        const fs::path edge_dir = argv[2];
        const fs::path output_dir = argv[3];
        fs::create_directories(output_dir);

        std::unordered_map<std::string, size_t> session_by_label;
        for (size_t i = 0; i < sessions.size(); ++i) session_by_label[sessions[i].label] = i;

        std::vector<std::vector<gtsam::Pose3>> poses(sessions.size());
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;

        auto anchor_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(1e-6));
        auto loose_prior = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(10.0));
        auto odo_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished());
        auto cross_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(0.5)));

        for (size_t i = 0; i < sessions.size(); ++i) {
            poses[i] = LoadSlamCsv(sessions[i].slam_csv);
            for (size_t j = 0; j < poses[i].size(); ++j) {
                gtsam::Symbol key(sessions[i].key, j);
                initial.insert(key, poses[i][j]);
                if (j == 0) {
                    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                        key, poses[i][j], i == 0 ? anchor_noise : loose_prior));
                } else {
                    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                        gtsam::Symbol(sessions[i].key, j - 1),
                        key,
                        poses[i][j - 1].between(poses[i][j]),
                        odo_noise));
                }
            }
            std::cout << "[LOAD] " << sessions[i].label << " poses=" << poses[i].size() << "\n";
        }

        size_t cross_edges = 0;
        for (const auto& entry : fs::directory_iterator(edge_dir)) {
            if (!entry.is_regular_file()) continue;
            std::string db_label;
            std::string query_label;
            if (!ParsePoseFilename(entry.path(), db_label, query_label)) continue;
            const auto db_it = session_by_label.find(db_label);
            const auto q_it = session_by_label.find(query_label);
            if (db_it == session_by_label.end() || q_it == session_by_label.end()) continue;
            const size_t db = db_it->second;
            const size_t q = q_it->second;
            size_t added = 0;
            for (const auto& edge : LoadCrossPose(entry.path())) {
                if (edge.overlap < 0.2) continue;
                if (edge.q_idx >= poses[q].size() || edge.db_idx >= poses[db].size()) continue;
                const gtsam::Pose3 rel(
                    gtsam::Rot3::RzRyRx(edge.meas(3), edge.meas(4), edge.meas(5)),
                    gtsam::Point3(edge.meas(0), edge.meas(1), edge.meas(2)));
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    gtsam::Symbol(sessions[q].key, edge.q_idx),
                    gtsam::Symbol(sessions[db].key, edge.db_idx),
                    rel,
                    cross_noise));
                ++added;
            }
            cross_edges += added;
            if (added > 0) {
                std::cout << "[CROSS] " << db_label << "_vs_" << query_label
                          << " edges=" << added << "\n";
            }
        }

        std::cout << "[SUMMARY] graph_factors=" << graph.size()
                  << " variables=" << initial.size()
                  << " cross_edges=" << cross_edges << "\n";
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("TERMINATION");
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimizeSafely();

        for (size_t i = 0; i < sessions.size(); ++i) {
            if (poses[i].empty()) continue;
            const fs::path out_path = output_dir / ("optimized_" + sessions[i].label + ".txt");
            std::ofstream out(out_path);
            out << std::fixed << std::setprecision(9);
            for (size_t j = 0; j < poses[i].size(); ++j) {
                gtsam::Symbol key(sessions[i].key, j);
                if (!result.exists(key)) continue;
                const gtsam::Pose3 pose = result.at<gtsam::Pose3>(key);
                const gtsam::Point3 t = pose.translation();
                const Eigen::Quaterniond q(pose.rotation().matrix());
                out << j << " " << t.x() << " " << t.y() << " " << t.z() << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            }
            std::cout << "[SAVE] " << out_path << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
