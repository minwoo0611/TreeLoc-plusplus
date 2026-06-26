#include "treelocpp/io.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <unordered_map>

namespace treelocpp {

namespace {

std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> cells;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    if (!line.empty() && line.back() == ',') cells.emplace_back();
    return cells;
}

double ToDouble(const std::string& value, double fallback) {
    if (value.empty()) return fallback;
    try {
        return std::stod(value);
    } catch (...) {
        return fallback;
    }
}

int ToInt(const std::string& value, int fallback) {
    if (value.empty()) return fallback;
    if (value == "True" || value == "true") return 1;
    if (value == "False" || value == "false") return 0;
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

std::string Cell(const std::vector<std::string>& cells,
                 const std::unordered_map<std::string, size_t>& columns,
                 const std::string& name) {
    auto it = columns.find(name);
    if (it == columns.end() || it->second >= cells.size()) return "";
    return cells[it->second];
}

const std::vector<std::string>& RequiredTreeCsvColumns() {
    static const std::vector<std::string> columns = [] {
        std::vector<std::string> out = {"location_x", "location_y", "location_z"};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out.push_back("axis_" + std::to_string(r) + std::to_string(c));
            }
        }
        return out;
    }();
    return columns;
}

void RequireColumn(const std::unordered_map<std::string, size_t>& columns,
                   const std::filesystem::path& path,
                   const std::string& name) {
    if (!columns.count(name)) {
        throw std::runtime_error(path.string() + " is missing required CSV column: " + name);
    }
}

double RequiredDouble(const std::vector<std::string>& cells,
                      const std::unordered_map<std::string, size_t>& columns,
                      const std::filesystem::path& path,
                      int line_number,
                      const std::string& name) {
    const double value = ToDouble(Cell(cells, columns, name), std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(value)) {
        throw std::runtime_error(path.string() + ":" + std::to_string(line_number) +
                                 " has invalid required numeric value for " + name);
    }
    return value;
}

}  // namespace

std::vector<Pose> ReadTrajectory(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::vector<Pose> poses;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        Pose p;
        ss >> p.stamp >> p.x >> p.y >> p.z >> p.qx >> p.qy >> p.qz >> p.qw;
        if (!ss.fail()) poses.push_back(p);
    }
    return poses;
}

std::vector<Tree> ReadTreeCsv(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::vector<Tree> trees;
    if (!in) return trees;

    std::string line;
    if (!std::getline(in, line)) return trees;
    const auto header = SplitCsv(line);
    std::unordered_map<std::string, size_t> columns;
    for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;

    for (const auto& required : RequiredTreeCsvColumns()) RequireColumn(columns, path, required);

    int line_number = 1;
    while (std::getline(in, line)) {
        ++line_number;
        if (line.empty()) continue;
        const auto cells = SplitCsv(line);
        Tree tree;
        tree.has_axis = true;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                tree.axis(r, c) =
                    RequiredDouble(cells, columns, path, line_number,
                                   "axis_" + std::to_string(r) + std::to_string(c));
            }
        }
        tree.x = RequiredDouble(cells, columns, path, line_number, "location_x");
        tree.y = RequiredDouble(cells, columns, path, line_number, "location_y");
        tree.z = RequiredDouble(cells, columns, path, line_number, "location_z");
        tree.dbh = ToDouble(Cell(cells, columns, "dbh"), std::numeric_limits<double>::quiet_NaN());
        tree.dbh_approximation =
            ToDouble(Cell(cells, columns, "dbh_approximation"), std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(tree.dbh) && std::isfinite(tree.dbh_approximation)) tree.dbh = tree.dbh_approximation;
        if (!std::isfinite(tree.dbh_approximation) && std::isfinite(tree.dbh)) tree.dbh_approximation = tree.dbh;
        tree.score = ToDouble(Cell(cells, columns, "score"),
                              ToDouble(Cell(cells, columns, "scores"), 1.0));
        tree.reconstructed = ToInt(Cell(cells, columns, "reconstructed"), 1);
        tree.number_clusters = ToInt(Cell(cells, columns, "number_clusters"), 3);
        if (std::isfinite(tree.dbh)) {
            trees.push_back(tree);
        }
    }
    return trees;
}

bool HasFrameCsv(const std::filesystem::path& root, int index) {
    return std::filesystem::exists(root / ("TreeManagerState_" + std::to_string(index) + ".csv"));
}

std::vector<int> DiscoverFrameIndices(const std::filesystem::path& root, int max_frames) {
    std::vector<int> indices;
    if (max_frames <= 0) {
        const std::string prefix = "TreeManagerState_";
        const std::string suffix = ".csv";
        for (const auto& entry : std::filesystem::directory_iterator(root)) {
            if (!entry.is_regular_file()) continue;
            const std::string name = entry.path().filename().string();
            if (name.rfind(prefix, 0) != 0 || name.size() <= prefix.size() + suffix.size()) continue;
            if (name.substr(name.size() - suffix.size()) != suffix) continue;
            const std::string id = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
            if (!std::all_of(id.begin(), id.end(), [](unsigned char ch) { return std::isdigit(ch); })) continue;
            indices.push_back(std::stoi(id));
        }
        std::sort(indices.begin(), indices.end());
        return indices;
    }
    for (int i = 0; i < max_frames; ++i) {
        if (HasFrameCsv(root, i)) indices.push_back(i);
    }
    return indices;
}

std::vector<std::string> TreeCsvSchemaErrors(const std::filesystem::path& root,
                                             int max_frames,
                                             const std::string& label) {
    const auto indices = DiscoverFrameIndices(root, max_frames);
    std::unordered_map<std::string, std::vector<std::string>> missing_by_column;
    std::vector<std::string> unreadable;
    std::vector<std::string> empty;

    for (int idx : indices) {
        const std::filesystem::path path = root / ("TreeManagerState_" + std::to_string(idx) + ".csv");
        const std::string name = path.filename().string();
        std::ifstream in(path);
        if (!in) {
            unreadable.push_back(name);
            continue;
        }
        std::string line;
        if (!std::getline(in, line)) {
            empty.push_back(name);
            continue;
        }
        const auto header = SplitCsv(line);
        std::unordered_map<std::string, size_t> columns;
        for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
        for (const auto& required : RequiredTreeCsvColumns()) {
            if (!columns.count(required)) missing_by_column[required].push_back(name);
        }
    }

    if (missing_by_column.empty() && unreadable.empty() && empty.empty()) return {};

    std::vector<std::string> lines;
    const std::string prefix = label.empty() ? root.string() : label + " (" + root.string() + ")";
    lines.push_back(prefix + " CSV schema errors:");
    if (!unreadable.empty()) {
        lines.push_back("  unreadable CSV files (" + std::to_string(unreadable.size()) + "):");
        for (const auto& name : unreadable) lines.push_back("    " + name);
    }
    if (!empty.empty()) {
        lines.push_back("  empty CSV files (" + std::to_string(empty.size()) + "):");
        for (const auto& name : empty) lines.push_back("    " + name);
    }
    for (const auto& required : RequiredTreeCsvColumns()) {
        const auto it = missing_by_column.find(required);
        if (it == missing_by_column.end()) continue;
        lines.push_back("  missing required column " + required + " in " +
                        std::to_string(it->second.size()) + " file(s):");
        for (const auto& name : it->second) lines.push_back("    " + name);
    }
    return lines;
}

std::string DatasetName(const std::filesystem::path& root) {
    return root.filename().empty() ? root.string() : root.filename().string();
}

}  // namespace treelocpp
