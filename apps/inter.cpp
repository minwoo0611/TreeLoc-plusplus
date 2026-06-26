#include <iostream>
#include <exception>

#include "treelocpp/eval.h"

int main(int argc, char** argv) {
    treelocpp::Config config;
    std::string error;
    const std::filesystem::path config_path =
        argc > 1 ? argv[1] : treelocpp::DefaultConfigPath("inter");
    if (!treelocpp::LoadConfig(config_path, config, &error)) {
        std::cerr << error << "\n";
        return 1;
    }
    config.mode = "inter";
    try {
        const auto summary = treelocpp::RunInterSession(config);
        treelocpp::PrintSummary(summary, std::cout);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
