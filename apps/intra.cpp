#include <iostream>
#include <exception>

#include "treelocpp/eval.h"

int main(int argc, char** argv) {
    treelocpp::Config config;
    std::string error;
    const std::filesystem::path config_path =
        argc > 1 ? argv[1] : treelocpp::DefaultConfigPath("intra");
    if (!treelocpp::LoadConfig(config_path, config, &error)) {
        std::cerr << error << "\n";
        return 1;
    }
    config.mode = "intra";
    try {
        const auto summary = treelocpp::RunIntraSession(config);
        treelocpp::PrintSummary(summary, std::cout);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
