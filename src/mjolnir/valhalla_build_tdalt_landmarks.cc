#include "argparse_utils.h"
#include "mjolnir/tdalt_landmarks_builder.h"

#include <cxxopts.hpp>

#include <filesystem>

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "valhalla_build_tdalt_landmarks builds the TDALT ALT landmark distance sidecar used by\n"
      "time-dependent bidirectional routing. This is separate from the POI landmarks.sqlite index.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config)) {
      return EXIT_SUCCESS;
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  try {
    valhalla::mjolnir::build_tdalt_landmarks(config);
  } catch (std::exception& e) {
    std::cerr << "TDALT landmark build failed: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
