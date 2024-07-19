#include <iostream>

#include <filesystem>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
int main(int argc, char ** argv)
{
  using namespace boost::program_options;
  options_description description("three_plane_points");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("pcd_dir", value<std::string>(), "directory of pcd files")
    ("dst_dir", value<std::string>(), "directory to save preprocessed data")
    ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on

  positional_options_description p;
  p.add("pcd_dir", 1);
  p.add("dst_dir", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("pcd_dir") || !vm.count("dst_dir")) {
    std::cout << description << std::endl;
    return true;
  }


  const std::string pcd_dir = vm["pcd_dir"].as<std::string>();
  const std::string dst_dir = vm["dst_dir"].as<std::string>();
  std::cout << "pcd_dir: " << pcd_dir << std::endl;
  std::cout << "dst_dir : " << dst_dir << std::endl;
  std::filesystem::create_directories(dst_dir);

  std::vector<std::string> pcd_paths;
  for (const auto& path : std::filesystem::directory_iterator(pcd_dir)) {
    std::filesystem::path extension = path.path().extension();
    if (extension == ".pcd") {
      pcd_paths.emplace_back(path.path().string());
    }
  }

  // TODO to finish detect points

  return 0;
}
