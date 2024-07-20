#include <iostream>

#include <filesystem>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>

#include "calibtools.h"

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("three_plane_points");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("pcd_dir", value<std::string>(), "directory of pcd files")
    ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on

  positional_options_description p;
  p.add("pcd_dir", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("pcd_dir")) {
    std::cout << description << std::endl;
    return true;
  }

  const std::string pcd_dir = vm["pcd_dir"].as<std::string>();
  std::cout << "pcd_dir: " << pcd_dir << std::endl;
  // std::filesystem::create_directories(dst_dir);

  std::vector<std::string> pcd_paths;
  for (const auto& path : std::filesystem::directory_iterator(pcd_dir)) {
    std::filesystem::path extension = path.path().extension();
    if (extension == ".pcd") {
      pcd_paths.emplace_back(path.path().string());
    }
  }

  // extract plane
  for (const auto& pcd_path : pcd_paths) {
    std::cout << "processing " << pcd_path << std::endl;

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(pcd_path, *frame);

    //
    Eigen::Vector4d ground_coef;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>());
    if (!extract_ground_plane(frame, 1, ground_coef, ground_plane)) {
      std::cout << "ground plane extraction failed" << std::endl;
      continue;
    }
    pcl::io::savePCDFileBinary(pcd_dir + "/ground_plane.pcd", *ground_plane);

    //
    std::vector<Eigen::Vector4d> x_coefs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> x_planes;
    while (true) {
      Eigen::Vector4d coef;
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
      if (!seg_plane(frame, Eigen::Vector3f(1, 0, 0), 10.0, true, 5, coef, plane)) {
        break;
      }
      x_coefs.emplace_back(coef);
      x_planes.emplace_back(plane);
    }

    if (x_coefs.empty()) {
      std::cout << "x plane extraction failed" << std::endl;
      std::cout << "" << std::endl;
      continue;
    }
    //
    std::vector<Eigen::Vector4d> y_coefs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> y_planes;
    while (true) {
      Eigen::Vector4d coef;
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
      if (!seg_plane(frame, Eigen::Vector3f(0, 1, 0), 10.0, true, 5, coef, plane)) {
        break;
      }
      y_coefs.emplace_back(coef);
      y_planes.emplace_back(plane);
    }
    if (y_coefs.empty()) {
      std::cout << "y plane extraction failed" << std::endl;
      std::cout << "" << std::endl;
      continue;
    }

    std::cout << "x_planes:" << x_planes.size() << std::endl;
    std::cout << "y_planes:" << y_planes.size() << std::endl;
    std::cout << "" << std::endl;
  }

  return 0;
}
