#include "pcl/point_types.h"
#include "pcl/features/moment_of_inertia_estimation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/approximate_voxel_grid.h>

bool seg_plane(
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  Eigen::Vector3f axis,
  double angle_tolerance,
  bool buseangle,
  double distance_threshold,
  Eigen::Vector4d& coef,
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane);

bool extract_ground_plane(
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  double distance_threshold,
  Eigen::Vector4d& coef,
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane,
  std::string field_name = "z",
  double limit_min = -100.0,
  double limit_max = 0.0);

void extract_all_planes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  std::vector<pcl::PointIndices>& seg_indices,
  std::vector<Eigen::Vector4d>& coefs,
  const std::string& save_dir,
  bool b_visualize=false);