#include "calibtools.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <filesystem>

bool seg_plane(
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  Eigen::Vector3f axis,
  double angle_tolerance,
  bool buseangle,
  double distance_threshold,
  Eigen::Vector4d& coef,
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane) {
  //
  if (frame->points.size() <= 0) return false;

  //
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(distance_threshold);
  if (buseangle) {
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(axis);
    seg.setEpsAngle(angle_tolerance * 3.141592653 / 180.0f);
  } else
    seg.setModelType(pcl::SACMODEL_PLANE);

  // seg
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setInputCloud(frame);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() > 0) {
    // 提取初步的平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpplane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(frame);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*tmpplane);
    extract.setNegative(true);
    extract.filter(*frame);

    if (tmpplane->points.size() <= 0) return false;

    // 删除离群点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(tmpplane);  // 设置输入
    sor.setMeanK(50);             // 设置用于平均距离估计的 KD-tree最近邻搜索点的个数.
    sor.setStddevMulThresh(1.0);  // 高斯分布标准差的倍数, 也就是 u+1*sigma,u+2*sigma,u+3*sigma 中的 倍数1、2、3
    sor.filter(*tmpplane);        // 滤波后输出

    if (tmpplane->points.size() <= 0) return false;

    // 提取cube
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(tmpplane);
    feature_extractor.compute();
    // 聲明一些必要的變量
    //  std::vector <float> moment_of_inertia;
    //  std::vector <float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    // 計算描述符和其他的特徵
    //  feature_extractor.getMomentOfInertia(moment_of_inertia);
    //  feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    // 计算包围盒的长、宽和高
    float length = max_point_OBB.x - min_point_OBB.x;
    float width = max_point_OBB.y - min_point_OBB.y;
    float height = max_point_OBB.z - min_point_OBB.z;

    // // 输出最小包围盒的长、宽和高
    // std::cout << "Length: " << length << std::endl;
    // std::cout << "Width: " << width << std::endl;
    // std::cout << "Height: " << height << std::endl;

    Eigen::Vector3f normal = major_vector.cross(middle_vector);
    normal.normalize();
    coef(0) = normal(0);
    coef(1) = normal(1);
    coef(2) = normal(2);
    coef(3) = -1 * normal.dot(mass_center);

    // 固定法向量方向
    //                pcl::PointXYZ p1 = tmpplane->points[0];
    Eigen::Vector3d vector1(0 - mass_center(0), 0 - mass_center(1), 5 - mass_center(2));
    Eigen::Vector3d vector2(coef(0), coef(1), coef(2));
    double c = 1.0;
    if (vector1.dot(vector2) < 0) c = -1;
    coef(0) *= c;
    coef(1) *= c;
    coef(2) *= c;
    coef(3) *= c;

    // 将点投影到求得的平面上
    plane->clear();
    for (auto point : tmpplane->points) {
      Eigen::Vector3f normal(coef(0), coef(1), coef(2));
      Eigen::Vector3f tmp(point.x, point.y, point.z);
      double scale = (tmp - mass_center).dot(normal);
      tmp = tmp - scale * normal;
      Eigen::Vector3f normalized_v = normal.normalized();
      pcl::PointXYZ newpoint;
      double offset_coef = 0.0;
      newpoint.x = tmp(0) + normalized_v(0) * offset_coef * height;  // TODO
      newpoint.y = tmp(1) + normalized_v(1) * offset_coef * height;
      newpoint.z = tmp(2) + normalized_v(2) * offset_coef * height;
      plane->points.push_back(newpoint);
    }
    plane->height = 1;
    plane->width = tmpplane->points.size();

    return true;
  } else
    return false;
}

bool extract_ground_plane(
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  double distance_threshold,
  Eigen::Vector4d& coef,
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane,
  std::string field_name,
  double limit_min,
  double limit_max) {
  //
  if (frame->points.size() <= 0) return false;

  // pass filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(frame);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(limit_min, limit_max);
  pass.setFilterLimitsNegative(false);
  pass.filter(*frame);
  if (frame->points.size() <= 0) return false;

  //
  return seg_plane(frame, Eigen::Vector3f(0, 0, 1), 2.0, true, distance_threshold, coef, plane);
}

void extract_all_planes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  std::vector<pcl::PointIndices>& seg_indices,
  std::vector<Eigen::Vector4d>& coefs,
  const std::string& save_dir,
  bool b_visualize) {
  //
  std::string final_save_dir = save_dir + "/planes/";
  std::filesystem::create_directories(final_save_dir);

  // compute_normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(40);
  // norm_est.setRadiusSearch(5);
  norm_est.setInputCloud(frame);
  norm_est.compute(*normals);

  // plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indices_plane(new pcl::PointIndices);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.2);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(3000);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(frame);
  seg.setInputNormals(normals);
  seg.segment(*indices_plane, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract(true);
  extract.setInputCloud(frame);
  pcl::PointIndices::Ptr indices_notplane(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  int plane_size = indices_plane->indices.size();
  pcl::PointIndices::Ptr indices_plane_all(new pcl::PointIndices);

  int cnt = 0;
  while (plane_size >= 200) {
    std::cout << "Plane points: " << plane_size << std::endl;

    Eigen::Vector4d coef;
    coef << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];
    coefs.emplace_back(coef);
    extract.setIndices(indices_plane);
    extract.filter(*cloud_out);
    pcl::io::savePCDFileBinary(final_save_dir + std::to_string(cnt++) + ".pcd", *cloud_out);

    if (b_visualize) {
      pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Plane" + std::to_string(cnt - 1)));

      // 设置点云颜色为绿色
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(frame, 0, 255, 0);  // 绿色 RGB(0, 255, 0)
      std::string frame_name = "frame";
      viewer->addPointCloud(frame, color_handler, frame_name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, frame_name);  // 设置点云的尺寸为10

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler1(cloud_out, 255, 0, 0);
      std::string plane_name = "plane";  // 绿色 RGB(0, 255, 0)
      viewer->addPointCloud(cloud_out, color_handler1, plane_name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, plane_name);  // 设置点云的尺寸为10

      // 可视化点云
      while (!viewer->wasStopped()) {
        viewer->spin();
      }
    }

    seg_indices.push_back(*indices_plane);
    indices_plane_all->indices.insert(indices_plane_all->indices.end(), indices_plane->indices.begin(), indices_plane->indices.end());
    extract.setIndices(indices_plane_all);
    extract.filter(*cloud_out);
    extract.getRemovedIndices(*indices_notplane);
    seg.setIndices(indices_notplane);
    seg.segment(*indices_plane, *coefficients);
    plane_size = indices_plane->indices.size();
  }
  std::cout << "Plane points < 200, stop extracting plane." << std::endl;
}
