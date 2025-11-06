#include <opencv2/core.hpp>
#include <opencv2/surface_matching/ppf_match_3d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <fstream>
#include <iostream>

int main()
{
  // Initialize PPF 3D detector
  cv::ppf_match_3d::PPF3DDetector detector(0.05, 0.05, 30);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

  if (pcl::io::loadPLYFile<pcl::PointNormal>("Fused.ply", *cloud) == -1)
  {
    std::cerr << "Failed to load PLY file." << std::endl;
    return -1;
  }

  cv::Mat model_mat(static_cast<int>(cloud->size()), 6, CV_32F);
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto &pt = (*cloud)[i];
    model_mat.at<float>(i, 0) = pt.x;
    model_mat.at<float>(i, 1) = pt.y;
    model_mat.at<float>(i, 2) = pt.z;
    model_mat.at<float>(i, 3) = pt.normal_x;
    model_mat.at<float>(i, 4) = pt.normal_y;
    model_mat.at<float>(i, 5) = pt.normal_z;
  }

  detector.trainModel(model_mat);
  std::cout << "Model trained with " << cloud->size() << " points." << std::endl;
}