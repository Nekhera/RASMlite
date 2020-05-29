#pragma once
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace RASMlite {
  class decimation {
    private:
      static float calcVariance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f centroid);
      
      static void frameTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      static void orientationCorrection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      static void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double x_dim, double y_dim, double z_dim);
      static void cutOff(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      static void decimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, unsigned int max_size, unsigned int max_variance);
      static void slimToSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      static void triangulate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      
    public:
      static void process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      
  };
}
