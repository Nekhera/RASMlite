#include "decimation.h"

#include <vector>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace RASMlite {
  static const float CAMERA_ANGLE = 0.0;
  static const float CAMERA_HEIGHT = 0.0;
  int pitch, roll;

  void decimation::frameTransform(PointCloud<PointXYZ>::Ptr cloud) {
    Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
    transform_matrix.translation() << 0.0, 0.0, CAMERA_HEIGHT;
    transform_matrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
    transform_matrix.rotate(Eigen::AngleAxisf(-(M_PI / 2) - CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

    transformPointCloud(*cloud, *cloud, transform_matrix);
  }

  void decimation::orientationCorrection(PointCloud<PointXYZ>::Ptr cloud) {
    Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
    transform_matrix.rotate(Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()));
    transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    transformPointCloud(*cloud, *cloud, transform_matrix);
  }

  void decimation::downSample(PointCloud<PointXYZ>::Ptr cloud, double x, double y, double z) {
    VoxelGrid<PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(x, y, z);
    //vox.setFilterFieldName("x");
    //vox.setFilterLimits(0.1, CAMERA_X_LIMIT);
    vox.filter(*cloud);
  }

  void decimation::cutOff(PointCloud<PointXYZ>::Ptr cloud) {
    PassThrough<PointXYZ> pt;
    pt.setInputCloud(cloud);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-5, 5);
    pt.filter(*cloud);
  }

  float decimation::calcVariance(PointCloud<PointXYZ>::Ptr cloud, Eigen::Vector4f centroid) {
    float variance = 0.0;
    for(unsigned int i = 0; i < cloud->size(); i++) {
      float num = cloud->points[i].z - centroid[2];
      variance += (num * num);
    }
    return variance / (cloud->size() - 1);
  }

  void decimation::decimate(PointCloud<PointXYZ>::Ptr cloud, uint32_t m_size, uint32_t m_var) {
    PCA<PointXYZ> pca;
    pca.setInputCloud(cloud);
    pca.project(*cloud, *cloud);

    PointCloud<PointXYZ>::Ptr cloud_group_1(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_group_2(new PointCloud<PointXYZ>());
    
    PassThrough<PointXYZ> pt;
    pt.setInputCloud(cloud);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(0, FLT_MAX);
    pt.filter(*cloud_group_1);
    
    Eigen::Vector4f centroid;
    pca.reconstruct(*cloud_group_1, *cloud_group_1);
    compute3DCentroid(*cloud_group_1, centroid);

    if (cloud_group_1->size() > m_size || calcVariance(cloud_group_1, centroid) > m_var) {
      decimate(cloud_group_1, m_size, m_var);
    } else {
      PointXYZ p;
      p.x = centroid[0];
      p.y = centroid[1];
      p.z = centroid[2];

      cloud_group_1->clear();
      cloud_group_1->points.push_back(p);
    }

    pt.setNegative(true);
    pt.filter(*cloud_group_2);

    pca.reconstruct(*cloud_group_2, *cloud_group_2);
    compute3DCentroid(*cloud_group_2, centroid);

    if (cloud_group_2->size() > m_size || calcVariance(cloud_group_2, centroid) > m_var) {
      decimate(cloud_group_2, m_size, m_var);
    } else {
      PointXYZ p;
      p.x = centroid[0];
      p.y = centroid[1];
      p.z = centroid[2];
     
      cloud_group_2->clear();
      cloud_group_2->points.push_back(p);
    }

    *cloud = *cloud_group_1;
    *cloud += *cloud_group_2;
  }

  CT decimation::triangulate(PointCloud<PointXYZ>::Ptr cloud) {
    std::vector<Point> vec;
    for(auto p : *cloud) {
      vec.push_back(Point(p.x, p.y, p.z));
    }

    CT ct;
    ct.insert(vec.begin(), vec.end());
    PS::simplify(ct, Cost(), Stop(0.5));
    return ct; 
  }
  
  void decimation::process(PointCloud<PointXYZ>::Ptr cloud) {
    frameTransform(cloud);
    downSample(cloud, 0.025, 0.025, 0.025);
    cutOff(cloud);
    decimate(cloud, 10, 60);

    CT ct = triangulate(cloud); 
  }
}
