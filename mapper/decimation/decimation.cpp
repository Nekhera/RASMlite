using namespace pcl;

static const float CAMERA_ANGLE;
static const float CAMERA_HEIGHT;

PointCloud<PointXYZ>::Ptr frameTransform(PointCloud<PointXYZ>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, 0.0, CAMERA_HEIGHT;
  transform_matrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
  transform_matrix.rotate(Eigen::AngleAxisf(-(M_PI / 2) - CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZ>::Ptr cloud_corrected (new PointCloud<PointXYZ>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

PointCloud<PointXYZ>::Ptr orientationCorrection(PointCloud<PointXYZ>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.rotate(Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()));
  transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZ>::Ptr cloud_corrected (new PointCloud<PointXYZ>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

void downSample(PointCloud<PointXYZ>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZ> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  //vox.setFilterFieldName("x");
  //vox.setFilterLimits(0.1, CAMERA_X_LIMIT);
  vox.filter(*cloud);
}

void cutOff(PointCloud<PointXYZ>::Ptr cloud) {
  PassThrough<PointXYZ> pt;
  pt.setInputCloud(cloud);
  pt.setFilterFieldName("y");
  //pt.setFilterLimits(-ROVER_OUTER_WIDTH / 2, ROVER_OUTER_WIDTH / 2);
  pt.filter(*cloud);
}

float calcVariance(PointCloud<PointXYZ>::Ptr cloud, Eigen::Vector4f centroid) {
  float variance = 0.0;
  for(unsigned int i = 0; i < cloud->size(); i++) {
    float num = cloud->points[i].z - centroid[2];
    variance += (num * num);
  }
  return variance / (cloud->size() - 1);
}

void decimate(PointCloud<PointXYZ>::Ptr cloud, unsigned int max_size, unsigned int max_variance) {
  PCA<PointXYZ> pca;
  pca.setInputCloud(cloud);
  pca.project(cloud, cloud);

  PointCloud<PointXYZ>::Ptr cloud_group_1(new PointCloud<PointXYZ>());
  PointCloud<PointXYZ>::Ptr cloud_group_2(new PointCloud<PointXYZ>());
  Passthrough<PointXYZ> pt;
  pt.setInputCloud(cloud);
  pt.setFilterFieldName("x");

  Eigen::Vector4f centroid;
  pt.setFilterLimits(0, FLT_MAX);
  pt.filter(*cloud_group_1);
  compute3DCentroid(*cloud_group_1, centroid);

  if (cloud_group_1->size() > max_size || calcVariance(cloud_group_1, centroid) > max_variance) {
    decimate(cloud_group_1, max_size, max_variance);
  } else {
    PointXYZ p(new PointXYZ(centroid[0], centroid[1], centroid[2]));
    cloud_group_1(new PointCloud<PointXYZ>());
    cloud_group_1->points.push_back(p);
  }

  pt.setNegative(true);
  pt.filter(*cloud_group_2);
  compute3DCentroid(*cloud_group_2, centroid);

  if (cloud_group_2->size() > max_size || calcVariance(cloud_group_2, centroid) > max_variance) {
    decimate(cloud_group_2, max_size, max_variance);
  } else {
    PointXYZ p(new PointXYZ(centroid[0], centroid[1], centroid[2]));
    cloud_group_2(new PointCloud<PointXYZ>());
    cloud_group_2->points.push_back(p);
  }

  *cloud = *cloud_group_1;
  *cloud += *cloud_group_2;
}

void slimToSize(PointCloud<PointXYZ>::Ptr cloud) {}
void triangulate(PointCloud<PointXYZ>::Ptr cloud) {}

void pointcloudCallback(const PointCloud<PointXYZ>::ConstPtr &cloud) {
  PointCloud<PointXYZ>::Ptr cloud_converted(new PointCloud<PointXYZ>(*cloud));
  PointCloud<PointXYZ>::Ptr cloud_transformed = frameTransform(cloud_converted);

  downSample(cloud_transformed, 0.025, 0.025, 0.025);
  cutOff(cloud_transformed, edgeDistance);
  
  decimate(cloud_transformed, 10, 60);
  slimToSize(cloud_transformed);
  triangulate(cloud_transformed);
}
