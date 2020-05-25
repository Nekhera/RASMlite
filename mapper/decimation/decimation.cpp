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

void decimate(PointCloud<PointXYZ>::Ptr cloud) {}
void slimToSize(PointCloud<PointXYZ>::Ptr cloud) {}
void triangulate(PointCloud<PointXYZ>::Ptr cloud) {}

void pointcloudCallback(const PointCloud<PointXYZ>::ConstPtr &cloud) {
  PointCloud<PointXYZ>::Ptr cloud_converted(new PointCloud<PointXYZ>(*cloud));
  PointCloud<PointXYZ>::Ptr cloud_transformed = frameTransform(cloud_converted);

  downSample(cloud_transformed, 0.025, 0.025, 0.025);
  cutOff(cloud_transformed, edgeDistance);
  
  decimate(cloud_transformed);
  slimToSize(cloud_transformed);
  triangulate(cloud_transformed);
}
