#include "groundplanfit_hesai.h"

pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(VPoint a, VPoint b) { return a.z < b.z; }

GroundPlaneFit::GroundPlaneFit() : node_handle_("~") {
  // Init ROS related
  ROS_INFO("Initializing Ground Plane Fitter...");
  node_handle_.param<std::string>("point_topic", point_topic_,
                                  "/lidar_on_roof");
  ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());

  node_handle_.param("half_road_width", half_road_width_, 17.0);
  ROS_INFO("Road Width: %f", half_road_width_ * 2);

  node_handle_.param("sensor_model", sensor_model_, 64);
  ROS_INFO("Sensor Model: %d", sensor_model_);

  node_handle_.param("sensor_height", sensor_height_, 2.1);
  ROS_INFO("Sensor Height: %f", sensor_height_);

  node_handle_.param("num_seg", num_seg_, 10);
  ROS_INFO("Num of Segments Forward: %d", num_seg_);

  node_handle_.param("num_seg_y", num_seg_y_, 3);
  ROS_INFO("Num of Segments Left-right: %d", num_seg_y_);

  node_handle_.param("num_iter", num_iter_, 3);
  ROS_INFO("Num of Iteration: %d", num_iter_);

  node_handle_.param("num_lpr", num_lpr_, 20);
  ROS_INFO("Num of LPR: %d", num_lpr_);

  node_handle_.param("th_seeds", th_seeds_, 0.4);
  ROS_INFO("Seeds Threshold: %f", th_seeds_);

  node_handle_.param("th_dist", th_dist_, 0.2);
  ROS_INFO("Distance Threshold: %f", th_dist_);

  // Listen to velodyne topic
  points_node_sub_ = node_handle_.subscribe(
      point_topic_, 1, &GroundPlaneFit::rslidar_callback_, this);

  // Publish Init
  std::string no_ground_topic, ground_topic;
  node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic,
                                  "/points_no_ground");
  ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
  node_handle_.param<std::string>("ground_point_topic", ground_topic,
                                  "/points_ground");
  ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());
  groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      no_ground_topic, 1, true);
  ground_points_pub_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1, true);
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.

*/
void GroundPlaneFit::estimate_plane_(void) {
  // Create covarian matrix.
  // 1. calculate (x,y,z) mean
  float x_mean = 0, y_mean = 0, z_mean = 0;
  for (size_t i = 0; i < g_ground_pc->points.size(); i++) {
    x_mean += g_ground_pc->points[i].x;
    y_mean += g_ground_pc->points[i].y;
    z_mean += g_ground_pc->points[i].z;
  }
  // incase of divide zero
  int size = g_ground_pc->points.size() != 0 ? g_ground_pc->points.size() : 1;
  x_mean /= size;
  y_mean /= size;
  z_mean /= size;
  // 2. calculate covariance
  // cov(x,x), cov(y,y), cov(z,z)
  // cov(x,y), cov(x,z), cov(y,z)
  float xx = 0, yy = 0, zz = 0;
  float xy = 0, xz = 0, yz = 0;
  for (size_t i = 0; i < g_ground_pc->points.size(); i++) {
    xx += (g_ground_pc->points[i].x - x_mean) *
          (g_ground_pc->points[i].x - x_mean);
    xy += (g_ground_pc->points[i].x - x_mean) *
          (g_ground_pc->points[i].y - y_mean);
    xz += (g_ground_pc->points[i].x - x_mean) *
          (g_ground_pc->points[i].z - z_mean);
    yy += (g_ground_pc->points[i].y - y_mean) *
          (g_ground_pc->points[i].y - y_mean);
    yz += (g_ground_pc->points[i].y - y_mean) *
          (g_ground_pc->points[i].z - z_mean);
    zz += (g_ground_pc->points[i].z - z_mean) *
          (g_ground_pc->points[i].z - z_mean);
  }
  // 3. setup covarian matrix cov
  MatrixXf cov(3, 3);
  cov << xx, xy, xz, xy, yy, yz, xz, yz, zz;
  cov /= size;
  // Singular Value Decomposition: SVD
  JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  MatrixXf seeds_mean(3, 1);
  seeds_mean << x_mean, y_mean, z_mean;
  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = th_dist_ - d_;

  // return the equation parameters
}

/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud

    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::

*/
void GroundPlaneFit::extract_initial_seeds_(
  const pcl::PointCloud<VPoint> &p_sorted) {
  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;
  // Calculate the mean height value.
  for (size_t i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
  g_seeds_pc->clear();
  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (size_t i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seeds_) {
      g_seeds_pc->points.push_back(p_sorted.points[i]);
    }
  }
  // return seeds points
}

void GroundPlaneFit::grid_map_filter_(const pcl::PointCloud<VPoint>::ConstPtr in_cloud_ptr){
  grid_pts_.resize(LOCAL_GRID_LENGTH);
  for (int j = 0; j < LOCAL_GRID_LENGTH; ++j) {
    grid_pts_[j].resize(LOCAL_GRID_WIDTH);
  }

  for (int j = 0; j < LOCAL_GRID_LENGTH; ++j){
    for (int i = 0; i < LOCAL_GRID_WIDTH; ++i){
      grid_pts_[j][i].clear();
    }
  }
  temp_ground_ptr_.reset(new pcl::PointCloud<VPoint>);
  temp_no_ground_ptr_.reset(new pcl::PointCloud<VPoint>);

  temp_ground_ptr_->clear();
  temp_no_ground_ptr_->clear();
  VPoint tmp_pt;
  for (size_t i = 0; i < in_cloud_ptr->size(); ++i){
    tmp_pt = in_cloud_ptr->points[i];       
    if (fabs(tmp_pt.x) * 100 < (FRONT_SCAN_RANGE - 0.0001) && tmp_pt.y * 100 <( LEFT_SCAN_RANGE - 0.0001) && tmp_pt.y * 100 > (-RIGHT_SCAN_RANGE + 0.0001)) {
      int GridI = (FRONT_SCAN_RANGE - tmp_pt.x * 100) /
                  GRID_RESOLUTION; //第几行
      int GridJ = (LEFT_SCAN_RANGE - tmp_pt.y * 100) /
                  GRID_RESOLUTION; //第几列
      grid_pts_[GridI][GridJ].push_back(tmp_pt);
    }
  }  

  for (int j = 0; j < LOCAL_GRID_LENGTH; ++j){
    for (int i = 0; i < LOCAL_GRID_WIDTH; ++i){
      if (grid_pts_[j][i].empty())
          continue;
      sort(grid_pts_[j][i].begin(), grid_pts_[j][i].end(), point_cmp);

      std::vector<VPoint>::iterator max = grid_pts_[j][i].end() - 1;
      std::vector<VPoint>::iterator min = grid_pts_[j][i].begin();

      float Height_dif = max->z - min->z;
      if (Height_dif < GRID_HIGHTTHRESHOLD){
        for (size_t k = 0; k < grid_pts_[j][i].size(); ++k){
          temp_ground_ptr_->push_back(grid_pts_[j][i][k]);
        }
      }
      else{
        for (size_t k = 0; k < grid_pts_[j][i].size(); ++k){
          temp_no_ground_ptr_->push_back(grid_pts_[j][i][k]);
        }
      }
    }
  }
}

void VPoint2pclPointCloud(const pcl::PointCloud<VPoint> &cloud_in,
                          pcl::PointCloud<pcl::PointXYZI> &cloud_out){
    cloud_out.resize(cloud_in.size());
    for(size_t i = 0;i<cloud_in.size();++i){
        cloud_out[i].x = cloud_in[i].x;
        cloud_out[i].y = cloud_in[i].y;
        cloud_out[i].z = cloud_in[i].z;
        cloud_out[i].intensity = cloud_in[i].ring;
    }
}

/*
    @brief pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void GroundPlaneFit::rslidar_callback_(
    const sensor_msgs::PointCloud2ConstPtr &in_cloud_msg) {
  clock_t start, end;
  start = clock();
  // 1.Msg to pointcloud
  PPointCloud rawCloudIn;
  pcl::PointCloud<VPoint>::Ptr laserCloudIn(new pcl::PointCloud<VPoint>);

  pcl::fromROSMsg(*in_cloud_msg, rawCloudIn);
  VPoint point;

  for (size_t i = 0; i < rawCloudIn.points.size(); i++) {
    point.x = rawCloudIn.points[i].x;
    point.y = rawCloudIn.points[i].y;
    point.z = rawCloudIn.points[i].z;
    point.intensity = rawCloudIn.points[i].intensity;
    point.ring = rawCloudIn.points[i].ring;

    if (i == 0 || i == rawCloudIn.points.size() - 1){
      point.pitch_angle = 0;
    }
    else{
      float xx1,yy1,xx2,yy2;
      if(point.ring >= 60){
        xx1 = rawCloudIn.points[i+2].x - rawCloudIn.points[i-2].x;
        yy1 = rawCloudIn.points[i+2].y - rawCloudIn.points[i-2].y;
      }
      else{
        xx1 = rawCloudIn.points[i+1].x - rawCloudIn.points[i-1].x;
        yy1 = rawCloudIn.points[i+1].y - rawCloudIn.points[i-1].y;
      }
      xx2 = rawCloudIn.points[i].x;
      yy2 = rawCloudIn.points[i].y;
      point.pitch_angle = abs(xx1 * xx2 + yy1 * yy2)/(sqrt(xx1 * xx1 + yy1 * yy1)*sqrt(xx2 * xx2 + yy2 * yy2));

      float xx3 = rawCloudIn.points[i].x - rawCloudIn.points[i-1].x;
      float yy3 = rawCloudIn.points[i].y - rawCloudIn.points[i-1].y;
      point.direction = atan2(yy3,xx3);
    }
    if (point.x > -29.0 && point.x < 29.0 && point.y > -10.0 && point.y < 17.0 && point.z > -1.2 * sensor_height_ && point.z < 0.1) {
      laserCloudIn->points.push_back(point);
    }
  }

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

  pcl::PointCloud<VPoint>::Ptr laserCloudOut(new pcl::PointCloud<VPoint>);
  pcl::ExtractIndices<VPoint> cliper;
  cliper.setInputCloud(laserCloudIn);
  pcl::PointIndices indices1;

  for (size_t i = 0; i < laserCloudIn->points.size(); i++) {
    if (laserCloudIn->points[i].x > -2 && laserCloudIn->points[i].x < 2.5 &&
        abs(laserCloudIn->points[i].y) < 1) {
      indices1.indices.push_back(i);
    }
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices1));
  cliper.setNegative(true); // ture to remove the indices1
  cliper.filter(*laserCloudOut);

  grid_map_filter_(laserCloudOut);

  // 2.Sort on Z-axis value.
  sort(temp_ground_ptr_->points.begin(), temp_ground_ptr_->points.end(), point_cmp);

  pcl::PointCloud<VPoint> not_ground_pc;
  pcl::PointCloud<VPoint> ground_pc;

  float seg_step = 60.0 / num_seg_;
  float seg_step_y = 27.0 / num_seg_y_;
  for (int seg_x = 0; seg_x < num_seg_; seg_x++) {
    for (int seg_y = 0; seg_y < num_seg_y_; seg_y++) {
      pcl::PointCloud<VPoint>::Ptr seg_laserCloudIn(
          new pcl::PointCloud<VPoint>);
      pcl::PassThrough<VPoint> seg_pass;
      seg_pass.setInputCloud(temp_ground_ptr_);
      seg_pass.setFilterFieldName("x");
      seg_pass.setFilterLimits(-30.0 + seg_step * seg_x,
                               -30.0 + seg_step * (seg_x + 1));
      seg_pass.filter(*seg_laserCloudIn);

      seg_pass.setInputCloud(seg_laserCloudIn);
      seg_pass.setFilterFieldName("y");
      seg_pass.setFilterLimits(-10.0 + seg_step_y * seg_y,
                               -10.0 + seg_step_y * (seg_y + 1));
      seg_pass.filter(*seg_laserCloudIn);
      // 4. Extract init ground seeds.
      extract_initial_seeds_(*seg_laserCloudIn);
      g_ground_pc = g_seeds_pc;

      // 5. Ground plane fitter mainloop
      for (int i = 0; i < num_iter_; i++) {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        // pointcloud to matrix
        MatrixXf points(seg_laserCloudIn->points.size(), 3);
        int j = 0;
        for (auto p : seg_laserCloudIn->points) {
          points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++) {
          if (result[r] < th_dist_d_) {
            g_ground_pc->points.push_back(seg_laserCloudIn->points[r]);
          } else {
            g_not_ground_pc->points.push_back(seg_laserCloudIn->points[r]);
          }
        }
      }
      ground_pc += *g_ground_pc;
      not_ground_pc += *g_not_ground_pc;
    }
  }
  not_ground_pc += *temp_no_ground_ptr_;

  // publish ground points
  sensor_msgs::PointCloud2 ground_msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  VPoint2pclPointCloud(ground_pc,*ground_cloud_ptr);
  pcl::toROSMsg(*ground_cloud_ptr, ground_msg);
  ground_msg.header.stamp = in_cloud_msg->header.stamp;
  ground_msg.header.frame_id = in_cloud_msg->header.frame_id;
  ground_points_pub_.publish(ground_msg);
  // publish not ground points
  sensor_msgs::PointCloud2 groundless_msg;

  pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  VPoint2pclPointCloud(not_ground_pc,*not_ground_cloud_ptr);
  pcl::toROSMsg(*not_ground_cloud_ptr, groundless_msg);
  groundless_msg.header.stamp = in_cloud_msg->header.stamp;
  groundless_msg.header.frame_id = in_cloud_msg->header.frame_id;
  groundless_points_pub_.publish(groundless_msg);

  end = clock();
  float runTime = ((float)(end - start)) / CLOCKS_PER_SEC;
 // ROS_INFO("grond palne fit: %f seconds\n", runTime);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "GroundPlaneFit");
  GroundPlaneFit node;
  ros::spin();

  return 0;
}
