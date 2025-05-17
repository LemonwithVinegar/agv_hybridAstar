#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// using eigen lib
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

//局部栅格地图尺寸，单位m
const double FRONT_SCAN_RANGE = 3000;      //前方扫描有效范围30m
const double REAR_SCAN_RANGE = 1000;       //后方扫描有效范围10m
const double LEFT_SCAN_RANGE = 1000;		//左方扫描有效范围长10m
const double RIGHT_SCAN_RANGE = 1000;	    //右方扫描有效范围10m
 
//栅格分辨率
const int GRID_RESOLUTION = 20; //栅格分辨率,单位cm

const int LOCAL_GRID_LENGTH = (FRONT_SCAN_RANGE + REAR_SCAN_RANGE)/GRID_RESOLUTION;	   //栅格地图长度
const int LOCAL_GRID_WIDTH = (RIGHT_SCAN_RANGE + LEFT_SCAN_RANGE)/GRID_RESOLUTION;		//宽度;

const float GRID_HIGHTTHRESHOLD = 0.2; //高度差阈值,单位m

namespace rslidar_pointcloud {
/** Euclidean rslidar coordinate, including intensity、ring number、pitch_angle and direction. */
struct PointXYZIRPD {
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  float pitch_angle;              //
  float direction;                // 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace rslidar_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rslidar_pointcloud::PointXYZIRPD,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring)(float, pitch_angle, pitch_angle)(float, direction, direction))

#define VPoint rslidar_pointcloud::PointXYZIRPD

class GroundPlaneFit {
public:
  GroundPlaneFit();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber points_node_sub_;
  ros::Publisher ground_points_pub_;
  ros::Publisher groundless_points_pub_;

  std::string point_topic_;

  cv::Mat grid_map;

  std::vector<std::vector<std::vector<VPoint> > > grid_pts_;   //每个栅格中三维点集合
  pcl::PointCloud<VPoint>::Ptr temp_no_ground_ptr_;
  pcl::PointCloud<VPoint>::Ptr temp_ground_ptr_;

  int sensor_model_;
  double sensor_height_;
  int num_seg_;
  int num_seg_y_;
  int num_iter_;
  int num_lpr_;
  double th_seeds_;
  double th_dist_;
  double half_road_width_;

  void rslidar_callback_(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
  void grid_map_filter_(const pcl::PointCloud<VPoint>::ConstPtr in_cloud_ptr);
  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);

  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_;
  MatrixXf normal_;
  float th_dist_d_;
};
