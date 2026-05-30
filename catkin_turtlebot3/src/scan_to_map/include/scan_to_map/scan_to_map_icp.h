#ifndef SCAN_TO_MAP_SCAN_TO_MAP_ICP_H
#define SCAN_TO_MAP_SCAN_TO_MAP_ICP_H

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

#include <scan_to_map/LocationInfo.h>

class ScanToMapICP
{
public:
  ScanToMapICP();

private:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  void initParams();

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  bool relocalizeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  void occupancyGridToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map_msg,
                                 PointCloudT::Ptr& cloud_msg);
  void scanToPointCloudOnMap(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                             PointCloudT::Ptr& cloud_msg);
  void pointCloudVoxelGridRemoval(PointCloudT::Ptr& cloud_msg, double leaf_size);
  void pointCloudObstacleRemoval(PointCloudT::Ptr& cloud_map_msg,
                                 PointCloudT::Ptr& cloud_msg,
                                 double distance_threshold);

  bool scanMatchWithICP(Eigen::Isometry3d& correction,
                        PointCloudT::Ptr& cloud_scan_msg,
                        PointCloudT::Ptr& cloud_map_msg);
  bool globalRelocalizationWithSACIA(Eigen::Isometry3d& trans,
                                     const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                     PointCloudT::Ptr& cloud_map_msg);
  bool reLocationWithICP(Eigen::Isometry3d& trans,
                         const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                         PointCloudT::Ptr& cloud_map_msg,
                         const Eigen::Isometry3d& robot_pose);

  bool getTransform(Eigen::Isometry3d& trans,
                    const std::string& parent_frame,
                    const std::string& child_frame,
                    const ros::Time& stamp);
  bool getOdomTransform(Eigen::Isometry3d& trans, double start_time, double end_time);
  bool get2TimeTransform(Eigen::Isometry3d& trans);

  Eigen::Isometry3d odomMsgToIsometry(const nav_msgs::Odometry& odom_msg) const;
  geometry_msgs::PoseWithCovarianceStamped isometryToPoseMsg(const Eigen::Isometry3d& iso,
                                                             const ros::Time& stamp) const;
  void rotatePointCloud(PointCloudT::Ptr& cloud_msg,
                        const Eigen::Affine3f& rotation,
                        const Eigen::Affine3f& robot_pose);
  bool isCoordinateInRange(const std::vector<double>& ranges, const Eigen::Isometry3d& coord) const;
  double latestAngularSpeed() const;
  void publishLocationInfo(bool relocation,
                           bool success,
                           double point_count,
                           double trans_dist,
                           double angle_dist,
                           double score);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber laser_scan_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber odom_subscriber_;

  ros::Publisher map_pointcloud_publisher_;
  ros::Publisher scan_pointcloud_publisher_;
  ros::Publisher removal_pointcloud_publisher_;
  ros::Publisher icp_pointcloud_publisher_;
  ros::Publisher location_publisher_;
  ros::Publisher relocate_visual_pose_publisher_;
  ros::Publisher relocate_initialpose_publisher_;
  ros::Publisher rotate_pointcloud_publisher_;
  ros::Publisher location_info_publisher_;

  void publishPointCloud(ros::Publisher& pub, const PointCloudT::Ptr& cloud,
                         const std::string& frame_id, const ros::Time& stamp);

  ros::ServiceServer relocalization_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool map_initialized_ = false;
  bool scan_initialized_ = false;
  bool odom_initialized_ = false;
  bool if_debug_ = false;
  bool save_pcd_ = false;
  bool use_tf_tree_always_ = true;

  std::mutex relocation_mutex_;
  std::condition_variable relocation_cv_;
  bool need_relocalization_ = false;
  bool relocalization_done_ = false;
  bool relocalization_result_ = false;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string lidar_frame_ = "base_scan";

  Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_to_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d match_result_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_match_result_ = Eigen::Isometry3d::Identity();

  double match_time_ = 0.0;
  double last_match_time_ = 0.0;
  double scan_time_ = 0.0;

  double angle_speed_threshold_ = 7.0;
  double age_threshold_ = 1.0;
  double angle_upper_threshold_ = 10.0;
  double angle_threshold_ = 0.01;
  double dist_threshold_ = 0.01;
  double score_threshold_max_ = 0.1;
  double point_quantity_threshold_ = 200.0;
  double maximum_iterations_ = 100.0;
  double obstacle_removal_distance_max_ = 2.0;
  double voxel_grid_leaf_size_ = 0.05;
  double variance_x_ = 0.01;
  double variance_y_ = 0.01;
  double variance_yaw_ = 0.01;
  double scan_range_max_ = 20.0;
  double scan_range_min_ = 0.3;

  double relocation_weight_score_ = 0.5;
  double relocation_weight_distance_ = 0.5;
  double relocation_weight_yaw_ = 0.5;
  double relocation_maximum_iterations_ = 80.0;
  double relocation_score_threshold_max_ = 0.15;
  double sacia_max_fitness_score_ = 10.0;
  int sacia_max_retries_ = 5;
  double sacia_max_correspondence_distance_ = 0.5;
  double sacia_max_iterations_ = 100;
  int sacia_min_sample_distance_ = 10;

  int loss_num_threshold_ = -1;
  int location_loss_num_ = 0;
  int odom_queue_length_ = 300;
  std::vector<double> location_restricted_zone_;

  mutable std::mutex odom_lock_;
  std::deque<nav_msgs::Odometry> odom_queue_;

  PointCloudT::Ptr cloud_map_;
  PointCloudT::Ptr cloud_scan_;

  std::chrono::steady_clock::time_point scan_last_end_time_;
};

#endif
