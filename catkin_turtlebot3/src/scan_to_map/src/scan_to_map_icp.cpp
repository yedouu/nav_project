#include <scan_to_map/scan_to_map_icp.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

ScanToMapICP::ScanToMapICP() : pnh_("~"), tf_listener_(tf_buffer_)
{
  ROS_INFO_STREAM("scan_to_map_icp_node started");

  initParams();

  laser_scan_subscriber_ = nh_.subscribe("/scan", 1, &ScanToMapICP::scanCallback, this,
                                         ros::TransportHints().tcpNoDelay());
  map_subscriber_ = nh_.subscribe("/map", 1, &ScanToMapICP::mapCallback, this);
  odom_subscriber_ = nh_.subscribe("/odom", 50, &ScanToMapICP::odomCallback, this,
                                   ros::TransportHints().tcpNoDelay());

  map_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1, true);
  scan_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_pointcloud", 1);
  removal_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("removal_pointcloud", 1);
  icp_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("icp_pointcloud", 1);
  rotate_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("rotate_pointcloud", 1);
  location_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/location_match", 1);
  relocate_visual_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("relocate_visual_pose", 1);
  relocate_initialpose_publisher_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
  location_info_publisher_ = nh_.advertise<scan_to_map::LocationInfo>("/location_info", 1);

  relocalization_srv_ =
      nh_.advertiseService("/relocalization", &ScanToMapICP::relocalizeCallback, this);

  cloud_map_.reset(new PointCloudT);
  cloud_scan_.reset(new PointCloudT);
  scan_last_end_time_ = std::chrono::steady_clock::now();
}

void ScanToMapICP::initParams()
{
  pnh_.param("if_debug", if_debug_, false);
  pnh_.param("save_pcd", save_pcd_, false);
  pnh_.param("Use_TfTree_Always", use_tf_tree_always_, true);

  pnh_.param("odom_frame", odom_frame_, std::string("odom"));
  pnh_.param("base_frame", base_frame_, std::string("base_link"));
  pnh_.param("map_frame", map_frame_, std::string("map"));
  pnh_.param("lidar_frame", lidar_frame_, std::string("base_scan"));

  pnh_.param("odom_queue_length", odom_queue_length_, 300);
  pnh_.param("ANGLE_SPEED_THRESHOLD", angle_speed_threshold_, 7.0);
  pnh_.param("AGE_THRESHOLD", age_threshold_, 1.0);
  pnh_.param("ANGLE_UPPER_THRESHOLD", angle_upper_threshold_, 10.0);
  pnh_.param("ANGLE_THRESHOLD", angle_threshold_, 0.01);
  pnh_.param("DIST_THRESHOLD", dist_threshold_, 0.01);
  pnh_.param("SCORE_THRESHOLD_MAX", score_threshold_max_, 0.1);
  pnh_.param("Point_Quantity_THRESHOLD", point_quantity_threshold_, 200.0);
  pnh_.param("Maximum_Iterations", maximum_iterations_, 100.0);

  pnh_.param("Variance_X", variance_x_, 0.01);
  pnh_.param("Variance_Y", variance_y_, 0.01);
  pnh_.param("Variance_Yaw", variance_yaw_, 0.01);

  pnh_.param("Scan_Range_Max", scan_range_max_, 20.0);
  pnh_.param("Scan_Range_Min", scan_range_min_, 0.3);
  pnh_.param("VoxelGridRemoval_LeafSize", voxel_grid_leaf_size_, 0.05);
  pnh_.param("ObstacleRemoval_Distance_Max", obstacle_removal_distance_max_, 2.0);
  pnh_.param("location_restricted_zone", location_restricted_zone_, std::vector<double>());

  pnh_.param("Relocation_Weight_Score", relocation_weight_score_, 0.5);
  pnh_.param("Relocation_Weight_Distance", relocation_weight_distance_, 0.5);
  pnh_.param("Relocation_Weight_Yaw", relocation_weight_yaw_, 0.5);
  pnh_.param("Relocation_Maximum_Iterations", relocation_maximum_iterations_, 80.0);
  pnh_.param("Relocation_Score_Threshold_Max", relocation_score_threshold_max_, 0.15);
  pnh_.param("Loss_Num_Threshold", loss_num_threshold_, -1);
  pnh_.param("SACIA_Max_Correspondence_Distance", sacia_max_correspondence_distance_, 0.5);
  pnh_.param("SACIA_Max_Iterations", sacia_max_iterations_, 100.0);
  pnh_.param("SACIA_Min_Sample_Distance", sacia_min_sample_distance_, 10);
}

void ScanToMapICP::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  std::lock_guard<std::mutex> lock(odom_lock_);
  odom_initialized_ = true;
  odom_queue_.push_back(*odom_msg);
  while (static_cast<int>(odom_queue_.size()) > odom_queue_length_)
  {
    odom_queue_.pop_front();
  }
}

void ScanToMapICP::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  cloud_map_->clear();
  occupancyGridToPointCloud(map_msg, cloud_map_);
  map_initialized_ = true;
  publishPointCloud(map_pointcloud_publisher_, cloud_map_, map_frame_, ros::Time(0));
  ROS_INFO_STREAM("Loaded occupied map cloud with " << cloud_map_->size() << " points");
}

bool ScanToMapICP::relocalizeCallback(std_srvs::Trigger::Request&,
                                      std_srvs::Trigger::Response& res)
{
  if (!map_initialized_ || !odom_initialized_)
  {
    res.success = false;
    res.message = "map or odom is not initialized";
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(relocation_mutex_);
    need_relocalization_ = true;
    relocalization_done_ = false;
    relocalization_result_ = false;
  }

  {
    std::unique_lock<std::mutex> lock(relocation_mutex_);
    relocation_cv_.wait(lock, [this] { return relocalization_done_ || !ros::ok(); });
    res.success = relocalization_result_;
  }

  res.message = res.success ? "relocalization succeeded" : "relocalization failed";
  return true;
}

void ScanToMapICP::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  const auto scan_start_time = std::chrono::steady_clock::now();

  if (!map_initialized_ || !odom_initialized_)
  {
    return;
  }

  bool need_reloc = false;
  {
    std::lock_guard<std::mutex> lock(relocation_mutex_);
    need_reloc = need_relocalization_;
  }

  // 重定位优先
  if (need_reloc)
  {
    if (!getTransform(base_to_lidar_, base_frame_, lidar_frame_, scan_msg->header.stamp) ||
        !getTransform(map_to_base_, map_frame_, base_frame_, scan_msg->header.stamp))
    {
      {
        std::lock_guard<std::mutex> lock(relocation_mutex_);
        relocalization_result_ = false;
        need_relocalization_ = false;
        relocalization_done_ = true;
      }
      relocation_cv_.notify_one();
      return;
    }

    map_to_lidar_ = map_to_base_ * base_to_lidar_;

    const bool ok = reLocationWithICP(match_result_, scan_msg, cloud_map_, map_to_base_);

    {
      std::lock_guard<std::mutex> lock(relocation_mutex_);
      relocalization_result_ = ok;
      need_relocalization_ = false;
      relocalization_done_ = true;
    }
    relocation_cv_.notify_one();

    if (ok)
    {
      // 重定位成功 → 重置丢帧计数，发布位姿
      location_loss_num_ = 0;
      location_publisher_.publish(isometryToPoseMsg(match_result_, scan_msg->header.stamp));
    }
    return;
  }

  if (!getTransform(base_to_lidar_, base_frame_, lidar_frame_, scan_msg->header.stamp))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to get %s -> %s transform",
                      base_frame_.c_str(), lidar_frame_.c_str());
    return;
  }

  if (!scan_initialized_)
  {
    scan_time_ = scan_msg->header.stamp.toSec();
    match_time_ = scan_time_;

    if (!getTransform(map_to_base_, map_frame_, base_frame_, scan_msg->header.stamp))
    {
      ROS_WARN_THROTTLE(1.0, "Failed to get %s -> %s transform",
                        map_frame_.c_str(), base_frame_.c_str());
      return;
    }

    scan_initialized_ = true;
  }
  else
  {
    last_match_time_ = match_time_;
    last_match_result_ = match_result_;
    scan_time_ = scan_msg->header.stamp.toSec();

    if (ros::Time::now().toSec() - scan_time_ > age_threshold_)
    {
      ROS_WARN_THROTTLE(1.0, "Scan data timeout");
      scan_initialized_ = false;
      return;
    }

    Eigen::Isometry3d base_last_to_base_now = Eigen::Isometry3d::Identity();
    if (!getOdomTransform(base_last_to_base_now, last_match_time_, scan_time_))
    {
      ROS_WARN_THROTTLE(1.0, "Failed to get odom delta");
      return;
    }

    map_to_base_ = last_match_result_ * base_last_to_base_now;
  }

  match_time_ = scan_time_;
  map_to_lidar_ = map_to_base_ * base_to_lidar_;

  if (use_tf_tree_always_)
  {
    scan_initialized_ = false;
  }

  if (isCoordinateInRange(location_restricted_zone_, map_to_base_))
  {
    ROS_INFO_THROTTLE(1.0, "In restricted area, skip ICP match");
    return;
  }

  scanToPointCloudOnMap(scan_msg, cloud_scan_);
  pointCloudVoxelGridRemoval(cloud_scan_, voxel_grid_leaf_size_);
  pointCloudObstacleRemoval(cloud_map_, cloud_scan_, obstacle_removal_distance_max_);

  publishPointCloud(scan_pointcloud_publisher_, cloud_scan_, map_frame_, scan_msg->header.stamp);

  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  if (!scanMatchWithICP(correction, cloud_scan_, cloud_map_))
  {
    scan_initialized_ = false;
    return;
  }

  match_result_ = correction * map_to_base_;

  Eigen::Isometry3d base_begin_to_base_now = Eigen::Isometry3d::Identity();
  if (get2TimeTransform(base_begin_to_base_now))
  {
    match_result_ = match_result_ * base_begin_to_base_now;
  }

  const auto location_msg = isometryToPoseMsg(match_result_, scan_msg->header.stamp);
  location_publisher_.publish(location_msg);

  if (if_debug_)
  {
    const auto scan_end_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> used = scan_end_time - scan_start_time;
    const std::chrono::duration<double> period = scan_end_time - scan_last_end_time_;
    ROS_INFO_STREAM("Scan callback cost: " << used.count() << " s, frequency: "
                    << (period.count() > 0.0 ? 1.0 / period.count() : 0.0) << " Hz");
    scan_last_end_time_ = scan_end_time;
  }
}

bool ScanToMapICP::scanMatchWithICP(Eigen::Isometry3d& correction,
                                    PointCloudT::Ptr& cloud_scan_msg,
                                    PointCloudT::Ptr& cloud_map_msg)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setMaxCorrespondenceDistance(5.0);
  icp.setMaximumIterations(static_cast<int>(maximum_iterations_));
  icp.setInputSource(cloud_scan_msg);
  icp.setInputTarget(cloud_map_msg);

  PointCloudT pointcloud_result;
  icp.align(pointcloud_result);

  if (!icp.hasConverged())
  {
    publishLocationInfo(false, false, cloud_scan_msg->size(), 0.0, 0.0,
                        std::numeric_limits<double>::infinity());
    return false;
  }

  Eigen::Affine3f transform;
  transform = icp.getFinalTransformation();
  float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);

  const double trans_dist = std::hypot(x, y);
  const double angle_dist = std::abs(yaw);
  const double score = icp.getFitnessScore();
  const double angular_speed = latestAngularSpeed();

  if (if_debug_)
  {
    ROS_INFO_STREAM("ICP trans=" << trans_dist << ", yaw=" << angle_dist
                    << ", score=" << score << ", angular_speed=" << angular_speed
                    << ", points=" << cloud_scan_msg->size());
  }

  if ((trans_dist < dist_threshold_ && angle_dist < angle_threshold_) ||
      cloud_scan_msg->size() < point_quantity_threshold_)
  {
    publishLocationInfo(false, false, cloud_scan_msg->size(), trans_dist, angle_dist, score);
    return false;
  }

  if (angle_dist > angle_upper_threshold_ || score > score_threshold_max_ ||
      angular_speed > angle_speed_threshold_)
  {
    ++location_loss_num_;
    if (location_loss_num_ > loss_num_threshold_ && loss_num_threshold_ != -1)
    {
      std::lock_guard<std::mutex> lock(relocation_mutex_);
      need_relocalization_ = true;
    }
    publishLocationInfo(false, false, cloud_scan_msg->size(), trans_dist, angle_dist, score);
    return false;
  }

  location_loss_num_ = 0;
  correction.matrix() = transform.matrix().cast<double>();
  publishLocationInfo(false, true, cloud_scan_msg->size(), trans_dist, angle_dist, score);
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pointcloud_result, ros_cloud);
    ros_cloud.header.frame_id = map_frame_;
    ros_cloud.header.stamp = ros::Time::now();
    icp_pointcloud_publisher_.publish(ros_cloud);
  }
  return true;
}

bool ScanToMapICP::globalRelocalizationWithSACIA(
    Eigen::Isometry3d& trans,
    const sensor_msgs::LaserScan::ConstPtr& scan_msg,
    PointCloudT::Ptr& cloud_map_msg)
{
  if (cloud_scan_->size() < 50 || cloud_map_msg->size() < 50)
  {
    ROS_WARN("SAC-IA: too few points (%zu scan, %zu map)",
             cloud_scan_->size(), cloud_map_msg->size());
    return false;
  }

  pcl::PointCloud<pcl::Normal>::Ptr scan_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  normal_est.setSearchMethod(tree);
  normal_est.setRadiusSearch(0.1);

  normal_est.setInputCloud(cloud_scan_);
  normal_est.compute(*scan_normals);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scan_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud(cloud_scan_);
  fpfh_est.setInputNormals(scan_normals);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.setRadiusSearch(0.2);
  fpfh_est.compute(*scan_features);

  pcl::PointCloud<pcl::Normal>::Ptr map_normals(new pcl::PointCloud<pcl::Normal>);
  normal_est.setInputCloud(cloud_map_msg);
  normal_est.compute(*map_normals);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr map_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh_est.setInputCloud(cloud_map_msg);
  fpfh_est.setInputNormals(map_normals);
  fpfh_est.compute(*map_features);

  pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sacia;
  sacia.setInputSource(cloud_scan_);
  sacia.setSourceFeatures(scan_features);
  sacia.setInputTarget(cloud_map_msg);
  sacia.setTargetFeatures(map_features);
  sacia.setMaxCorrespondenceDistance(static_cast<float>(sacia_max_correspondence_distance_));
  sacia.setMaximumIterations(static_cast<int>(sacia_max_iterations_));
  sacia.setMinSampleDistance(static_cast<float>(sacia_min_sample_distance_));

  PointCloudT sacia_result_cloud;
  sacia.align(sacia_result_cloud);

  if (!sacia.hasConverged())
  {
    ROS_WARN("SAC-IA did not converge");
    return false;
  }

  Eigen::Affine3f sacia_transform;
  sacia_transform = sacia.getFinalTransformation();
  trans.matrix() = sacia_transform.matrix().cast<double>();

  ROS_INFO("SAC-IA succeeded, fitness score: %.6f", sacia.getFitnessScore());
  return true;
}

bool ScanToMapICP::reLocationWithICP(Eigen::Isometry3d& trans,
                                     const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                     PointCloudT::Ptr& cloud_map_msg,
                                     const Eigen::Isometry3d& robot_pose)
{
  scanToPointCloudOnMap(scan_msg, cloud_scan_);
  pointCloudVoxelGridRemoval(cloud_scan_, voxel_grid_leaf_size_);

  // Step 1: SAC-IA 全局匹配（不依赖初始位姿）
  Eigen::Isometry3d sacia_result = Eigen::Isometry3d::Identity();
  if (globalRelocalizationWithSACIA(sacia_result, scan_msg, cloud_map_msg))
  {
    // SAC-IA 成功，用其结果做 ICP 精化
    PointCloudT::Ptr transformed_scan(new PointCloudT);
    pcl::transformPointCloud(*cloud_scan_, *transformed_scan, sacia_result.matrix().cast<float>());

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(transformed_scan);
    icp.setInputTarget(cloud_map_msg);
    icp.setMaxCorrespondenceDistance(5.0);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    PointCloudT icp_final;
    icp.align(icp_final);

    if (icp.hasConverged() && icp.getFitnessScore() < relocation_score_threshold_max_)
    {
      Eigen::Affine3f icp_correction;
      icp_correction = icp.getFinalTransformation();
      Eigen::Isometry3d refined;
      refined.matrix() = icp_correction.matrix().cast<double>();
      trans = refined * sacia_result;

      *cloud_scan_ = *transformed_scan;
      pcl::transformPointCloud(*cloud_scan_, *cloud_scan_, icp_correction);

      float score = icp.getFitnessScore();
      relocate_initialpose_publisher_.publish(isometryToPoseMsg(trans, ros::Time::now()));
      publishPointCloud(icp_pointcloud_publisher_, cloud_scan_, map_frame_, ros::Time::now());
      publishLocationInfo(true, true, cloud_scan_->size(), 0.0, 0.0, score);
      ROS_INFO("SAC-IA+ICP relocalization succeeded, score: %.6f", score);
      return true;
    }
    ROS_WARN("SAC-IA succeeded but ICP refinement failed, falling back to rotation search");
  }

  // Step 2: 36角度暴力搜索（后备）
  double best_score = std::numeric_limits<double>::infinity();
  Eigen::Affine3f best_icp = Eigen::Affine3f::Identity();
  Eigen::Isometry3d best_initial = Eigen::Isometry3d::Identity();
  PointCloudT::Ptr best_cloud(new PointCloudT);

  for (double angle_deg = 0.0; angle_deg < 360.0; angle_deg += 10.0)
  {
    const double angle_rad = angle_deg * M_PI / 180.0;
    Eigen::Affine3f initial_affine = Eigen::Affine3f::Identity();
    initial_affine.rotate(Eigen::AngleAxisf(static_cast<float>(angle_rad), Eigen::Vector3f::UnitZ()));

    PointCloudT::Ptr rotated_scan_cloud(new PointCloudT(*cloud_scan_));
    rotatePointCloud(rotated_scan_cloud, initial_affine, robot_pose.cast<float>());
    if (if_debug_)
    {
      publishPointCloud(rotate_pointcloud_publisher_, rotated_scan_cloud, map_frame_, ros::Time::now());
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(rotated_scan_cloud);
    icp.setInputTarget(cloud_map_msg);
    icp.setMaxCorrespondenceDistance(15.0);
    icp.setMaximumIterations(static_cast<int>(relocation_maximum_iterations_));
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.01);

    PointCloudT::Ptr pointcloud_result(new PointCloudT);
    icp.align(*pointcloud_result);
    if (!icp.hasConverged())
    {
      continue;
    }

    Eigen::Affine3f transform;
    transform = icp.getFinalTransformation();
    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);

    const double trans_dist = std::hypot(x, y);
    const double angle_dist = std::abs(yaw);
    const double weighted_score = relocation_weight_score_ * icp.getFitnessScore() +
                                  relocation_weight_distance_ * trans_dist +
                                  relocation_weight_yaw_ * angle_dist;

    Eigen::Isometry3d initial_transform = Eigen::Isometry3d::Identity();
    initial_transform.rotate(Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()));

    Eigen::Isometry3d visual_pose = Eigen::Isometry3d::Identity();
    visual_pose.matrix() = transform.matrix().cast<double>();
    visual_pose = visual_pose * robot_pose * initial_transform;
    relocate_visual_pose_publisher_.publish(isometryToPoseMsg(visual_pose, ros::Time::now()));

    if (weighted_score < best_score)
    {
      best_score = weighted_score;
      best_icp = transform;
      best_initial = initial_transform;
      best_cloud = pointcloud_result;
    }
  }

  if (!std::isfinite(best_score) || best_score > relocation_score_threshold_max_)
  {
    publishLocationInfo(true, false, cloud_scan_->size(), 0.0, 0.0, best_score);
    ROS_WARN("Relocalization failed, best score: %.6f", best_score);
    return false;
  }

  Eigen::Isometry3d best_correction = Eigen::Isometry3d::Identity();
  best_correction.matrix() = best_icp.matrix().cast<double>();
  trans = best_correction * robot_pose * best_initial;

  relocate_initialpose_publisher_.publish(isometryToPoseMsg(trans, ros::Time::now()));
  publishPointCloud(icp_pointcloud_publisher_, best_cloud, map_frame_, ros::Time::now());
  publishLocationInfo(true, true, cloud_scan_->size(), 0.0, 0.0, best_score);
  ROS_INFO("Relocalization succeeded, best score: %.6f", best_score);
  return true;
}

void ScanToMapICP::occupancyGridToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map_msg,
                                             PointCloudT::Ptr& cloud_msg)
{
  cloud_msg->clear();
  cloud_msg->height = 1;
  cloud_msg->is_dense = false;

  std_msgs::Header header;
  header.stamp = ros::Time(0.0);
  header.frame_id = map_frame_;
  cloud_msg->header = pcl_conversions::toPCL(header);

  const double origin_x = map_msg->info.origin.position.x;
  const double origin_y = map_msg->info.origin.position.y;
  const double resolution = map_msg->info.resolution;

  for (unsigned int y = 0; y < map_msg->info.height; ++y)
  {
    for (unsigned int x = 0; x < map_msg->info.width; ++x)
    {
      const size_t index = x + y * map_msg->info.width;
      if (map_msg->data[index] == 100)
      {
        PointT point;
        point.x = static_cast<float>((0.5 + x) * resolution + origin_x);
        point.y = static_cast<float>((0.5 + y) * resolution + origin_y);
        point.z = 0.0f;
        cloud_msg->points.push_back(point);
      }
    }
  }

  cloud_msg->width = cloud_msg->points.size();
}

void ScanToMapICP::scanToPointCloudOnMap(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                         PointCloudT::Ptr& cloud_msg)
{
  cloud_msg->clear();
  cloud_msg->height = 1;
  cloud_msg->is_dense = false;

  for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
  {
    const float range = scan_msg->ranges[i];
    if (!std::isfinite(range) || range <= scan_msg->range_min || range >= scan_msg->range_max ||
        range <= scan_range_min_ || range >= scan_range_max_)
    {
      continue;
    }

    const double angle = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
    const Eigen::Vector3d point_lidar(range * std::cos(angle), range * std::sin(angle), 0.0);
    const Eigen::Vector3d point_map = map_to_lidar_ * point_lidar;

    PointT point;
    point.x = static_cast<float>(point_map.x());
    point.y = static_cast<float>(point_map.y());
    point.z = 0.0f;
    cloud_msg->points.push_back(point);
  }

  cloud_msg->width = cloud_msg->points.size();

  std_msgs::Header header;
  header.stamp = scan_msg->header.stamp;
  header.frame_id = map_frame_;
  cloud_msg->header = pcl_conversions::toPCL(header);
}

void ScanToMapICP::pointCloudVoxelGridRemoval(PointCloudT::Ptr& cloud_msg, double leaf_size)
{
  if (cloud_msg->empty() || leaf_size <= 0.0)
  {
    return;
  }

  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud_msg);
  voxel_grid.setLeafSize(static_cast<float>(leaf_size),
                         static_cast<float>(leaf_size),
                         static_cast<float>(leaf_size));
  voxel_grid.filter(*cloud_msg);
}

void ScanToMapICP::pointCloudObstacleRemoval(PointCloudT::Ptr& cloud_map_msg,
                                             PointCloudT::Ptr& cloud_msg,
                                             double distance_threshold)
{
  if (cloud_map_msg->empty() || cloud_msg->empty())
  {
    return;
  }

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud_map_msg);

  const double distance_threshold_sq = distance_threshold * distance_threshold;
  PointCloudT::Ptr kept_cloud(new PointCloudT);
  PointCloudT::Ptr removed_cloud(new PointCloudT);
  kept_cloud->reserve(cloud_msg->size());

  for (const auto& point : cloud_msg->points)
  {
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree.nearestKSearch(point, 1, indices, distances) > 0 &&
        distances[0] <= distance_threshold_sq)
    {
      kept_cloud->push_back(point);
    }
    else
    {
      removed_cloud->push_back(point);
    }
  }

  kept_cloud->header = cloud_msg->header;
  kept_cloud->height = 1;
  kept_cloud->width = kept_cloud->size();
  kept_cloud->is_dense = false;
  cloud_msg.swap(kept_cloud);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = map_frame_;
  publishPointCloud(removal_pointcloud_publisher_, removed_cloud, map_frame_, header.stamp);
}

bool ScanToMapICP::getTransform(Eigen::Isometry3d& trans,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const ros::Time& stamp)
{
  try
  {
    const auto transform_stamped =
        tf_buffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(0.5));

    const auto& t = transform_stamped.transform.translation;
    const auto& q_msg = transform_stamped.transform.rotation;
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);

    trans = Eigen::Isometry3d::Identity();
    trans.rotate(q.normalized());
    trans.pretranslate(Eigen::Vector3d(t.x, t.y, t.z));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    return false;
  }
}

bool ScanToMapICP::getOdomTransform(Eigen::Isometry3d& trans, double start_time, double end_time)
{
  std::lock_guard<std::mutex> lock(odom_lock_);
  if (odom_queue_.empty() || odom_queue_.front().header.stamp.toSec() > start_time ||
      odom_queue_.back().header.stamp.toSec() < end_time)
  {
    return false;
  }

  nav_msgs::Odometry start_odom = odom_queue_.front();
  nav_msgs::Odometry end_odom = odom_queue_.back();

  for (const auto& odom : odom_queue_)
  {
    const double time = odom.header.stamp.toSec();
    if (time <= start_time)
    {
      start_odom = odom;
    }
    if (time <= end_time)
    {
      end_odom = odom;
    }
    else
    {
      break;
    }
  }

  const Eigen::Isometry3d odom_to_base_start = odomMsgToIsometry(start_odom);
  const Eigen::Isometry3d odom_to_base_end = odomMsgToIsometry(end_odom);
  trans = odom_to_base_start.inverse() * odom_to_base_end;
  return true;
}

bool ScanToMapICP::get2TimeTransform(Eigen::Isometry3d& trans)
{
  Eigen::Isometry3d current_map_to_base = Eigen::Isometry3d::Identity();
  if (!getTransform(current_map_to_base, map_frame_, base_frame_, ros::Time(0)))
  {
    return false;
  }

  trans = map_to_base_.inverse() * current_map_to_base;
  return true;
}

Eigen::Isometry3d ScanToMapICP::odomMsgToIsometry(const nav_msgs::Odometry& odom_msg) const
{
  const auto& p = odom_msg.pose.pose.position;
  const auto& q_msg = odom_msg.pose.pose.orientation;
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.rotate(q.normalized());
  iso.pretranslate(Eigen::Vector3d(p.x, p.y, p.z));
  return iso;
}

geometry_msgs::PoseWithCovarianceStamped
ScanToMapICP::isometryToPoseMsg(const Eigen::Isometry3d& iso, const ros::Time& stamp) const
{
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = map_frame_;

  const Eigen::Quaterniond q(iso.rotation());
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();
  pose_msg.pose.pose.orientation.w = q.w();
  pose_msg.pose.pose.position.x = iso.translation().x();
  pose_msg.pose.pose.position.y = iso.translation().y();
  pose_msg.pose.pose.position.z = iso.translation().z();

  pose_msg.pose.covariance = {
      variance_x_, 0, 0, 0, 0, 0,
      0, variance_y_, 0, 0, 0, 0,
      0, 0, 1e-9, 0, 0, 0,
      0, 0, 0, 1e-9, 0, 0,
      0, 0, 0, 0, 1e-9, 0,
      0, 0, 0, 0, 0, variance_yaw_};

  return pose_msg;
}

void ScanToMapICP::publishPointCloud(ros::Publisher& pub, const PointCloudT::Ptr& cloud,
                         const std::string& frame_id, const ros::Time& stamp)
{
  if (!cloud || cloud->empty())
  {
    return;
  }
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = frame_id;
  ros_cloud.header.stamp = stamp;
  pub.publish(ros_cloud);
}

void ScanToMapICP::rotatePointCloud(PointCloudT::Ptr& cloud_msg,
                                    const Eigen::Affine3f& rotation,
                                    const Eigen::Affine3f& robot_pose)
{
  pcl::transformPointCloud(*cloud_msg, *cloud_msg, robot_pose.inverse());
  pcl::transformPointCloud(*cloud_msg, *cloud_msg, rotation);
  pcl::transformPointCloud(*cloud_msg, *cloud_msg, robot_pose);
}

bool ScanToMapICP::isCoordinateInRange(const std::vector<double>& ranges,
                                       const Eigen::Isometry3d& coord) const
{
  if (ranges.size() % 4 != 0)
  {
    ROS_WARN_THROTTLE(5.0, "location_restricted_zone size must be a multiple of 4");
    return false;
  }

  const double x = coord.translation().x();
  const double y = coord.translation().y();
  for (size_t i = 0; i < ranges.size(); i += 4)
  {
    const double min_x = std::min(ranges[i], ranges[i + 2]);
    const double max_x = std::max(ranges[i], ranges[i + 2]);
    const double min_y = std::min(ranges[i + 1], ranges[i + 3]);
    const double max_y = std::max(ranges[i + 1], ranges[i + 3]);
    if (x >= min_x && x <= max_x && y >= min_y && y <= max_y)
    {
      return true;
    }
  }

  return false;
}

double ScanToMapICP::latestAngularSpeed() const
{
  std::lock_guard<std::mutex> lock(odom_lock_);
  if (odom_queue_.empty())
  {
    return 0.0;
  }
  return std::abs(odom_queue_.back().twist.twist.angular.z);
}

void ScanToMapICP::publishLocationInfo(bool relocation,
                                       bool success,
                                       double point_count,
                                       double trans_dist,
                                       double angle_dist,
                                       double score)
{
  scan_to_map::LocationInfo msg;
  msg.if_relocation = relocation;
  msg.if_match_success = success;
  msg.point_cloud_quantity = point_count;
  msg.tranDist = trans_dist;
  msg.angleDist = angle_dist;
  msg.angle_apeed = latestAngularSpeed();
  msg.score = score;
  location_info_publisher_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_map_icp_node");
  ScanToMapICP scan_to_map_icp;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
