#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <deque>
#include <limits>

class ScanDistortionCorrector
{
public:
  ScanDistortionCorrector()
    : nh_("~"),
      tf_listener_(tf_buffer_)
  {
    target_frame_ = nh_.param<std::string>("target_frame", "map");
    source_frame_ = nh_.param<std::string>("source_frame", "base_scan");

    scan_sub_ = nh_.subscribe("/scan", 10,
                              &ScanDistortionCorrector::scanCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 200,
                              &ScanDistortionCorrector::odomCallback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_corrected", 10);

    ROS_INFO("[scan_corrector] start: source=%s target=%s",
             source_frame_.c_str(), target_frame_.c_str());
  }

private:
  struct OdomPose
  {
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static tf2::Transform poseToTransform(const OdomPose& pose)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    return tf2::Transform(q, tf2::Vector3(pose.x, pose.y, 0.0));
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    OdomPose pose;
    pose.stamp = msg->header.stamp;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;

    const geometry_msgs::Quaternion& q_msg = msg->pose.pose.orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, pose.yaw);

    if (!odom_buffer_.empty() && pose.stamp < odom_buffer_.back().stamp)
    {
      ROS_WARN_THROTTLE(2.0,
                        "[scan_corrector] ignoring out-of-order odom sample");
      return;
    }

    odom_buffer_.push_back(pose);

    const ros::Duration keep_duration(10.0);
    while (odom_buffer_.size() > 2 &&
           (odom_buffer_.back().stamp - odom_buffer_.front().stamp) > keep_duration)
    {
      odom_buffer_.pop_front();
    }
  }

  bool interpolatePose(const ros::Time& stamp, OdomPose& pose) const
  {
    if (odom_buffer_.size() < 2)
    {
      return false;
    }

    if (stamp < odom_buffer_.front().stamp || stamp > odom_buffer_.back().stamp)
    {
      return false;
    }

    for (std::size_t i = 1; i < odom_buffer_.size(); ++i)
    {
      const OdomPose& before = odom_buffer_[i - 1];
      const OdomPose& after = odom_buffer_[i];

      if (stamp > after.stamp)
      {
        continue;
      }

      const double dt = (after.stamp - before.stamp).toSec();
      if (dt <= 0.0)
      {
        pose = before;
        pose.stamp = stamp;
        return true;
      }

      const double ratio = (stamp - before.stamp).toSec() / dt;
      pose.stamp = stamp;
      pose.x = before.x + (after.x - before.x) * ratio;
      pose.y = before.y + (after.y - before.y) * ratio;
      pose.yaw = normalizeAngle(before.yaw +
                                normalizeAngle(after.yaw - before.yaw) * ratio);
      return true;
    }

    return false;
  }

  bool lookupLatestScanTransform(geometry_msgs::TransformStamped& transform) const
  {
    try
    {
      // Latest available source_frame -> target_frame TF. This avoids exact
      // beam timestamp lookups, which are brittle when TF publication lags.
      transform = tf_buffer_.lookupTransform(target_frame_, source_frame_,
                                             ros::Time(0), ros::Duration(0.2));
      return true;
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(2.0, "[scan_corrector] latest TF not ready: %s", e.what());
      return false;
    }
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    if (scan->ranges.empty())
    {
      scan_pub_.publish(scan);
      return;
    }

    const std::size_t beam_count = scan->ranges.size();
    double time_increment = scan->time_increment;
    if (time_increment <= 0.0 && scan->scan_time > 0.0)
    {
      time_increment = scan->scan_time / static_cast<double>(beam_count);
    }
    if (time_increment < 0.0 || !std::isfinite(time_increment))
    {
      ROS_WARN_THROTTLE(2.0,
                        "[scan_corrector] invalid scan time_increment: %.9f",
                        time_increment);
      scan_pub_.publish(scan);
      return;
    }

    const ros::Time scan_start = scan->header.stamp;
    OdomPose odom_start;
    if (!interpolatePose(scan_start, odom_start))
    {
      if (!interpolatePose(ros::Time(0), odom_start))
      {
        if (odom_buffer_.empty())
        {
          ROS_WARN_THROTTLE(2.0,
                            "[scan_corrector] odom unavailable for scan start %.6f",
                            scan_start.toSec());
          scan_pub_.publish(scan);
          return;
        }

        odom_start = odom_buffer_.back();
      }
    }

    const tf2::Transform tf_odom_start = poseToTransform(odom_start);

    sensor_msgs::LaserScan corrected = *scan;
    corrected.header.frame_id = source_frame_;

    std::size_t corrected_count = 0;
    for (std::size_t i = 0; i < beam_count; ++i)
    {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) ||
          range < scan->range_min ||
          range > scan->range_max)
      {
        continue;
      }

      const ros::Time beam_time = scan_start + ros::Duration(i * time_increment);

      OdomPose odom_beam;
      if (!interpolatePose(beam_time, odom_beam))
      {
        continue;
      }

      const double angle = scan->angle_min + i * scan->angle_increment;
      const tf2::Vector3 beam_point(range * std::cos(angle),
                                    range * std::sin(angle),
                                    0.0);

      const tf2::Transform tf_odom_beam = poseToTransform(odom_beam);
      const tf2::Transform tf_beam_to_start =
          tf_odom_start.inverse() * tf_odom_beam;
      const tf2::Vector3 corrected_point = tf_beam_to_start * beam_point;

      const double corrected_range =
          std::hypot(corrected_point.x(), corrected_point.y());
      if (std::isfinite(corrected_range) &&
          corrected_range >= scan->range_min &&
          corrected_range <= scan->range_max)
      {
        corrected.ranges[i] = static_cast<float>(corrected_range);
        ++corrected_count;
      }
    }

    ROS_DEBUG("[scan_corrector] corrected %zu/%zu beams",
              corrected_count, beam_count);
    scan_pub_.publish(corrected);
  }

  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string target_frame_;
  std::string source_frame_;
  std::deque<OdomPose> odom_buffer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_corrector");
  ScanDistortionCorrector corrector;
  ros::spin();
  return 0;
}
