#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit();

private:
  struct csvFileData
  {
    std::vector<double> X_;
    std::vector<double> Y_;
    std::vector<double> V_;

    int index_;
    int velocity_index_;

    Eigen::Vector3d lookahead_point_world_;
    Eigen::Vector3d lookahead_point_car_;
    Eigen::Vector3d current_point_world_;
  };

  Eigen::Matrix3d rotation_m_;
  double steering_angle_;
  bool odom_ready_;

  double x_car_world_;
  double y_car_world_;
  double point_x1_;
  double point_x2_;
  double point_y1_;
  double point_y2_;

  std::string outer_waypoints_path_;
  std::string inner_waypoints_path_;
  std::string odom_topic_;
  std::string car_refFrame_;
  std::string drive_topic_;
  std::string global_refFrame_;
  std::string rviz_current_waypoint_topic_;
  std::string rviz_lookahead_waypoint_topic_;

  double lookahead_;

  double low_alpha_;
  double high_alpha_;
  double min_lookahead_;
  double max_lookahead_;
  double high_lookahead_;
  double lookahead_ratio_;
  double steering_limit_;
  double high_speed_;
  double middle_speed_;
  double low_speed_;
  double high_speed_percentage_;
  double hm_speed_percentage_;
  double ml_speed_percentage_;
  double low_speed_percentage_;

  double inner_low_alpha_;
  double inner_high_alpha_;
  double inner_min_lookahead_;
  double inner_max_lookahead_;
  double inner_high_lookahead_;
  double inner_lookahead_ratio_;
  double inner_steering_limit_;
  double inner_high_speed_;
  double inner_middle_speed_;
  double inner_low_speed_;
  double inner_high_speed_percentage_;
  double inner_real_high_speed_percentage_;
  double inner_hm_speed_percentage_;
  double inner_ml_speed_percentage_;
  double inner_low_speed_percentage_;

  double outer_low_alpha_;
  double outer_high_alpha_;
  double outer_min_lookahead_;
  double outer_max_lookahead_;
  double outer_high_lookahead_;
  double outer_lookahead_ratio_;
  double outer_steering_limit_;
  double outer_high_speed_;
  double outer_middle_speed_;
  double outer_low_speed_;
  double outer_high_speed_percentage_;
  double outer_real_high_speed_percentage_;
  double outer_hm_speed_percentage_;
  double outer_ml_speed_percentage_;
  double outer_low_speed_percentage_;

  rclcpp::Time current_time_;
  rclcpp::Time joy_time_;

  double curr_velocity_ = 0.0;
  double velocity_ = 0.0;
  double index_velocity_ = 0.0;
  double prev_velocity_ = 0.0;

  bool emergency_breaking_ = false;
  bool inner_ = false;

  std::string lane_number_ = "left";
  std::fstream csvFile_waypoints_;
  csvFileData outer_waypoints_;
  csvFileData inner_waypoints_;
  csvFileData * waypoints_ = &outer_waypoints_;
  int num_waypoints_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr joy_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double to_radians(double degrees);
  double to_degrees(double radians);
  double p2pdist(double & x1, double & x2, double & y1, double & y2);
  void load_waypoints(std::string & waypoint_file_path, csvFileData & waypoints);
  void visualize_lookahead_point(Eigen::Vector3d & point);
  void visualize_current_point(Eigen::Vector3d & point);
  void get_waypoint();
  void quat_to_rot(double q0, double q1, double q2, double q3);
  void transformandinterp_waypoint();
  double p_controller();
  double get_velocity(double steering_angle);
  void publish_message(double steering_angle);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
  void joy_callback(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr joy);
  void set_waypoints(csvFileData * waypoints);
};
