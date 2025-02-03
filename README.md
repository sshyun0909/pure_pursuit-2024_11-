# pure_pursuit-2024_11(F1Tenth)
pure_pursuit


https://github.com/user-attachments/assets/2b806c1a-989e-4c91-bf57-020cf7791397




### Overview

This project implements the Pure Pursuit algorithm for autonomous path tracking in ROS2. The algorithm calculates the required steering angle to follow a given path based on a lookahead point.

### Installation
- Ubuntu20.04
- ROS2 FOXY
```
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select pure_pursuit
source install/setup.bash
```

### Launch the Node
경로가 하나인 경우
```
ros2 launch pure_pursuit pure_pursuit.launch.py
```


두 가지 경로를 사용해야 하는 경우

ex) 바깥 경로를 돌다가 안쪽 경로로 돌아야 하는 경우
```
ros2 launch pure_pursuit multi_pure_pursuit_launch.py
```

### Parameters
경로가 하나인 경우
```pure_pursuit:
  ros__parameters:
    waypoints_path: "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal.csv"
    odom_topic: "/pf/pose/odom"
    car_refFrame: "laser"
    global_refFrame: "map"
    drive_topic: "/ackermann_cmd"
    rviz_current_waypoint_topic: "/current_waypoint"
    rviz_lookahead_waypoint_topic: "/lookahead_waypoint"

    min_lookahead: 0.7
    max_lookahead: 1.85
    high_lookahead: 2.5
    lookahead_ratio: 2.5
    high_alpha: 0.4
    low_alpha : 0.6
    steering_limit: 21.0

    high_speed : 6.6
    middle_speed : 4.5
    low_speed : 3.2

    high_speed_percentage: 0.6
    real_high_speed_percentage: 0.825
    hm_speed_percentage: 0.8
    ml_speed_percentage: 0.9
    low_speed_percentage: 1.2

waypoint_visualizer_node:
  ros__parameters:
    waypoints_path: "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal.csv"
```
두 가지 경로를 사용해야 하는 경우

(point_x1, point_x2, point_y1, point_y2의 영역을 차량이 지나가는 경우 경로가 바뀜)
```
pure_pursuit:
  ros__parameters:
    inner_waypoints_path: "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal_inner.csv"
    outer_waypoints_path: "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal_outer.csv"
    odom_topic: "/pf/pose/odom"
    car_refFrame: "laser"
    global_refFrame: "map"
    drive_topic: "/ackermann_cmd"
    rviz_current_waypoint_topic: "/current_waypoint"
    rviz_lookahead_waypoint_topic: "/lookahead_waypoint"

    point_x1: -1.7004
    point_x2: 1.2408
    point_y1: 8.0782
    point_y2: 10.316

    inner_max_lookahead: 0.7
    inner_min_lookahead: 2.0
    inner_high_lookahead: 2.5
    inner_lookahead_ratio: 2.5
    inner_high_alpha: 0.5
    inner_low_alpha : 0.7
    inner_steering_limit: 21.0

    inner_high_speed : 6.6
    inner_middle_speed : 4.5
    inner_low_speed : 3.2

    inner_high_speed_percentage: 0.7
    inner_real_high_speed_percentage: 0.8
    inner_hm_speed_percentage: 0.825
    inner_ml_speed_percentage: 0.825
    inner_low_speed_percentage: 1.2


    outer_min_lookahead: 0.7
    outer_max_lookahead: 2.0
    outer_high_lookahead: 2.5
    outer_lookahead_ratio: 2.5
    outer_high_alpha: 0.4
    outer_low_alpha : 0.55
    outer_steering_limit: 21.0

    outer_high_speed : 6.6
    outer_middle_speed : 4.6
    outer_low_speed : 3.2

    outer_high_speed_percentage: 0.7
    outer_real_high_speed_percentage: 0.7
    outer_hm_speed_percentage: 0.85
    outer_ml_speed_percentage: 0.8
    outer_low_speed_percentage: 0.9

waypoint_visualizer_node:
  ros__parameters:
    waypoints_path: "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal.csv"
```
### Code
```
#include "pure_pursuit.hpp"

PurePursuit::PurePursuit()
: Node("pure_pursuit_node")
{
  this->declare_parameter(
    "waypoints_path",
    "/home/seunghyun/mrlab_ws/src/pure_pursuit/racelines/lane_optimal.csv");
  this->declare_parameter("odom_topic", "/ego_racecar/odom");
  this->declare_parameter("car_refFrame", "ego_racecar/base_link");
  this->declare_parameter("global_refFrame", "map");
  this->declare_parameter("drive_topic", "/drive");
  this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
  this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
  this->declare_parameter("min_lookahead", 0.5);
  this->declare_parameter("max_lookahead", 1.0);
  this->declare_parameter("high_lookahead", 1.0);
  this->declare_parameter("lookahead_ratio", 8.0);
  this->declare_parameter("high_alpha", 0.5);
  this->declare_parameter("low_alpha", 0.5);
  this->declare_parameter("steering_limit", 25.0);
  this->declare_parameter("high_speed", 1.0);
  this->declare_parameter("middle_speed", 1.0);
  this->declare_parameter("low_speed", 1.0);
  this->declare_parameter("high_speed_percentage", 1.0);
  this->declare_parameter("real_high_speed_percentage", 1.0);
  this->declare_parameter("hm_speed_percentage", 0.6);
  this->declare_parameter("ml_speed_percentage", 0.6);
  this->declare_parameter("low_speed_percentage", 0.6);

  this->get_parameter("waypoints_path", waypoints_path_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("car_refFrame", car_refFrame_);
  this->get_parameter("global_refFrame", global_refFrame_);
  this->get_parameter("drive_topic", drive_topic_);
  this->get_parameter("rviz_current_waypoint_topic", rviz_current_waypoint_topic_);
  this->get_parameter("rviz_lookahead_waypoint_topic", rviz_lookahead_waypoint_topic_);
  this->get_parameter("min_lookahead", min_lookahead_);
  this->get_parameter("max_lookahead", max_lookahead_);
  this->get_parameter("high_lookahead", high_lookahead_);
  this->get_parameter("lookahead_ratio", lookahead_ratio_);
  this->get_parameter("high_alpha", high_alpha_);
  this->get_parameter("low_alpha", low_alpha_);
  this->get_parameter("steering_limit", steering_limit_);
  this->get_parameter("high_speed", high_speed_);
  this->get_parameter("middle_speed", middle_speed_);
  this->get_parameter("low_speed", low_speed_);
  this->get_parameter("high_speed_percentage", high_speed_percentage_);
  this->get_parameter("real_high_speed_percentage", real_high_speed_percentage_);
  this->get_parameter("hm_speed_percentage", hm_speed_percentage_);
  this->get_parameter("ml_speed_percentage", ml_speed_percentage_);
  this->get_parameter("low_speed_percentage", low_speed_percentage_);

  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 1, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
  joy_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
    "/ackermann_cmd_joy", 1, std::bind(&PurePursuit::joy_callback, this, std::placeholders::_1));

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);
  vis_current_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    rviz_current_waypoint_topic_, 10);
  vis_lookahead_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    rviz_lookahead_waypoint_topic_, 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  current_time_ = now();
  joy_time_ = now();

  load_waypoints();
}

void PurePursuit::joy_callback(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr joy)
{
  joy_time_ = now();
}

void PurePursuit::load_waypoints()
{
  RCLCPP_INFO(this->get_logger(), "load_waypoints");
  csvFile_waypoints_.open(waypoints_path_, std::ios::in);

  if (!csvFile_waypoints_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open the CSV File: %s", waypoints_path_);
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "CSV File Opened");
  }

  std::string line, word;

  while (!csvFile_waypoints_.eof()) {
    std::getline(csvFile_waypoints_, line, '\n');
    std::stringstream s(line);

    int j = 0;
    while (getline(s, word, ',')) {
      if (!word.empty()) {
        if (j == 0) {
          waypoints_.X_.push_back(std::stod(word));
        } else if (j == 1) {
          waypoints_.Y_.push_back(std::stod(word));
        } else if (j == 2) {
          waypoints_.V_.push_back(std::stod(word));
        }
      }
      j++;
    }
  }

  csvFile_waypoints_.close();
  num_waypoints_ = waypoints_.X_.size();
  RCLCPP_INFO(this->get_logger(), "Finished loading, Open %d waypoints.", num_waypoints_);

  double average_dist_between_waypoints = 0.0;
  for (int i = 0; i < num_waypoints_ - 1; i++) {
    average_dist_between_waypoints +=
      p2pdist(waypoints_.X_[i], waypoints_.X_[i + 1], waypoints_.Y_[i], waypoints_.Y_[i + 1]);
  }
  average_dist_between_waypoints = average_dist_between_waypoints / num_waypoints_;
}


void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  auto current_time = now();
  if ((current_time - joy_time_).seconds() > 2.0) {
    high_speed_percentage_ = real_high_speed_percentage_;
  }

  x_car_world_ = odom_msg->pose.pose.position.x;
  y_car_world_ = odom_msg->pose.pose.position.y;

  get_waypoint();
  transformandinterp_waypoint();

  steering_angle_ = p_controller();

  publish_message(steering_angle_);
}


void PurePursuit::get_waypoint()
{
  double longest_distance = 0;
  int final_i = -1;
  int start = waypoints_.index_;
  int end = (waypoints_.index_ + 30) % num_waypoints_;

  if (index_velocity_ >= high_speed_) {
    lookahead_ = high_lookahead_;
  } else {
    lookahead_ = std::min(
      std::max(min_lookahead_, max_lookahead_ * curr_velocity_ / lookahead_ratio_),
      max_lookahead_);
  }

  if (end < start) {
    for (int i = start; i < num_waypoints_; i++) {
      if (p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) <= lookahead_ &&
        p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) >= longest_distance)
      {
        longest_distance = p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_);
        final_i = i;
      }
    }
    for (int i = 0; i < end; i++) {
      if (p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) <= lookahead_ &&
        p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) >= longest_distance)
      {
        longest_distance = p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_);
        final_i = i;
      }
    }
  } else {
    for (int i = start; i < end; i++) {
      if (p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) <= lookahead_ &&
        p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) >= longest_distance)
      {
        longest_distance = p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_);
        final_i = i;
      }
    }
  }

  if (final_i == -1) {
    final_i = 0;
    for (int i = 0; i < num_waypoints_; i++) {
      if (p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) <= lookahead_ &&
        p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) >= longest_distance)
      {
        longest_distance = p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_);
        final_i = i;
      }
    }
  }

  double shortest_distance =
    p2pdist(waypoints_.X_[0], x_car_world_, waypoints_.Y_[0], y_car_world_);
  int velocity_i = 0;
  for (int i = 0; i < num_waypoints_; i++) {
    if (p2pdist(
        waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_) <= shortest_distance)
    {
      shortest_distance = p2pdist(waypoints_.X_[i], x_car_world_, waypoints_.Y_[i], y_car_world_);
      velocity_i = i;
    }
  }

  waypoints_.index_ = final_i;
  waypoints_.velocity_index_ = velocity_i;
}


void PurePursuit::transformandinterp_waypoint()
{
  waypoints_.lookahead_point_world_ << waypoints_.X_[waypoints_.index_],
    waypoints_.Y_[waypoints_.index_], 0.0;
  waypoints_.current_point_world_ << waypoints_.X_[waypoints_.velocity_index_],
    waypoints_.Y_[waypoints_.velocity_index_], 0.0;

  visualize_lookahead_point(waypoints_.lookahead_point_world_);
  visualize_current_point(waypoints_.current_point_world_);

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      car_refFrame_, global_refFrame_,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not read transform data, Error : %s", ex.what());
  }

  Eigen::Vector3d translation_v(transformStamped.transform.translation.x,
    transformStamped.transform.translation.y,
    transformStamped.transform.translation.z);
  quat_to_rot(
    transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

  waypoints_.lookahead_point_car_ =
    (rotation_m_ * waypoints_.lookahead_point_world_) + translation_v;
}


double PurePursuit::p_controller()
{
  double r = waypoints_.lookahead_point_car_.norm();
  double y = waypoints_.lookahead_point_car_(1);
  // double angle = alpha_ * 2 * y / pow(r, 2);
  double angle = low_alpha_ * 2 * y / pow(r, 2);

  if (index_velocity_ >= high_speed_) {
    angle = high_alpha_ * 2 * y / pow(r, 2);
  }

  return angle;
}


void PurePursuit::publish_message(double steering_angle)
{
  ackermann_msgs::msg::AckermannDriveStamped drive;
  if (steering_angle < 0.0) {
    drive.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit_));
  } else {
    drive.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit_));
  }

  curr_velocity_ = get_velocity(drive.drive.steering_angle);
  drive.drive.speed = curr_velocity_;

  RCLCPP_INFO(
    this->get_logger(), "Speed : %.3f m/s, Steering angle : %.3f, lookahead_distance : %.3f",
    drive.drive.speed, to_degrees(drive.drive.steering_angle), lookahead_);

  drive_pub_->publish(drive);
}


double PurePursuit::get_velocity(double steering_angle)
{
  if (waypoints_.V_[waypoints_.velocity_index_]) {
    velocity_ = waypoints_.V_[waypoints_.velocity_index_];
    index_velocity_ = velocity_;
    if (velocity_ >= high_speed_) {
      velocity_ = velocity_ * high_speed_percentage_;
    } else if (velocity_ < high_speed_ && velocity_ >= middle_speed_) {
      velocity_ = velocity_ * hm_speed_percentage_;
    } else if (velocity_ < middle_speed_ && velocity_ >= low_speed_) {
      velocity_ = velocity_ * ml_speed_percentage_;
    } else if (velocity_ < low_speed_) {
      velocity_ = velocity_ * low_speed_percentage_;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Could Not Get Velocity Data");
    if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
      velocity_ = 6.0 * high_speed_percentage_;
    } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
      velocity_ = 2.5 * high_speed_percentage_;
    } else {
      velocity_ = 2.0 * high_speed_percentage_;
    }
  }

  return velocity_;
}


double PurePursuit::p2pdist(double & x1, double & x2, double & y1, double & y2)
{
  double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  return dist;
}

double PurePursuit::to_radians(double degrees)
{
  double radians;
  return radians = degrees * M_PI / 180.0;
}


double PurePursuit::to_degrees(double radians)
{
  double degrees;
  return degrees = radians * 180 / M_PI;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3)
{
  double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
  double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
  double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

  double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
  double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
  double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

  double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
  double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
  double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

  rotation_m_ << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}


void PurePursuit::visualize_lookahead_point(Eigen::Vector3d & point)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0;
  marker.color.r = 1.0;

  marker.pose.position.x = point(0);
  marker.pose.position.y = point(1);
  marker.id = 1;
  vis_lookahead_point_pub_->publish(marker);
}

void PurePursuit::visualize_current_point(Eigen::Vector3d & point)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0;
  marker.color.b = 1.0;

  marker.pose.position.x = point(0);
  marker.pose.position.y = point(1);
  marker.id = 1;
  vis_current_point_pub_->publish(marker);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuit>();
  rclcpp::Rate rate(20);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```
