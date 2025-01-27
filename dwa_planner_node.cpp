#include "dwa_planner/dwa_planner_node.hpp"
#include "dwa_planner/dwa_planner_component.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <limits>

long num_existed_markers = 0;

namespace dwa_planner
{

DWAPlannerNode::DWAPlannerNode()
: Node("dwa_planner"),
  x_{0.0, 0.0, 0.0, 0.0, 0.0},
  goal_{0.0, 0.0},
  robot_radius_(0.3),
  obstacle_radius_(0.3),
  received_obstacles_(false),
  received_goal_(false),
  received_odom_(false)
  //received_traj_(false)
{
  // デフォルトパラメータ
//   kinematic_ = {1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1.0)};
  kinematic_ = {0.5, toRadian(20.0), 0.1, toRadian(50.0), 0.1, toRadian(10.0)};
  eval_param_ = {0.3, 0.5, 0.1, 3.0};

  // Subscriber
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&DWAPlannerNode::odomCallback, this, std::placeholders::_1));

  local_obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/closest_point_marker", 10,
    std::bind(&DWAPlannerNode::local_obstacle_callback, this, std::placeholders::_1));

  /*trajectory_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/trajectory", 10,
    std::bind(&DWAPlannerNode::trajectory_callback, this, std::placeholders::_1));
  //*/
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "waypoint", 10,
    std::bind(&DWAPlannerNode::target_callback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  trajectory_end_points_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_end_points", 10);

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DWAPlannerNode::timerCallback, this));

  // TF
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  send_static_transform();
}

/*void DWAPlannerNode::trajectory_publisher(const std::vector<std::array<double,5>> edb){
  visualization_msgs::msg::MarkerArray marker_array;
  for(auto ed : edb){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link"; // フレームIDは適切なものを指定
    marker.header.stamp = this->get_clock()->now();
    //closest_point_marker.ns = "obstacle_points";
    marker.ns = "trajectory_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = ed[0];
    marker.pose.position.y = ed[1];
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; // 球の大きさ
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // 青色
        // MarkerArrayに追加
    marker_array.markers.push_back(marker);
  }
  trajectory_pub_->publish(marker_array);
}//*/

void DWAPlannerNode::timerCallback()
{
  if(!received_odom_){
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for /odom...");
    return;
  }
  if(!received_obstacles_){
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for local_obstacle_markers...");
    return;
  }
  if(!received_goal_){
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for /waypoint...");
    return;
  }
  /*if(!received_traj_){
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for /trajectory...");
  }//*/

  // DWA計算
  auto u = DWA::DynamicWindowApproach(
    x_, 
    kinematic_, 
    goal_, 
    eval_param_,
    obstacle_, 
    obstacle_radius_, 
    robot_radius_);
  
  auto ep = DWA::DynamicWindowApproach_traject_end_points(
    x_, 
    kinematic_, 
    goal_, 
    eval_param_,
    obstacle_, 
    obstacle_radius_, 
    robot_radius_);
  
  RCLCPP_INFO(get_logger(), "traject: (%.2f, %.2f)", u[0], u[1]);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = u[0];
  cmd.angular.z = u[1];
  cmd_vel_pub_->publish(cmd);

  visualization_msgs::msg::MarkerArray eps;
  visualization_msgs::msg::MarkerArray eps_d;
  long d = 0;
  if(num_existed_markers != 0){
    for(long i = 0; i< num_existed_markers; i++){
      visualization_msgs::msg::Marker point_d;
      point_d.header.frame_id = "map";
      point_d.ns = "trajectory_point";
      point_d.id = i;
      point_d.action = visualization_msgs::msg::Marker::DELETE;
      eps_d.markers.push_back(point_d);
    }
    num_existed_markers = 0;
    trajectory_end_points_pub_->publish(eps_d);
  }

  for(auto e : ep){
    visualization_msgs::msg::Marker point;
    if(e[2] == 0.0){
      point.scale.x = 0.1;
      point.scale.y = 0.1;
      point.scale.z = 0.1;
      point.color.a = 1.0;
      point.color.r = 1.0;
      point.color.g = 1.0;
      point.color.b = 1.0; 
    }else{
      point.scale.x = 0.2;
      point.scale.y = 0.2;
      point.scale.z = 0.2;
      point.color.a = 1.0;
      point.color.r = 1.0;
      point.color.g = 0.0;
      point.color.b = 1.0; 
    }
    point.id = d;
    point.header.frame_id = "map"; // フレームIDは適切なものを指定
    point.ns = "trajectory_point";
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;
    point.pose.position.x = e[0];
    point.pose.position.y = e[1];
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    eps.markers.push_back(point);
    d++;
  }
  num_existed_markers = d;
  trajectory_end_points_pub_->publish(eps);
}

void DWAPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x_[0] = msg->pose.pose.position.x;
  x_[1] = msg->pose.pose.position.y;
  
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  x_[2] = yaw;
  x_[3] = msg->twist.twist.linear.x;
  x_[4] = msg->twist.twist.angular.z;

  if(!x_.empty()) {
    received_odom_ = true;
  }
}

//以下書き換え
void DWAPlannerNode::local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  obstacle_.clear();
  int count_obj = 0;

  for(const auto &marker : msg->markers){
    if(marker.type == visualization_msgs::msg::Marker::SPHERE){
      double ox = x_[0] + marker.pose.position.x;
      double oy = x_[1] + marker.pose.position.y;
      //RCLCPP_INFO(get_logger(), "Object: (%.2f, %.2f)", ox, oy);
      obstacle_.push_back({ox, oy});
      count_obj++;
    }
  }
  //RCLCPP_INFO(get_logger(), "num: (%.2d)", count_obj);
  if(!obstacle_.empty()){
    received_obstacles_ = true;
  }
}

void DWAPlannerNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_[0] = msg->pose.position.x;
  goal_[1] = msg->pose.position.y;

  //RCLCPP_INFO(get_logger(), "New goal set: (%.2f, %.2f)", goal_[0], goal_[1]);

  if(!goal_.empty()) {
    received_goal_ = true;
  }
}

/*void DWAPlannerNode::trajectory_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  trajectory_.clear();
  for(const auto &tr : msg->markers){
    if(tr.type == visualization_msgs::msg::Marker::SPHERE){
      double ox = tr.pose.position.x;
      double oy = tr.pose.position.y;
      RCLCPP_INFO(get_logger(), "trajectory found: (%.2f,%.2f)", ox,oy);
      trajectory_.push_back({ox,oy});
    }
  }
  if(!obstacle_.empty()){
    received_traj_ = true;
  }
}//*/

void DWAPlannerNode::send_static_transform()
{
  geometry_msgs::msg::TransformStamped static_transform_stamped;
  static_transform_stamped.header.stamp = now();
  //static_transform_stamped.header.frame_id = "base_link";
  static_transform_stamped.header.frame_id = "map";
  static_transform_stamped.child_frame_id = "odom";
  static_transform_stamped.transform.translation.x=0.0;
  static_transform_stamped.transform.translation.y=0.0;
  static_transform_stamped.transform.translation.z=0.0;
  static_transform_stamped.transform.rotation.x=0.0;
  static_transform_stamped.transform.rotation.y=0.0;
  static_transform_stamped.transform.rotation.z=0.0;
  static_transform_stamped.transform.rotation.w=1.0;

  static_broadcaster_->sendTransform(static_transform_stamped);
}

} // namespace dwa_planner