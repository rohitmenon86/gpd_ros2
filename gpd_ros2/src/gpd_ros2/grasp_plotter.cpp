// grasp_plotter_ros2.cpp

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

#include <gpd_ros2/grasp_plotter.h>  // header you ported earlier

GraspPlotter::GraspPlotter(const rclcpp::Node::SharedPtr& node,
                           const gpd::candidate::HandGeometry& params)
: node_(node)
{
  node_->declare_parameter<std::string>("rviz_topic", "");
  std::string rviz_topic = node_->get_parameter("rviz_topic").as_string();

  rviz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_topic, 1);

  hand_depth_ = params.depth_;
  hand_height_ = params.height_;
  outer_diameter_ = params.outer_diameter_;
  finger_width_ = params.finger_width_;
}

void GraspPlotter::drawGrasps(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
                              const std::string& frame)
{
  auto markers = convertToVisualGraspMsg(hands, frame);
  rviz_pub_->publish(markers);
}

visualization_msgs::msg::MarkerArray
GraspPlotter::convertToVisualGraspMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
                                      const std::string& frame_id)
{
  double hw = 0.5 * outer_diameter_ - 0.5 * finger_width_;

  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < hands.size(); ++i) {
    const auto& h = hands[i];

    Eigen::Vector3d left_bottom  = h->getPosition() - hw * h->getBinormal();
    Eigen::Vector3d right_bottom = h->getPosition() + hw * h->getBinormal();
    Eigen::Vector3d left_top     = left_bottom  + hand_depth_ * h->getApproach();
    Eigen::Vector3d right_top    = right_bottom + hand_depth_ * h->getApproach();
    Eigen::Vector3d left_center  = left_bottom  + 0.5 * (left_top - left_bottom);
    Eigen::Vector3d right_center = right_bottom + 0.5 * (right_top - right_bottom);
    Eigen::Vector3d base_center  = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * h->getApproach();
    (void)base_center; // kept for parity with ROS1; not used directly here

    Eigen::Vector3d finger_lwh(hand_depth_, finger_width_, hand_height_);
    Eigen::Vector3d approach_lwh(0.08, finger_width_, hand_height_);
    Eigen::Matrix3d R = h->getFrame();

    auto base     = createHandBaseMarker(left_bottom, right_bottom, R, 0.02, hand_height_, static_cast<int>(i), frame_id);
    auto left_f   = createFingerMarker(left_center,  R, finger_lwh, static_cast<int>(i*3),   frame_id);
    auto right_f  = createFingerMarker(right_center, R, finger_lwh, static_cast<int>(i*3+1), frame_id);
    auto approach = createFingerMarker(base_center - 0.04 * h->getApproach(), R, approach_lwh, static_cast<int>(i*3+2), frame_id);

    marker_array.markers.push_back(left_f);
    marker_array.markers.push_back(right_f);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}

visualization_msgs::msg::Marker
GraspPlotter::createFingerMarker(const Eigen::Vector3d& center,
                                 const Eigen::Matrix3d& frame,
                                 const Eigen::Vector3d& lwh,
                                 int id,
                                 const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();

  Eigen::Quaterniond q(frame);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = lwh.x();
  marker.scale.y = lwh.y();
  marker.scale.z = lwh.z();

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  marker.lifetime = rclcpp::Duration::from_seconds(10.0);

  return marker;
}

visualization_msgs::msg::Marker
GraspPlotter::createHandBaseMarker(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end,
                                   const Eigen::Matrix3d& frame,
                                   double length,
                                   double height,
                                   int id,
                                   const std::string& frame_id)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = center.x();
  marker.pose.position.y = center.y();
  marker.pose.position.z = center.z();

  Eigen::Quaterniond q(frame);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = length;
  marker.scale.y = (end - start).norm();
  marker.scale.z = height;

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(10.0);

  return marker;
}
