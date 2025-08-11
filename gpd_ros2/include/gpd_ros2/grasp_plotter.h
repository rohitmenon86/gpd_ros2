/*
 * Software License Agreement (BSD License)
 *
 *  Original Copyright (c) 2018, Andreas ten Pas
 *  ROS 2 port adapted by Rohit Menon Humanoid Robots Lab, University of Bonn, 2025.
 */

#ifndef GRASP_PLOTTER_HPP_
#define GRASP_PLOTTER_HPP_

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// GPD
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

/**
 * @brief Draw grasps in RViz (ROS 2).
 */
class GraspPlotter
{
public:
  /**
   * @brief Constructor.
   * @param node  Shared ROS 2 node.
   * @param params Hand geometry parameters (from GPD).
   */
  GraspPlotter(const rclcpp::Node::SharedPtr& node,
               const gpd::candidate::HandGeometry& params);

  /**
   * @brief Visualize grasps in RViz.
   * @param hands The grasps to be visualized.
   * @param frame The frame that the grasps are in.
   */
  void drawGrasps(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
                  const std::string& frame);

  /**
   * @brief Convert a list of grasps to a MarkerArray.
   * @param hands   List of grasps.
   * @param frame_id Frame ID.
   */
  visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
      const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
      const std::string& frame_id);

  /**
   * @brief Create a finger marker (box).
   * @param center Finger center.
   * @param rot    Orientation (rotation matrix).
   * @param lwh    Length/width/height.
   * @param id     Unique marker ID.
   * @param frame_id Frame ID.
   */
  visualization_msgs::msg::Marker createFingerMarker(
      const Eigen::Vector3d& center,
      const Eigen::Matrix3d& rot,
      const Eigen::Vector3d& lwh,
      int id,
      const std::string& frame_id);

  /**
   * @brief Create the hand base marker (between fingers).
   * @param start  Start position.
   * @param end    End position.
   * @param frame  Orientation frame.
   * @param length Base length.
   * @param height Base height.
   * @param id     Unique marker ID.
   * @param frame_id Frame ID.
   */
  visualization_msgs::msg::Marker createHandBaseMarker(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      const Eigen::Matrix3d& frame,
      double length,
      double height,
      int id,
      const std::string& frame_id);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;  // RViz publisher

  // Hand geometry (copied from params)
  double outer_diameter_{0.0};
  double hand_depth_{0.0};
  double finger_width_{0.0};
  double hand_height_{0.0};
};

#endif  // GRASP_PLOTTER_HPP_
