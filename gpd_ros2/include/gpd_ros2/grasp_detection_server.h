/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRASP_DETECTION_SERVER_H_
#define GRASP_DETECTION_SERVER_H_


// ROS2
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// this project (services)
#include <gpd_ros2_msgs/srv/detect_grasps.h>

// this project (messages)
#include <gpd_ros2_msgs/msg/grasp_config.h>
#include <gpd_ros2_msgs/msg/grasp_config_list.h>

// this project (headers)
#include <gpd_ros2/grasp_messages.h>
#include <gpd_ros2/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


class GraspDetectionServer
{
public:
  /**
   * \brief Constructor.
   * \param node the ROS2 node shared pointer
   */
  GraspDetectionServer(const rclcpp::Node::SharedPtr& node);

  /**
   * \brief Destructor.
   */
  ~GraspDetectionServer();

  /**
   * \brief Service callback for detecting grasps.
   * \param req the service request
   * \param res the service response
   */
  void detectGrasps(
    const std::shared_ptr<gpd_ros2_msgs::srv::DetectGrasps::Request> request,
    std::shared_ptr<gpd_ros2_msgs::srv::DetectGrasps::Response> response);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<gpd_ros2_msgs::msg::GraspConfigList>::SharedPtr grasps_pub_; ///< ROS2 publisher for grasp list messages

  std_msgs::msg::Header cloud_camera_header_; ///< stores header of the point cloud
  std::string frame_; ///< point cloud frame

  std::unique_ptr<gpd::GraspDetector> grasp_detector_; ///< used to run the grasp pose detection
  std::unique_ptr<gpd::util::Cloud> cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals
  //std::unique_ptr<GraspPlotter> rviz_plotter_; ///< used to plot detected grasps in rviz

  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits
  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud
};

#endif /* GRASP_DETECTION_SERVER_H_ */
