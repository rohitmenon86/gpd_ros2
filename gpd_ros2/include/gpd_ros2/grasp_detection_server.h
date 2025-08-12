/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas Ported to ROS2 by Rohit Menon
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

#include <memory>
#include <string>
#include <vector>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <gpd/candidate/hand.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


// gpd_ros2_msgs (services and messages)
#include <gpd_ros2_msgs/srv/detect_grasps.hpp>
#include <gpd_ros2_msgs/msg/grasp_config_list.hpp>
#include <gpd_ros2_msgs/msg/cloud_indexed.hpp>
#include <gpd_ros2_msgs/msg/cloud_sources.hpp>
#include <gpd_ros2_msgs/msg/grasp_params.hpp>


// this project (headers)
#include <gpd_ros2/grasp_messages.h>
#include <gpd_ros2/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

namespace gpd_ros2
{

class GraspDetectionServer : public rclcpp::Node
{
public:
  /**
   * \brief Constructor.
   * \param node the ROS2 node shared pointer
   */
  explicit GraspDetectionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using DetectGrasps = gpd_ros2_msgs::srv::DetectGrasps;
  using Request = std::shared_ptr<DetectGrasps::Request>;
  using Response = std::shared_ptr<DetectGrasps::Response>;

  void handleRequest(const Request request, Response response);

  rclcpp::Publisher<gpd_ros2_msgs::msg::GraspConfigList>::SharedPtr grasps_pub_; ///< ROS2 publisher for grasp list messages

  std_msgs::msg::Header cloud_camera_header_; ///< stores header of the point cloud
  std::string frame_; ///< point cloud frame

  std::unique_ptr<gpd::GraspDetector> grasp_detector_; ///< used to run the grasp pose detection
  std::unique_ptr<gpd::util::Cloud> cloud_camera_; ///< stores point cloud with (optional) camera information and surface normals
  //std::unique_ptr<gpd_ros2::GraspPlotter> rviz_plotter_; ///< used to plot detected grasps in rviz


  bool use_rviz_; ///< if rviz is used for visualization instead of PCL
  std::vector<double> workspace_; ///< workspace limits
  Eigen::Vector3d view_point_; ///< (input) view point of the camera onto the point cloud
};

}  // namespace gpd_ros2

#endif /* GRASP_DETECTION_SERVER_H_ */
