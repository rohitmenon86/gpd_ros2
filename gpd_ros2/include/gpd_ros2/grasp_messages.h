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

#ifndef GRASP_MESSAGES_H_
#define GRASP_MESSAGES_H_

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/header.hpp>

#include <gpd/candidate/hand.h>

#include <gpd_ros2_msgs/msg/grasp_config.hpp>
#include <gpd_ros2_msgs/msg/grasp_config_list.hpp>
#include <gpd_ros2_msgs/msg/grasp_params.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <gpd/grasp_detector.h>

inline geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

inline geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

namespace GraspMessages
{
  gpd_ros2_msgs::msg::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::msg::Header& header);

  gpd_ros2_msgs::msg::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand);

  /**
   * @brief Convert ROS 2 grasp parameters message to GPD detect parameters.
   * 
   * @param m Grasp parameters message.
   * @return DetectParams Struct containing the converted parameters.
   */
  gpd::DetectParams convertGraspParamsToDetectParams(const gpd_ros2_msgs::msg::GraspParams &m);
}  // namespace GraspMessages


#endif /* GRASP_MESSAGES_H_ */
