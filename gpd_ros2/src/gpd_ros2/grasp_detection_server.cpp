// grasp_detection_server_ros2.cpp

#include <memory>
#include <string>
#include <vector>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/ros/conversions.h>  // if you still use pcl::fromROSMsg

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gpd_ros2_msgs/srv/detect_grasps.hpp>
#include <gpd_ros2_msgs/msg/grasp_config_list.hpp>
#include <gpd_ros2_msgs/msg/cloud_indexed.hpp>
#include <gpd_ros2_msgs/msg/cloud_sources.hpp>

// Your existing headers (assumed ROS2-ready)
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <gpd/candidate/hand.h>
#include <gpd_ros2/grasp_plotter.h>
#include <gpd_ros2/grasp_messages.h>

using PointCloudPointNormal = pcl::PointCloud<pcl::PointNormal>;
using PointCloudRGBA        = pcl::PointCloud<pcl::PointXYZRGBA>;

class GraspDetectionServer : public rclcpp::Node {
public:
  GraspDetectionServer()
  : rclcpp::Node("detect_grasps_server")
  {
    declare_parameter<std::vector<double>>("camera_position", {0.0, 0.0, 0.0});
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>("rviz_topic", "");
    declare_parameter<std::vector<double>>("workspace", {-10.0, -10.0, -10.0, 10.0, 10.0, 10.0});

    auto camera_position = get_parameter("camera_position").as_double_array();
    view_point_ << camera_position[0], camera_position[1], camera_position[2];

    auto cfg_file = get_parameter("config_file").as_string();
    grasp_detector_ = std::make_unique<gpd::GraspDetector>(cfg_file);

    auto rviz_topic = get_parameter("rviz_topic").as_string();
    if (!rviz_topic.empty()) {
      rviz_plotter_ = std::make_unique<GraspPlotter>(
        shared_from_this(), grasp_detector_->getHandSearchParameters().hand_geometry_);
      use_rviz_ = true;
    } else {
      use_rviz_ = false;
    }

    workspace_ = get_parameter("workspace").as_double_array();

    grasps_pub_ = create_publisher<gpd_ros2_msgs::msg::GraspConfigList>("clustered_grasps", 10);

    service_ = create_service<gpd_ros2_msgs::srv::DetectGrasps>(
      "detect_grasps",
      std::bind(&GraspDetectionServer::detectGrasps, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "Grasp detection service is ready.");
  }

private:
  void detectGrasps(const std::shared_ptr<rmw_request_id_t>,
                    const std::shared_ptr<gpd_ros2_msgs::srv::DetectGrasps::Request> req,
                    std::shared_ptr<gpd_ros2_msgs::srv::DetectGrasps::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Received service request...");

    cloud_camera_.reset();
    const auto & cloud_sources = req->cloud_indexed.cloud_sources;

    Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
    for (size_t i = 0; i < cloud_sources.view_points.size(); ++i) {
      view_points.col(i) << cloud_sources.view_points[i].x,
                             cloud_sources.view_points[i].y,
                             cloud_sources.view_points[i].z;
    }

    std_msgs::msg::Header header = cloud_sources.cloud.header;
    frame_ = header.frame_id;

    if (cloud_sources.cloud.fields.size() == 6 &&
        cloud_sources.cloud.fields[3].name == "normal_x" &&
        cloud_sources.cloud.fields[4].name == "normal_y" &&
        cloud_sources.cloud.fields[5].name == "normal_z") {
      auto cloud = PointCloudPointNormal::Ptr(new PointCloudPointNormal);
      pcl::fromROSMsg(cloud_sources.cloud, *cloud);

      Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
      for (size_t i = 0; i < cloud_sources.camera_source.size(); ++i)
        camera_source(cloud_sources.camera_source[i].data, static_cast<int>(i)) = 1;

      cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
    } else {
      auto cloud = PointCloudRGBA::Ptr(new PointCloudRGBA);
      pcl::fromROSMsg(cloud_sources.cloud, *cloud);

      Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
      for (size_t i = 0; i < cloud_sources.camera_source.size(); ++i)
        camera_source(cloud_sources.camera_source[i].data, static_cast<int>(i)) = 1;

      cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
      std::cout << "view_points:\n" << view_points << "\n";
    }

    std::vector<int> indices(req->cloud_indexed.indices.size());
    for (size_t i = 0; i < indices.size(); ++i)
      indices[i] = req->cloud_indexed.indices[i].data;
    cloud_camera_->setSampleIndices(indices);

    RCLCPP_INFO(get_logger(), "Received cloud with %zu points, %zu samples",
                cloud_camera_->getCloudProcessed()->size(), req->cloud_indexed.indices.size());

    grasp_detector_->preprocessPointCloud(*cloud_camera_);
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps =
      grasp_detector_->detectGrasps(*cloud_camera_);

    if (!grasps.empty()) {
      if (use_rviz_) rviz_plotter_->drawGrasps(grasps, frame_);

      auto msg = GraspMessages::createGraspListMsg(grasps, header);
      res->grasp_configs = msg;
      grasps_pub_->publish(msg);

      RCLCPP_INFO(get_logger(), "Detected %zu highest-scoring grasps.", msg.grasps.size());
      return;
    }

    RCLCPP_WARN(get_logger(), "No grasps detected!");
  }

private:
  std::unique_ptr<gpd::GraspDetector> grasp_detector_;
  std::unique_ptr<gpd::util::Cloud> cloud_camera_;
  std::unique_ptr<GraspPlotter> rviz_plotter_;

  rclcpp::Publisher<gpd_ros2_msgs::msg::GraspConfigList>::SharedPtr grasps_pub_;
  rclcpp::Service<gpd_ros2_msgs::srv::DetectGrasps>::SharedPtr service_;

  Eigen::Vector3d view_point_{0.0, 0.0, 0.0};
  std::vector<double> workspace_;
  std::string frame_;
  bool use_rviz_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspDetectionServer>());
  rclcpp::shutdown();
  return 0;
}
