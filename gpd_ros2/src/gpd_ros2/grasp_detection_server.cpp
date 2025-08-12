// grasp_detection_server_ros2.cpp
#include <gpd_ros2/grasp_detection_server.h>
#include <gpd_ros2/grasp_messages.h>

using PointCloudPointNormal = pcl::PointCloud<pcl::PointNormal>;
using PointCloudRGBA        = pcl::PointCloud<pcl::PointXYZRGBA>;


namespace gpd_ros2 {

GraspDetectionServer::GraspDetectionServer(const rclcpp::NodeOptions &options)
  : rclcpp::Node("grasp_detection_server", options)
{
  declare_parameter<std::vector<double>>("camera_position", {0.0, 0.0, 0.0});
  declare_parameter<std::string>("config_file", std::string(""));
  declare_parameter<std::string>("rviz_topic", std::string(""));
  declare_parameter<std::vector<double>>("workspace", {-10.0, -10.0, -10.0, 10.0, 10.0, 10.0});

  auto camera_position = get_parameter("camera_position").as_double_array();
  view_point_ << camera_position[0], camera_position[1], camera_position[2];

  auto cfg_file = get_parameter("config_file").as_string();
  try
  {
    grasp_detector_ = std::make_unique<gpd::GraspDetector>(cfg_file);
  }
  catch(const std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize gpd::GraspDetector: %s", e.what());
    throw;
  }
  
  // auto rviz_topic = get_parameter("rviz_topic").as_string();
  // if (!rviz_topic.empty()) {
  //   rviz_plotter_ = std::make_unique<GraspPlotter>(
  //     shared_from_this(), grasp_detector_->getHandSearchParameters().hand_geometry_);
  //   use_rviz_ = true;
  // } else {
  //   use_rviz_ = false;
  // }


  grasps_pub_ = create_publisher<gpd_ros2_msgs::msg::GraspConfigList>("clustered_grasps", 10);

  
    // Service
  auto cb = [this](const std::shared_ptr<DetectGrasps::Request> req,
                   std::shared_ptr<DetectGrasps::Response> res) {
    this->handleRequest(req, res);
  };
  this->create_service<DetectGrasps>("detect_grasps", cb);

  RCLCPP_INFO(get_logger(), "Grasp detection service is ready.");
}

void GraspDetectionServer::handleRequest(const Request req, Response res)
{
  RCLCPP_INFO(get_logger(), "Received Grasp Detection Service Request ...");

  cloud_camera_.reset();
  const auto & cloud_sources = req->cloud_indexed.cloud_sources;

  Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
  for (size_t i = 0; i < cloud_sources.view_points.size(); ++i) 
  {
    view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y, cloud_sources.view_points[i].z;
  }

  cloud_camera_header_ = cloud_sources.cloud.header;
  frame_ = cloud_camera_header_.frame_id;

  if (cloud_sources.cloud.fields.size() == 6 && cloud_sources.cloud.fields[3].name == "normal_x" && cloud_sources.cloud.fields[4].name == "normal_y" && cloud_sources.cloud.fields[5].name == "normal_z")
  {
    auto cloud = PointCloudPointNormal::Ptr(new PointCloudPointNormal);
    pcl::fromROSMsg(cloud_sources.cloud, *cloud);

    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
    for (size_t i = 0; i < cloud_sources.camera_source.size(); ++i)
      camera_source(cloud_sources.camera_source[i].data, static_cast<int>(i)) = 1;

    cloud_camera_ = std::make_unique<gpd::util::Cloud>(cloud, camera_source, view_points);
  } 
  else 
  {
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
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;
  if (req->params_policy == gpd_ros2_msgs::srv::DetectGrasps::Request::USE_CFG_FILE) {
    grasps = grasp_detector_->detectGrasps(*cloud_camera_);
  } else if (req->params_policy == gpd_ros2_msgs::srv::DetectGrasps::Request::USE_REQUEST_PARAMS) {
    gpd::DetectParams detect_params = GraspMessages::convertGraspParamsToDetectParams(req->grasp_params);
    grasps = grasp_detector_->detectGrasps(*cloud_camera_, detect_params);
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid params policy: %d", req->params_policy);
    return;
  }
  if (grasps.empty()) {
    RCLCPP_WARN(get_logger(), "No grasps detected!");
    return;
  }
  if (!grasps.empty()) {
    //if (use_rviz_) rviz_plotter_->drawGrasps(grasps, frame_);

    auto msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res->grasp_configs = msg;
    //grasps_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Detected %zu highest-scoring grasps.", msg.grasps.size());
    return;
  }

}

}


int main(int argc, char** argv) {
  // seed the random number generator
  std::srand(std::time(0));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetectionServer>());
  rclcpp::shutdown();
  return 0;
}
