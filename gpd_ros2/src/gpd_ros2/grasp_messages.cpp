#include <gpd_ros2/grasp_messages.h>
#include <Eigen/Geometry>

namespace GraspMessages {

gpd_ros2_msgs::msg::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::msg::Header& header)
{
  gpd_ros2_msgs::msg::GraspConfigList msg;

  for (size_t i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros2_msgs::msg::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros2_msgs::msg::GraspConfig msg;
  msg.position = toPoint(hand.getPosition());
  msg.approach = toVector3(hand.getApproach());
  msg.binormal = toVector3(hand.getBinormal());
  msg.axis = toVector3(hand.getAxis());
  msg.width.data = (hand.getGraspWidth());
  msg.score.data = float_t(hand.getScore());
  msg.sample = toPoint(hand.getSample());

  return msg;
}

gpd::DetectParams convertGraspParamsToDetectParams(const gpd_ros2_msgs::msg::GraspParams &m) {
  gpd::DetectParams d;
  d.approach_direction << m.approach_direction.x, m.approach_direction.y, m.approach_direction.z;

  const size_t n = m.camera_position.size();
  const size_t cols = n / 3;
  if (n % 3 == 0 && n > 0) {
    d.camera_position = Eigen::Map<const Eigen::Matrix<double,3,Eigen::Dynamic>>(m.camera_position.data(), 3, cols);
  } else {
    d.camera_position.resize(3, 0);
  }

  const auto &t = m.transform_camera2base.translation;
  const auto &r = m.transform_camera2base.rotation;
  d.transform_camera2base = Eigen::Translation3d(t.x, t.y, t.z) * Eigen::Quaterniond(r.w, r.x, r.y, r.z);

  d.workspace = m.workspace;
  d.can_filter_approach = m.enable_approach_dir_filtering;
  d.thresh_rad = m.approach_dir_threshold;
  return d;
}

}  // namespace GraspMessages

