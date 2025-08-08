#include <gpd_ros2/grasp_messages.h>

gpd_ros2_msgs::msg::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::msg::Header& header)
{
  gpd_ros2_msgs::msg::GraspConfigList msg;

  for (size_t i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros2_msgs::msg::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
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
