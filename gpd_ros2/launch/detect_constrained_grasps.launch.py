# launch/detect_constrained_grasps.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # Launch args (tweak defaults as needed)
  config_file = LaunchConfiguration("config_file")
  camera_position = LaunchConfiguration("camera_position")  # e.g. "[0.0, 0.0, 0.0]"
  workspace = LaunchConfiguration("workspace")              # e.g. "[-0.5,0.5,-0.5,0.5,0.0,1.5]"
  rviz_topic = LaunchConfiguration("rviz_topic")            # leave empty to disable RViz publisher
  use_sim_time = LaunchConfiguration("use_sim_time")
  namespace = LaunchConfiguration("namespace")
  node_name = LaunchConfiguration("node_name")

  return LaunchDescription([
    DeclareLaunchArgument(
      "config_file",
      default_value=PathJoinSubstitution([
        FindPackageShare("gpd_ros2"), "cfg", "ros_eigen_params.cfg"
      ]),
      description="Path to GPD config file used by gpd::GraspDetector."
    ),
    DeclareLaunchArgument(
      "camera_position", default_value="[0.0, 0.0, 0.0]",
      description="Camera position (x,y,z) in base frame for view filtering (if enabled)."
    ),
    DeclareLaunchArgument(
      "workspace", default_value="[-10.0,-10.0,-10.0, 10.0,10.0,10.0]",
      description="Workspace limits [xmin,xmax,ymin,ymax,zmin,zmax]."
    ),
    DeclareLaunchArgument(
      "rviz_topic", default_value="",
      description="RViz marker topic. Leave empty to disable RViz visualization."
    ),
    DeclareLaunchArgument(
      "use_sim_time", default_value="false",
      description="Use /clock (sim time)."
    ),
    DeclareLaunchArgument(
      "namespace", default_value="",
      description="ROS namespace for the node."
    ),
    DeclareLaunchArgument(
      "node_name", default_value="grasp_detection_server",
      description="Node name."
    ),

    Node(
      package="gpd_ros2",
      executable="gpd_ros2_detect_grasps_server",         # your installed target/binary name
      name=node_name,
      namespace=namespace,
      output="screen",
      parameters=[{
        "config_file": config_file,
        "camera_position": camera_position,
        "workspace": workspace,
        "rviz_topic": rviz_topic,
        "use_sim_time": use_sim_time,
      }],
      # If you later want to remap topics, add in 'remappings=[(...), ...]'
    ),
  ])
