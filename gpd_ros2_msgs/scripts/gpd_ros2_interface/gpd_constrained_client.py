#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.task import Future

# ROS messages
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Transform
from sensor_msgs.msg import PointCloud2, PointField
from gpd_ros2_msgs.srv import DetectGrasps
from gpd_ros2_msgs.msg import CloudIndexed, CloudSources, GraspParams

try:
  import open3d as o3d  # optional
  _HAS_O3D = True
except Exception:
  _HAS_O3D = False

def _to_numpy_xyz(pcd: Union[np.ndarray, 'o3d.geometry.PointCloud']) -> np.ndarray:
  if isinstance(pcd, np.ndarray):
    assert pcd.shape[1] == 3, "pcd must be Nx3"
    return pcd.astype(np.float32, copy=False)
  if _HAS_O3D and isinstance(pcd, o3d.geometry.PointCloud):
    return np.asarray(pcd.points, dtype=np.float32)
  raise TypeError("pcd must be numpy Nx3 or open3d.geometry.PointCloud")

def _pcd_bounds(xyz: np.ndarray):
  if xyz.size == 0:
    return np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
  return xyz.min(axis=0), xyz.max(axis=0)

def _pad_workspace(min_xyz: np.ndarray, max_xyz: np.ndarray, margin: float) -> List[float]:
  return [
    float(min_xyz[0] - margin), float(max_xyz[0] + margin),
    float(min_xyz[1] - margin), float(max_xyz[1] + margin),
    float(min_xyz[2] - margin), float(max_xyz[2] + margin),
  ]

def _normalize(v: np.ndarray) -> np.ndarray:
  n = np.linalg.norm(v)
  if n < 1e-9:
    return v
  return v / n

def _vector3(xyz: Iterable[float]) -> Vector3:
  v = Vector3()
  v.x, v.y, v.z = [float(x) for x in xyz]
  return v

def _transform_from_rt(translation_xyz: Iterable[float], quat_xyzw: Iterable[float]) -> Transform:
  t = Transform()
  tx, ty, tz = translation_xyz
  qx, qy, qz, qw = quat_xyzw
  t.translation.x = float(tx)
  t.translation.y = float(ty)
  t.translation.z = float(tz)
  t.rotation.x = float(qx)
  t.rotation.y = float(qy)
  t.rotation.z = float(qz)
  t.rotation.w = float(qw)
  return t

def _xyz_to_pc2(xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
  msg = PointCloud2()
  msg.header.stamp = stamp
  msg.header.frame_id = frame_id
  msg.height = 1
  msg.width = xyz.shape[0]
  msg.fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
  ]
  msg.is_bigendian = False
  msg.point_step = 12
  msg.row_step = msg.point_step * msg.width
  msg.is_dense = True
  msg.data = np.asarray(xyz, dtype=np.float32).tobytes()
  return msg

@dataclass
class StretchApproachConfig:
  max_lift_z: float = 1.1
  table_clearance: float = 0.05
  fallback_dir: Tuple[float, float, float] = (1.0, 0.0, 0.0)

def decide_approach_direction(object_xyz: np.ndarray,
                              cfg: StretchApproachConfig) -> np.ndarray:
  if object_xyz.size == 0:
    return np.array([0.0, 0.0, -1.0], dtype=np.float32)
  _, maxs = _pcd_bounds(object_xyz)
  top = float(maxs[2])
  if top + cfg.table_clearance < cfg.max_lift_z:
    return np.array([0.0, 0.0, -1.0], dtype=np.float32)
  return np.array(cfg.fallback_dir, dtype=np.float32)

class GpdConstrainedClient(Node):
  def __init__(self):
    super().__init__("gpd_constrained_client")
    self.cli = self.create_client(DetectGrasps, "detect_grasps")
    if not self.cli.wait_for_service(timeout_sec=2.0):
      self.get_logger().warn("detect_grasps service not immediately available. Will still try to call.")

  def call_with_pcds(
      self,
      pcd_obj: Union[np.ndarray, 'o3d.geometry.PointCloud'],
      pcd_env: Union[np.ndarray, 'o3d.geometry.PointCloud'],
      frame_id: str,
      cam_positions: List[Tuple[float, float, float]],
      cam_to_base: Tuple[float, float, float, float, float, float, float],
      params_policy: int,
      approach_threshold_deg: float = 20.0,
      workspace_margin: float = 0.02,
      enable_approach_filter: bool = True,
      stretch_max_lift_z: float = 1.1,
  ) -> DetectGrasps.Response:

    obj_xyz = _to_numpy_xyz(pcd_obj)
    env_xyz = _to_numpy_xyz(pcd_env)

    full_xyz = np.vstack([env_xyz, obj_xyz]) if env_xyz.size and obj_xyz.size else (
      obj_xyz if env_xyz.size == 0 else env_xyz
    )
    obj_start = env_xyz.shape[0]
    obj_count = obj_xyz.shape[0]
    obj_indices = list(range(obj_start, obj_start + obj_count))

    stamp = self.get_clock().now().to_msg()
    cloud_msg = _xyz_to_pc2(full_xyz, frame_id, stamp)

    sources = CloudSources()
    sources.cloud = cloud_msg

    cloud_indexed = CloudIndexed()
    cloud_indexed.cloud_sources = sources
    cloud_indexed.indices = obj_indices

    mins, maxs = _pcd_bounds(env_xyz if env_xyz.size else full_xyz)
    workspace = _pad_workspace(mins, maxs, workspace_margin)

    approach_dir = decide_approach_direction(obj_xyz, StretchApproachConfig(max_lift_z=stretch_max_lift_z))

    gp = GraspParams()
    gp.approach_direction = _vector3(_normalize(approach_dir))
    flat_cams: List[float] = []
    for (cx, cy, cz) in cam_positions:
      flat_cams.extend([float(cx), float(cy), float(cz)])
    gp.camera_position = flat_cams
    tx, ty, tz, qx, qy, qz, qw = cam_to_base
    gp.transform_camera2base = _transform_from_rt((tx, ty, tz), (qx, qy, qz, qw))
    gp.workspace = workspace
    gp.enable_approach_dir_filtering = bool(enable_approach_filter)
    gp.approach_dir_threshold = float(approach_threshold_deg)

    req = DetectGrasps.Request()
    req.cloud_indexed = cloud_indexed
    req.grasp_params = gp
    req.params_policy = params_policy

    future: Future = self.cli.call_async(req)
    rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
    if not future.done() or future.result() is None:
      raise RuntimeError("detect_grasps call failed or timed out")
    return future.result()

def main():
  rclpy.init()
  node = GpdConstrainedClient()
  # Example synthetic clouds
  env = np.random.uniform([-0.3,-0.3,0.65],[0.3,0.3,0.8], size=(5000,3)).astype(np.float32)
  obj = np.random.uniform([-0.05,-0.05,0.7],[0.05,0.05,0.75], size=(800,3)).astype(np.float32)
  frame_id = "base_link"
  cam_positions = [(0.5, 0.0, 1.2)]
  cam_to_base = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
  try:
    res = node.call_with_pcds(
      pcd_obj=obj, pcd_env=env, frame_id=frame_id,
      cam_positions=cam_positions, cam_to_base=cam_to_base,
      params_policy=DetectGrasps.Request.USE_REQUEST_PARAMS,
      approach_threshold_deg=25.0, workspace_margin=0.01,
      enable_approach_filter=True, stretch_max_lift_z=1.1,
    )
    node.get_logger().info(f"Got {len(res.grasp_configs.grasps)} grasps")
  except Exception as e:
    node.get_logger().error(f"Service call failed: {e}")
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
  main()
