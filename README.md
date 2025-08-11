# ROS 2 Wrapper for GPD

* [Author's website](http://www.ccs.neu.edu/home/atp/)
* [License](https://github.com/atenpas/gpd_ros/blob/master/LICENSE.md)
* [GPD library](https://github.com/rohitmenon86/gpd.git)

## Overview

A ROS 2 wrapper around the [GPD](https://github.com/rohitmenon86/gpd.git) package for detecting 6-DOF grasp poses for a
2-finger robot hand (e.g., a parallel jaw gripper) in 3D point clouds.

## 1) Installation

The following instructions have been tested on **Ubuntu 22.04** with ROS 2 (e.g., Humble, Iron). Similar
instructions should work for other ROS 2 distributions.

1. Install GPD. You can follow [these instructions](https://github.com/rohitmenon86/gpd.git#install). Make sure to run `make install` to install GPD as a library.

2. Clone this repository into the `src` folder of your ROS 2 workspace:

  ```bash
  cd <location_of_your_ros2_workspace>/src
  git clone https://github.com/rohitmenon86/gpd_ros2.git
  ```

3. Build your ROS 2 workspace using `colcon`:

  ```bash
  cd <location_of_your_ros2_workspace>
  colcon build --symlink-install
  ```

4. Source the workspace:

  ```bash
  source install/setup.bash
  ```
