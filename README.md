# AGV ROS2 Workspace

Monorepo workspace untuk AGV:
- agv_bringup: launch utama
- agv_description: URDF + TF static
- agv_configs: parameter YAML (EKF, SLAM, dll)
- agv_tools: tools (rosbag scripts, helper)

Build:
- cd ~/agv_ws
- colcon build --symlink-install
- source install/setup.bash
