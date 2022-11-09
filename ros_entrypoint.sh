set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
ros2 launch storm32_gimbal gimbal_launch.py