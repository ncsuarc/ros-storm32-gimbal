set -e

# setup ros environment
source "/app/install/local_setup.sh"
ros2 launch storm32_gimbal gimbal_launch.py
