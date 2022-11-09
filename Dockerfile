FROM ros:foxy
ENV ROS_DISTRO foxy
WORKDIR /app
COPY . .

# Install dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep update && rosdep install -y \
      --from-paths \
        . \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; colcon build'

# Run
CMD ["./ros_entrypoint.sh"]
