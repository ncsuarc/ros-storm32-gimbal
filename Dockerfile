FROM ros:humble
ENV ROS_DISTRO humble
WORKDIR /app
COPY --from=ghcr.io/ncsuarc/arcros_interface:main /app/install install
COPY . src/storm32_gimbal


# Install dependencies
RUN . /app/install/local_setup.sh && \
    apt-get update && rosdep update --rosdistro=$ROS_DISTRO && rosdep install -y \
      --ignore-packages-from-source \
      --from-paths \
        src/storm32_gimbal \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build
RUN . /app/install/local_setup.sh; colcon build

# Run
CMD ["./ros_entrypoint.sh"]
