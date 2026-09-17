# Reference build for the ros_noetic_ready image used by ros_desktop and every
# ros_client_<username> container. Verify against your working image before
# relying on it (docker exec ros_desktop rospack list | sort > packages.txt).
#
#   docker build --platform linux/amd64 -t ros_noetic_ready:latest docker/

FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-rosbridge-suite \
      ros-noetic-turtlebot3 \
      ros-noetic-turtlebot3-simulations \
      ros-noetic-navigation \
      ros-noetic-map-server \
      ros-noetic-universal-robots \
      ros-noetic-ur-gazebo \
      ros-noetic-joint-trajectory-controller \
      ros-noetic-effort-controllers \
      python3-pip nano vim less procps sudo \
    && pip3 install --no-cache-dir roslibpy \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

WORKDIR /root
CMD ["sleep", "infinity"]
