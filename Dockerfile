# Define the ROS distribution as a build argument
ARG ROS_DISTRO=kilted

FROM ros:${ROS_DISTRO}-ros-base

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Default libraries to handle PEP 668
RUN apt-get update && \
    apt-get install -y \
    python3-venv \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

# Default libraries to handle PEP 668
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth=1 https://github.com/OpenFinderBot/interface_definitions.git /ofb_ws/src/interface_definitions

COPY . /ofb_ws/src/image_processing_node/

RUN cd /ofb_ws \
    && python3 -m venv /opt/venv --system-site-packages \
    && . /opt/venv/bin/activate \
    && for i in $(find . -name "*dependencies.txt"); do pip3 install -r "$i"; done \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

# Set entrypoint
RUN chmod +x /ofb_ws/src/image_processing_node/entrypoint.sh
CMD ["/bin/bash", "/ofb_ws/src/image_processing_node/entrypoint.sh"]
