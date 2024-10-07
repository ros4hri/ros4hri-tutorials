FROM osrf/ros:noetic-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-hri-rviz \
    evince \
    wget

# preinstall rosdeps extracted from source via 
# rosdep resolve -i  `rosdep keys -i  --from-paths . `
RUN apt-get update && apt-get install -y \
    python3-pil \
    python3-opencv \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-pybind11-catkin \
    ros-noetic-robot-state-publisher \
    ros-noetic-image-geometry \
    ros-noetic-diagnostic-msgs \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    python3-numpy \
    ros-noetic-hri-msgs \
    python3-tk \
    ros-noetic-rosconsole \
    ros-noetic-human-description \
    ros-noetic-catkin \
    python3-scipy \
    pybind11-dev \
    libdlib-dev \
    libblas-dev \
    liblapack-dev \
    ros-noetic-hri \
    ros-noetic-rospy

RUN rosdep update

RUN pip3 install mediapipe ikpy graphviz
