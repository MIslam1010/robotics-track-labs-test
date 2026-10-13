# ROS 2 Jazzy desktop (RViz) + the course Python packages, for the labs with ROS 2 parts.
#   docker build -t sbeg46x-labs-ros2 -f docker/ros2/Dockerfile .
#   Linux:   docker run -it --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD":/work sbeg46x-labs-ros2
#   macOS/Windows: use VS Code "Reopen in Container" with .devcontainer/ros2, or the lab PCs for RViz windows.
FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-venv build-essential cmake libeigen3-dev git ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt
WORKDIR /work
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
CMD ["bash"]
