# FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_bcf535ea3b9d16a854aaeb1701ab5a86
# FROM ros:humble
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

# Install ROS2 
ARG distro="humble"
ENV ROS_ROOT=/opt/ros/$distro/
ENV ROS_DISTRO $distro
ENV DEBIAN_FRONTEND=noninteractive
ENV HOST 0.0.0.0
ENV WS /root/ros2_ws
ENV WHISPER_CUDA=1
ENV LD_LIBRARY_PATH /usr/local/cuda/compat:$LD_LIBRARY_PATH

WORKDIR $ROS_ROOT

# Install ROS2
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y locales curl vim gnupg2 lsb-release && \
    locale-gen ja_JP ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 &&\
    export LANG=ja_JP.UTF-8 && \
    curl http://repo.ros2.org/repos.key | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64] http:packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && \
    apt-get install -y ros-$ROS_DISTRO-desktop \
                python3-colcon-common-extensions \
                python3-rosdep \
                python3-argcomplete \
                ros-$ROS_DISTRO-foxglove-bridge && \
    rosdep init && rosdep update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*rm 


# AI Stuff

RUN apt-get update && \
    apt-get install -y \
        portaudio19-dev \
        libsdl-kitchensink-dev \
        python3-pip \
        build-essential \
        ffmpeg \
        && rm -rf /var/lib/apt/lists/*rm 

WORKDIR $WS
ADD requirements.txt ./
RUN python3 -m pip install -r requirements.txt

ADD src $WS/src

RUN echo "source $WS/install/setup.bash" >> ~/.bashrc

RUN bash -c "rosdep install --from-paths src --ignore-src -r -y"
RUN bash -c "source $ROS_ROOT/setup.bash && colcon build --symlink-install"

