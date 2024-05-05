# FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_bcf535ea3b9d16a854aaeb1701ab5a86
# FROM ros:humble
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

RUN apt-get update

# Install ROS2 
ARG distro="humble"
ENV ROS_ROOT=/opt/ros/$distro/

WORKDIR $ROS_ROOT

ENV HOST 0.0.0.0

# Install necessary software for the installation of ROS2
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \ 
                      locales \
                      curl \
                      vim \
                      gnupg2 \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

RUN locale-gen ja_JP ja_JP.UTF-8 \
    && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
    && export LANG=ja_JP.UTF-8

RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http:packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt update

ENV ROS_DISTRO $distro
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2
RUN apt-get install -y ros-$ROS_DISTRO-desktop \
                python3-colcon-common-extensions \
                python3-rosdep \
                python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*rm 

# Initialize rosdep
RUN rosdep init && rosdep update

# Custom stuff
ENV WS /root/ros2_ws

# Unless otherwise specified, we make a fat build.
ARG CUDA_DOCKER_ARCH=all
# Set nvcc architecture
ENV CUDA_DOCKER_ARCH=${CUDA_DOCKER_ARCH}
# Enable cuBLAS
ENV WHISPER_CUBLAS=1
ENV WHISPER_CUDA=1

ENV CUDAToolkit_ROOT=/usr/local/cuda/compat
# Ref: https://stackoverflow.com/a/53464012
ENV LD_LIBRARY_PATH /usr/local/cuda/compat:$LD_LIBRARY_PATH
RUN ldconfig


# TODO: libsdl is not installed by rosdep automatically
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

RUN bash -c "rosdep install --from-paths src --ignore-src -r -y"
RUN echo "source $WS/install/setup.bash" >> ~/.bashrc

RUN apt-get install -y ros-$ROS_DISTRO-foxglove-bridge



# RUN apt-get install -y git build-essential \
#     python3 python3-pip gcc wget \
#     ocl-icd-opencl-dev opencl-headers clinfo \
#     libclblast-dev libopenblas-dev \
#     && mkdir -p /etc/OpenCL/vendors && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

RUN bash -c "source $ROS_ROOT/setup.bash && colcon build --symlink-install --executor parallel"
# 164.6 /usr/bin/ld: warning: libcuda.so.1, needed by whisper_cpp/libwhisper.so.1.5.5, not found (try using -rpath or -rpath-link)


# RUN python3 -c 'import torch; print(torch.cuda.device_count())'


