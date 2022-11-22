#IMG-NAME: crawler-ros-img
FROM arm32v7/ubuntu:jammy
SHELL ["/bin/bash", "-c"]
ENV TZ=America/Sao_Paulo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

WORKDIR /root

#COPY compiled_opencv.tar.bz2 /root/compiled_opencv.tar.bz2

RUN apt-get update
RUN apt-get dist-upgrade

RUN apt-get install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
ENV DEBIAN_FRONTEND=noninteractive
#RUN apt-get install -y ros-humble-desktop
RUN apt-get install -y ros-galactic-desktop
RUN apt-get install -y python3-colcon-common-extensions

# C++ compile tools
RUN apt-get install -y build-essential cmake pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    sudo libgtk2.0-dev

# OpenCV (pre built)
RUN apt-get install bzip2
RUN tar jxvf compiled_opencv.tar
RUN cd tf/opencv/build && make install

# Gstream
RUN apt-get install -y v4l-utils \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    ubuntu-restricted-extras
    
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev    
RUN apt-get install -y gir1.2-gst-rtsp-server-1.0

# OMPL
RUN curl -sSL https://ompl.kavrakilab.org/install-ompl-ubuntu.sh  > install-ompl-ubuntu.sh
RUN chmod a+x ./install-ompl-ubuntu.sh
RUN sudo ./install-ompl-ubuntu.sh --python
RUN rm ./install-ompl-ubuntu.sh