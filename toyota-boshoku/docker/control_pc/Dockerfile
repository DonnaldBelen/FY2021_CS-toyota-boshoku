#
# control_pc用Docker
#
# < Build方法 >
#       docker build -t control_pc:0.1 -f Dockerfile .
#
# docker network create -d macvlan \
#   --subnet=192.168.11.0/24 \
#   --gateway=192.168.11.1 \
#   -o parent=enp3s0 mcv
#
#export DISPLAY=:0
#xhost +
#nvidia-docker run --name control_pc --net=mcv --ip=192.168.11.73 -it -v=/dev:/dev -v /home/share:/root/share -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v $HOME/.Xauthority:/root/.Xauthority:rw --env="QT_X11_NO_MITSHM=1" --privileged control_pc:0.1 bash
#
#
#
# image
FROM nvidia/cuda:10.0-cudnn7-runtime-ubuntu18.04

# install X11
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y x11-apps xterm

# Install 日本語環境
RUN apt-get install -y language-pack-ja-base language-pack-ja
RUN locale-gen ja_JP.UTF-8
ENV LANG ja_JP.utf8

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install GLX-Gears
RUN apt update && apt install -y --no-install-recommends mesa-utils x11-apps && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y curl net-tools

# openCV
ENV OPENCV_VERSION='4.1.2'
RUN apt-get -y update
RUN apt-get install -y build-essential cmake
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y tzdata
RUN apt-get install -y qt5-default libvtk6-dev
RUN apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev
RUN apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev
RUN apt-get install -y libtbb-dev libeigen3-dev
RUN apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy
RUN apt-get install -y ant default-jdk
RUN apt-get install -y doxygen
RUN apt-get install -y unzip wget
#RUN apt-get install -y libjasper-dev
RUN echo "a"
RUN wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz
RUN tar zxvf ${OPENCV_VERSION}.tar.gz
RUN mv opencv-${OPENCV_VERSION} OpenCV


WORKDIR /OpenCV
RUN wget https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.tar.gz
RUN tar zxvf ${OPENCV_VERSION}.tar.gz
RUN mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

RUN cd /OpenCV && mkdir build

WORKDIR "/OpenCV/build/"
RUN cmake  –DWITH_VTK=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON \
      -DWITH_XINE=ON -DBUILD_EXAMPLES=ON -DENABLE_PRECOMPILED_HEADERS=OFF \
      -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ..
RUN make -j4
RUN make install
RUN ldconfig

ENV CPATH=$CPATH:/usr/local/include
ENV LIBRARY_PATH=$LIBRARY_PATH:/usr/local/lib
ENV CPATH=$CPATH:/usr/local/include/opencv4

RUN apt-get install -y git vim

RUN apt install -y python3-pip python3-dev
ENV PYTHONIOENCODING "utf-8"
RUN pip3 install --upgrade pip
RUN pip install bottle msgpack mysql-connector-python-rf configparser requests websocket-client

RUN apt install -y usbutils

# END

# WORKDIR "/root/share"

# ENTRYPOINT /bin/bash

# ros2
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl net-tools git build-essential vim sudo lsb-release locales bash-completion tzdata && \
    apt-get install -yq python3-pip python3-dev && \
    rm -rf /var/lib/apt/lists/*
RUN useradd -m -d /home/ubuntu ubuntu -p $(perl -e 'print crypt("ubuntu", "salt"),"\n"') && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
ENV ROS_DISTRO dashing
COPY ./run.sh /
RUN /run.sh && \
    sudo rm -rf /var/lib/apt/lists/*

RUN apt-get update
RUN apt-get install -y ros-dashing-rosbridge-server

RUN apt-get install -y ros-$ROS_DISTRO-ros2bag ros-$ROS_DISTRO-rosbag2-transport
RUN apt-get install -y ros-$ROS_DISTRO-rosbag2-storage-default-plugins

### kinect

### ↓↓↓ここから　Azure-Kinect-Sensor-SDK/scripts/dockerのコピー↓↓↓
## https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git

# Set the working directory to /app
WORKDIR /app

ARG ARCH=amd64

ADD setup-ubuntu.sh /app
ADD sources.list /app

# Tell Ubuntu non-interactive install
ARG DEBIAN_FRONTEND=noninteractive


RUN ./setup-ubuntu.sh ${ARCH}

RUN apt-get install -y ca-certificates
RUN update-ca-certificates

### ↑↑↑ここまで Azure-Kinect-Sensor-SDK/scripts/dockerのコピー↑↑↑

ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype

RUN echo "yes" | apt-get -y install libk4a1.3-dev
RUN apt-get -y install k4a-tools=1.3.0
RUN echo "yes" | apt-get -y install libk4abt1.0-dev

# selenium
RUN pip install selenium
RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
RUN dpkg -i google-chrome-stable_current_amd64.deb; exit 0
RUN apt update
RUN apt -f install -y
RUN apt install unzip
RUN wget https://chromedriver.storage.googleapis.com/2.44/chromedriver_linux64.zip
RUN unzip chromedriver_linux64.zip -d /usr/bin/

#追加モジュール
RUN pip install git+https://github.com/Pithikos/python-websocket-server
RUN pip install mutagen pygame paste ntplib

# cloud
RUN pip install waitress matplotlib ntplib seaborn
RUN apt-get install -y alsa-utils kmod alsa sox
RUN apt install -y iputils-ping

ENV AUDIODEV=dmixer

# 最新(1.9.5)はmp3再生に失敗する
RUN pip install pygame==1.9.4

WORKDIR /root/
COPY .asoundrc .asoundrc

WORKDIR /etc/modprobe.d/
COPY alsa-base.conf alsa-base.conf

# コンテナのタイムゾーンがデフォルトでUTCになっているので、ホストOSの/etc/localtimeを読み込み専用でマウント
VOLUME /etc/localtime:/etc/localtime:ro


WORKDIR "/root/share"
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
