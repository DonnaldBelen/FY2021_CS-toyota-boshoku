#
# ros12用Docker
#
# < Build方法 >
#       docker build -t ros12:0.1 -f Dockerfile .
#
# docker network create -d macvlan \
#   --subnet=192.168.11.0/24 \
#   --gateway=192.168.11.1 \
#   -o parent=enp3s0 mcv
#
# docker network create -d macvlan --subnet=192.168.12.0/24 -o parent=enx00051b6269ea mcv_autoware
# docker run --name ros12 -it --net=mcv -v /home/share:/root/share ros12:0.1 bash
# docker network connect mcv_autoware ros12
#
# □初回起動後
# ROS1系とROS2系ネットワークをmacvlanアタッチ
#
# □メッセージ定義追加
# 新規シェル起動(環境変数リセットのため)
# source /opt/ros/melodic/setup.bash
# mkdir -p ~/catkin_ws/src
# cd ~/catkin_ws/
# catkin_init_workspace src
# cd src
# git clone https://github.com/ros/common_msgs
# cd ..
# catkin_make
# 
# □bridge install
# 新規シェル起動(環境変数リセットのため)
# source /opt/ros/dashing/local_setup.bash
# source /opt/ros/melodic/setup.bash
# source ~/catkin_ws/devel/setup.bash
# mkdir -p ~/ros1_bridge_ws/src
# cd ~/ros1_bridge_ws/src
# git clone -b dashing https://github.com/ros2/ros1_bridge.git
# cd ..
# colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
#
# ROS1,2 bridge起動
# 新規シェル起動(環境変数リセットのため)
# source /opt/ros/melodic/setup.bash
# source /opt/ros/dashing/setup.bash
# source ~/catkin_ws/devel/setup.bash
# source ~/ros1_bridge_ws/install/setup.bash
#
# export ROS_MASTER_URI=http://192.168.12.73:11311
# export ROS_IP=192.168.12.11
#
# ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
# 
#
# 【websocketまでの導通テスト】
# ws://192.168.11.73:9090
# {
# "op": "subscribe",
# "type":"a",
# "topic": "/rpark_pose"
# }
#

FROM ubuntu:18.04
LABEL maintainer="TB <tb@toyota-boshoku.com>"

RUN cat /etc/apt/sources.list
RUN sed -i'' 's/archive.ubuntu.com/jp.archive.ubuntu.com/' /etc/apt/sources.list
RUN cat /etc/apt/sources.list

RUN apt-get update
RUN apt-get install -y wget

# install X11
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y x11-apps xterm

# Install 日本語環境
RUN apt-get install -y language-pack-ja-base language-pack-ja
RUN locale-gen ja_JP.UTF-8
ENV LANG ja_JP.utf8

ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype

RUN apt install -y curl net-tools git vim usbutils lsb-release

RUN apt-get install -my wget gnupg

# ROS1インストール
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update

RUN apt install -y ros-melodic-desktop-full

RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# ROS2インストール
RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt update

RUN apt install -y ros-dashing-desktop
RUN apt install -y python3-colcon-common-extensions
RUN apt install -y python3-argcomplete

# 初期設定
RUN apt-get install -y python-pip
RUN pip install -U rosdep
RUN rosdep init
RUN rosdep update

RUN apt install -y ros-dashing-launch*


# コンテナのタイムゾーンがデフォルトでUTCになっているので、ホストOSの/etc/localtimeを読み込み専用でマウント
VOLUME /etc/localtime:/etc/localtime:ro

WORKDIR /root/share/

CMD ["bash"]
