FROM autoware/autoware:1.7.0-kinetic
# 参考：https://qiita.com/Leonardo-mbc/items/cfd38a4fae8667593cf1
# 
# docker build -t ros-listener:0.1 -f Dockerfile .
# docker run --name ros-listener -it -v /home/share:/root/share -v /ramdisk:/ramdisk ros-listener:0.1 bash
#
# docker run -it --name ros-listener --env ROS_HOSTNAME=mooc_control_pc --env ROS_MASTER_URI=http://192.168.1.10:11311 -v /home/share:/root/share -v /ramdisk:/ramdisk ros-listener:1.0 bash

# Dockerfile作成者
MAINTAINER toyota-boshoku

# パッケージのインストールとアップデート
RUN apt-get update && apt-get install -y
RUN apt install -y wget curl
RUN apt-get install -y net-tools vim git

RUN apt-get install -y language-pack-ja-base language-pack-ja
RUN locale-gen ja_JP.UTF-8
RUN echo "export LANG=ja_JP.UTF-8" >> .bashrc

# python3.6
# RUN apt-get install -y software-properties-common
# RUN add-apt-repository ppa:jonathonf/python-3.6
# RUN apt-get update
# RUN apt-get install -y python3.6 python3.6-dev
# RUN wget https://bootstrap.pypa.io/get-pip.py
# RUN python3.6 get-pip.py
# RUN ln -sf /usr/bin/python3.6 /usr/bin/python3

# python3.5
# RUN apt install -y python3-pip python3-dev

RUN apt install -y python-pip

ENV PYTHONIOENCODING "utf-8"

#RUN pip install --upgrade pip

RUN pip install numpy

# Autoware
RUN pip install paho-mqtt==1.3.1

# 指定はbson==0.5.2ですが入らないので
RUN pip install bson==0.5.5
RUN pip install ntplib==0.3.3
RUN pip install pyyaml

RUN apt-get install -y python-yaml

#RUN pip install rospkg catkin_pkg

# コンテナのタイムゾーンがデフォルトでUTCになっているので、ホストOSの/etc/localtimeを読み込み専用でマウント
VOLUME /etc/localtime:/etc/localtime:ro

ENTRYPOINT /bin/bash

# install ros tutorials packages
# RUN apt-get install -y ros-kinetic-ros-tutorials \
#     ros-kinetic-common-tutorials \
#     && rm -rf /var/lib/apt/lists/
