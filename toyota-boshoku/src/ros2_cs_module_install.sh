#!/bin/bash
# Usage：source ros2_cs_module_install.sh
# ビルド除外したいものは対象パッケージにCOLCON_IGNOREを入れる
# .gitignoreに追加してあるのでコミットはされない
# 例：touch ros2_cs/src/moox_azure_kinect/COLCON_IGNORE

cd ros2_cs
colcon build  --parallel-workers 24 --symlink-install --packages-skip-build-finished
source ./install/setup.bash
cd ../
