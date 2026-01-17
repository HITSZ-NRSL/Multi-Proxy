#!/bin/bash

# 获取ROS包路径
ros_package_path="$(rospack find multi_proxy)"

# 要删除的文件夹相对路径
relative_folders=("pcd/robot_0" "pcd/robot_1" "pcd/robot_2")

# 构建绝对路径并遍历文件夹删除*.pcd文件
for relative_folder in "${relative_folders[@]}"; do
  folder="$ros_package_path/$relative_folder"
  if [ -d "$folder" ]; then
    echo "删除文件夹中的*.pcd文件：$folder"
    find "$folder" -type f -name "*.pcd" -exec rm -f {} \;
  else
    echo "文件夹不存在：$folder"
  fi
done
