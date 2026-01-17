# multi-proxy

This repository will host the official implementation of the paper:

**Communication-Robust Asynchronous Distributed LiDAR Collaborative Smoothing and Mapping**

## Overview
`multi-proxy` focuses on communication-robust, asynchronous distributed and decentralized LiDAR collaborative smoothing and mapping.  
The project targets multi-robot scenarios where communication is constrained, and aims to maintain consistent and accurate mapping performance.

Note that the algorithm itself does not provide odometry; you must supply your own front-end LiDAR odometry. Alternatively, you may pre-record odometry and point-cloud data from an LIO (LiDAR–Inertial Odometry) or LO (LiDAR Odometry) system.

multi-proxy subscribes to point clouds of type sensor_msgs::PointCloud2 and odometry of type nav_msgs::Odometry, and associates them into keyframes by nearest timestamps, thereby achieving complete decoupling from the front-end LiDAR odometry.

## Demo Video
[![Demo Video](doc/output.gif)](https://www.bilibili.com/video/BV1Lk6mBhErU)

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Noetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (tested with 4.2.0)
  ```bash
  curl -L -o gtsam-4.2.0.tar.gz https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.tar.gz
  tar -xzf gtsam-4.2.0.tar.gz
  rm gtsam-4.2.0.tar.gz && cd gtsam-4.2.0 && mkdir build && cd build && \
  cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_TBB=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES=OFF && \
  make -j$(nproc) && make install && ldconfig
  ```

The system can be run either in a Docker environment or locally. **Docker is recommended**.

## Docker

Install Docker and docker-compose (installation methods and URLs may change; please install them yourself and add your user to the Docker group to enable running Docker without `sudo`). After installation, build the image:

```bash
  docker build -t sylaranh/multi-proxy:latest .
```
After the image is built, you can check it using:
```bash
  docker images
```
To evaluate the impact of inter-robot communication quality, navigate to the docker directory and launch three containers using docker-compose. Each container represents one robot. Network delay, packet loss, and bandwidth constraints are automatically injected between containers via net_work_loss_robot_0.sh.
```bash
  xhost +
  cd <your_workspace>/src/multi_proxy/docker/
  docker-compose up
```
Start ROS Master (Host)

Start roscore on the host machine:
```bash
  roslaunch src/multi_proxy/launch/multi_s3e_master.launch
```
Launch Robots in Docker Containers
Open a new terminal and enter the multi-proxy0 container:
```bash
  docker exec -it multi-proxy0 bash
  cd ws_code
  catkin_make
  source devel/setup.bash
  roslaunch multi_proxy multi_s3e0.launch
```
Open another terminal and enter the multi-proxy1 container:
```bash
  docker exec -it multi-proxy1 bash
  cd ws_code
  catkin_make
  source devel/setup.bash
  roslaunch multi_proxy multi_s3e1.launch
```
Open another terminal and enter the multi-proxy2 container:
```bash
  docker exec -it multi-proxy2 bash
  cd ws_code
  catkin_make
  source devel/setup.bash
  roslaunch multi_proxy multi_s3e2.launch
```

Using Pre-recorded Odometry and Pointcloud
```bash
rosbag play S3E_College_robot_odom.bag
```

If Single robot LIO is not configured, instead use a merged single-robot odometry dataset that has been precomputed (e.g., using FastLIO). Typical topics include:
/robot_0/odometry
/robot_0/cloud_deskewed

A common workflow is:
Play each robot’s dataset independently.
Add robot name prefixes to the odometry and cloud topics.
Merge all robots’ lio output into a single rosbag.

## Status
- 📄 Paper accepted
- 🚧 Datasets is under preparation