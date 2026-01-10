# multi-proxy

This repository will host the official implementation of the paper:

**Communication-Robust Asynchronous Distributed LiDAR Collaborative Smoothing and Mapping**

## Overview
`multi-proxy` focuses on communication-robust, asynchronous distributed and decentralized LiDAR collaborative smoothing and mapping.  
The project targets multi-robot scenarios where communication is constrained, and aims to maintain consistent and accurate mapping performance.

Note that the algorithm itself does not provide odometry; you must supply your own front-end LiDAR odometry. Alternatively, you may pre-record odometry and point-cloud data from an LIO (LiDAR–Inertial Odometry) or LO (LiDAR Odometry) system.

multi-proxy subscribes to point clouds of type sensor_msgs::PointCloud2 and odometry of type nav_msgs::Odometry, and associates them into keyframes by nearest timestamps, thereby achieving complete decoupling from the front-end LiDAR odometry.

## Status
- 📄 Paper accepted  
- 🚧 Code is under preparation  
- ⏳ Code will be released soon

## Planned Contents
- Core algorithms for asynchronous distributed LiDAR smoothing and mapping  
- Communication-robust collaboration mechanisms  
- Experiment scripts to reproduce paper results  
- Evaluation and visualization tools  
