//
// Created by jcwang on 24-10-19.
//

#ifndef MULTI_PROXY_KEYFRAMELIST_H
#define MULTI_PROXY_KEYFRAMELIST_H

#include "KeyFrame.h"
#include "utility.h"
#include <pcl/filters/random_sample.h>

class KeyFrameList {
private:
  std::mutex mtx_keyframe_list;
  std::mutex mtx_map;
  std::vector<std::shared_ptr<KeyFrame>> keyframe_list;

public:
  pcl::PointCloud<PointType>::Ptr lidarmap_current;
  std::deque<int> descriptor_dis;
  KeyFrameList();

  void addKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
  std::shared_ptr<KeyFrame> at(int index);
  std::shared_ptr<KeyFrame> back();

  int size();

  bool buildMapNB(int frame_id, const pcl::PointCloud<PointType>::Ptr& pointCloud, int frame_num = 10);
  bool buildMapWrtLocalMap(int frame_id, const pcl::PointCloud<PointType>::Ptr& pointCloud, int frame_num);
  int getKeyFrameId(int frame_id);
  std::shared_ptr<KeyFrame> getKeyFrame(int frame_id);
  bool empty();

  pcl::PointCloud<PointType>::Ptr getMap();

  void setMap(const pcl::PointCloud<PointType>::Ptr& pointCloud);

  void buildMap(const pcl::PointCloud<PointType>::Ptr &pointCloud);

  void rebuildMap();
  pcl::PointCloud<PointType>::Ptr rebuildMap(double voxel_size);
  void resetFlag();
  void resetPose();
  double getListSize();
  void clearPointCloud();
  float getMaxError();
  bool getOptimizedTraj(int robot_id, std::vector<PointTypePose> &poses);
  bool getInitialTraj(int robot_id, std::vector<PointTypePose> &poses);
  bool getTrajInCurrentOdom(int robot_id, std::vector<PointTypePose> &poses);
  int getNearestFrameIdByTime(double time, int robot_id);

  PointTypePose getInterpolatePoseByTime(double time, int robot_id, int frame_id);

};


#endif //MULTI_PROXY_KEYFRAMELIST_H
