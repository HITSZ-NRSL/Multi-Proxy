//
// Created by jcwang on 25-3-11.
//

#ifndef MULTI_PROXY_LOOPDETECTOR_H
#define MULTI_PROXY_LOOPDETECTOR_H

#include "KeyFrame.h"
#include "STDesc/STDesc.h"
#include "utility.h"
#include <iostream>
#include <nav_msgs/Odometry.h>

extern ros::Publisher empty_pub;

struct DetectorConfig {
  STDConfigSetting config_setting;  // add std config
};

class LoopDetector {
private:
  int robot_id;
  STDescManager *std_manager_ptr;

public:
  LoopDetector(DetectorConfig config);
  bool makeDescriptorsSTD(pcl::PointCloud<PointType>::Ptr sub_map_temp, std::shared_ptr<KeyFrame> &keyFrame);

  /*new end*/
  void addHistoryDescriptorSTD(std::shared_ptr<KeyFrame> &keyFrame);
  bool searchCandidateFrameSTD(std::shared_ptr<KeyFrame> &keyFrame, PointTypePose &cur_p_in_ref_p_init, int &current_idx,
                               std::vector<std::pair<STDesc, STDesc>> &loop_std_pair);
  void setRobotID(int id) { robot_id = id;}
  void pubSTDPairs(const std::string& ref_robot_name, std::vector<std::pair<STDesc, STDesc>> &loop_std_pair, PointTypePose& cur_wrt_ref_map, const ros::Publisher &stdpair_pub);
  geometry_msgs::Point transPoint(tf::StampedTransform transform, geometry_msgs::Point p);
};


#endif//MULTI_PROXY_LOOPDETECTOR_H
