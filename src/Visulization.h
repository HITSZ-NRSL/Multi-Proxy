//
// Created by jcwang on 25-9-7.
//

#ifndef MULTI_PROXY_VISULIZATION_H
#define MULTI_PROXY_VISULIZATION_H

#include "utility.h"
#include "DualFrame.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

//msg
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Visulization {
  int robot_id_th = 0;
  std::string robot_name = "alpha";
  KeyFrameList *cur_info_list;
  KeyFrameList *ref_info_list;
  std::string log_folder = std::string(ROOT_DIR) + "log";

public:
  ros::Publisher pub_dual_viz, pub_key_frame_node, pub_loop_constraint_edge, pub_fused_map_frame;
  void publishMapGlobalCurrent(ros::Time time_lidar_stamp, int ref_robot_id, PointTypePose current2global);
  void visualizeLoopClosure(ros::Time time_lidar_stamp, std::vector<LoopClosure> &accepted_loops);
  void visualizeKeyFrame(ros::Time time_lidar_stamp);
  void visualizeDualFrames(ros::Time time_lidar_stamp, std::unordered_map< int, DualFrameList> &dual_lists);
  void setRobotID(int id) {robot_id_th = id;}
  void setRobotName(const std::string &name) {robot_name = name;}
  void setCurKeyFrameList(KeyFrameList *list) {cur_info_list = list;}
  void setRefKeyFrameList(KeyFrameList *list) {ref_info_list = list;}
  void setLogFolder(const std::string &folder) {log_folder = folder;}


};


#endif //MULTI_PROXY_VISULIZATION_H
