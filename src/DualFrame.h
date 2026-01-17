#ifndef DUALFRAME_H
#define DUALFRAME_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <map>
#include <set>
#include "KeyFrameList.h"
#include "utility.h"
#include <fstream>
#include <iostream>
#include <sstream>

class DualFrame {
public:
  int robot_id;
  bool updated = true;
  gtsam::Pose3 pose;
  gtsam::Pose3 pose_error;
  uint16_t update_cnt = 0;

  DualFrame(){};
  DualFrame(int robot_id, const gtsam::Pose3& pose_in):
      robot_id(robot_id), pose(pose_in){};
};

class DualFrameList {
public:
  bool valid = false;
  bool regenerate = false;
  int robot_id;
  int ref_robot_id;
  int debug_node_id = -1;
  std::map<int32_t, DualFrame> dual_frames_local;
  std::map<int32_t, DualFrame> dual_frames_remote;

  double ita = 1;
  double min_error = 0.005;
  uint16_t max_iters = 20;
  int max_keyframes = 10000;

  DualFrameList();

  void setRobotID(int r_id);
  void setRefRobotID(int r_id) {ref_robot_id = r_id;}
  void setMaxInters(uint16_t max_iters_in) {
    std::cout << "Robot " << robot_id << " setting Max Inters: " << max_iters_in << std::endl;
    max_iters = max_iters_in;
  }
  void setMaxKeyFrames(int max_frame) {
    max_keyframes = max_frame;
  }
  void setMinError(double min_error_in) {
    std::cout << "Robot " << robot_id << " setting Min Error: " << min_error_in << std::endl;
    min_error = min_error_in;
  }
  bool update_remote(int32_t frame_id, DualFrame dual_frame);
  bool add_local(int32_t frame_id, DualFrame dual_frame);
  void clear_local_dual(int32_t frame_id) {
    dual_frames_local.erase(frame_id);
  }
  void clear_remote_dual(){
    dual_frames_remote.clear();
  }
  void clear_local_dual(){
    dual_frames_local.clear();
    regenerate = true;
  }
  void local_update(KeyFrameList &curr_frames, KeyFrameList &ref_frames,
                    const std::unordered_map<int, PointTypePose>& global_maps);
  bool saveFrames(int rID, int target, std::unordered_map< int, PointTypePose> global_maps, bool trunc=true);

  void reset_curr_update_cnt();
};

#endif
