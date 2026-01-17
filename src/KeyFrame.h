//
// Created by jcwang on 24-10-19.
//

#ifndef MULTI_PROXY_KEYFRAME_H
#define MULTI_PROXY_KEYFRAME_H

#include "STDesc/STDesc.h"
#include "utility.h"

#include <std_msgs/Int8MultiArray.h>
#include <gtsam/sam/RangeFactor.h>
#include <iostream>
#include <ros/serialization.h>

struct CandidateInfo {
  std::string robotName;
  int robotID;
  int candidateFrameId;
  int loopFrameId;
  PointTypePose pose;
  PointTypePose initial_guess;
  sensor_msgs::PointCloud2 scanCloud;
};

class KeyFrame {

  PointTypePose pose_init;
  unsigned int check_count = 0;
  unsigned int skip_num = 1;
public:
  std::string robot_name = "robot";
  int robot_id = 0;
  int pair_robot_id = 0;
  int frame_id = 0;
  bool get_pose_decentralized = false;
  bool has_optimized = false;

  bool is_degenerate = false;
  bool is_corrected = false;

  bool is_key_frame = false;
  bool is_checkdes = false;

  std_msgs::Header cloud_header;
  std::vector<std::pair<PointTypePose, PointTypeNormal>> degenerateData;
  double time;

  PointTypePose decentralized_pose_in_cur_map;
  PointTypePose distributed_pose_in_cur_map;

  pcl::PointCloud<PointType>::Ptr cloud;

  pcl::PointCloud<PointTypeNormal>::Ptr std_corner;

  std::vector<STDesc> stdesc;
  gtsam::Matrix noiseMatrix = gtsam::I_6x6;
  Eigen::Matrix3f posConstraintMatrix = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f oriConstraintMatrix = Eigen::Matrix3f::Identity();

  std::map<int, std::deque<sensor_msgs::Range::ConstPtr>> uwb_map{};
  std::map<int, PointTypePose> candidate_inter_loop;



  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  KeyFrame() = default;
  KeyFrame(std::string robot_name_init, int id, int frame = 0);

  KeyFrame(const KeyFrame& contextWithPose_in);

  KeyFrame(const CandidateInfo& candidate_info_input);
  KeyFrame(const std_msgs::Int8MultiArray& descriptor_msg);

  ~KeyFrame();


  void publishDescriptorStream(const ros::Publisher &thisPub, std::ofstream &log_file);

  void publishCandidateStream(const ros::Publisher &thisPub, std::ofstream &log_file, pcl::PointCloud<PointType>::Ptr &cloud_in, int loop_frame_id = -1,
                            PointTypePose trans = PointTypePose());

  pcl::PointCloud<PointType>::Ptr transformPointCloud(PointTypePose &transformIn);

  static pcl::PointCloud<PointType>::Ptr transformPointCloud(const pcl::PointCloud<PointType>::Ptr& cloudIn, PointTypePose &transformIn);

  bool checkCount() {
    check_count = ++check_count % skip_num;
    return check_count == 0;} //

  void punishment() {skip_num < 16 ? skip_num++ : skip_num = 16;}

  bool checkRefFrameId(int frame_id_input);
  void setRefFrameId(int frame_id_input, PointTypePose pose_in_ref_frame);
  PointTypePose getPoseOptimized() { distributed_pose_in_cur_map.time = time; distributed_pose_in_cur_map.intensity = frame_id; return distributed_pose_in_cur_map; }
  PointTypePose getPoseInit() { pose_init.time = time; pose_init.intensity = frame_id; return pose_init; }
  void setPoseInit(PointTypePose pose_init_in) { pose_init = pose_init_in; }
  void publishSTD(const ros::Publisher &std_publisher);
  void clearPointCloud();
  double getDataSize();

  template<typename T>
  double rosTime(const T &stamp) { return ros::Time(stamp).toSec(); }

//  KeyFrame& operator=(const KeyFrame& contextWithPose_in);
};


#endif //MULTI_PROXY_KEYFRAME_H
