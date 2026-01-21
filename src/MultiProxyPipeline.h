//
// Created by jcwang on 25-9-7.
//

#ifndef MULTI_PROXY_MULTIPROXYPIPELINE_H
#define MULTI_PROXY_MULTIPROXYPIPELINE_H

#include "utility.h"
#include "KeyFrame.h"
#include "KeyFrameList.h"
#include "pcManager.h"
#include "LoopDetector.h"
#include "MultiOutlierRejection.h"
#include "BackEnd.h"
#include "Visulization.h"

#include "multi_proxy/save_traj.h"

class MultiProxyPipeline
{
  ros::NodeHandle node_handler; // global node handler for publishing everthing
  ros::Subscriber _sub_laser_cloud_info;
  ros::Subscriber _sub_localization_info;
  ros::Subscriber _sub_descriptor_info;
  ros::Subscriber _sub_candidate_info;
  ros::Subscriber _sub_verified_info;
  ros::Subscriber _sub_dual_info;
  ros::Subscriber _sub_heartbeat;
  ros::Publisher _pub_heartbeat;
  //    ros pub for stdmatchdebug
  ros::Publisher _pub_STD_current;
  ros::Publisher _pub_STD_received;
  ros::Publisher _pub_STD_matched;
  ros::ServiceServer _srv_save_traj;
  ros::ServiceServer _srv_debug_node;
  ros::ServiceServer _srv_debug_loop;
  ros::Timer _timer_heartbeat;
  ros::Time _time_lidar_stamp;

  std::map<int, RobotID> _neighbor_id_list;

  std::string _local_odom_topic;
  std::string _local_cloud_topic;

  bool _use_ARock = false;
  bool _use_bag_time = true;
  bool _save_pose_and_map = false;
  bool _key_frame_only = false;

  int _max_keyframes = 10000;
  int _max_robots = 10;

  int _pcm_start_threshold{};
  int _pcm_max_clique_size{};
  int _dual_max_iters{};

  int _current_frame_count{};
  int _icp_maxiter = 50;
  int _verification_points = 5000;

  float _downsample_resolution_map = 0.2;
  float _save_map_resolution = 0.1;
  float _icp_thres{};
  float _icp_disthr{};
  float _rebuild_thres = 0.01;
  double _pcm_thres{};
  double _time_th{};
  double _distance_th{};
  double _angle_th{};
  double _s2m_std = 1e-8;
  double _odom_std = 1e-8;
  double _inter_loop_std = 1e-1;
  double _intra_loop_std = 1e-1;
  double _dual_std = 1e-1;
  double _dual_min_error = 0.005;

  std::mutex _mtx_descriptor_info_list_waiting;
  std::mutex _mtx_candidate_verification;
  std::mutex _mtx_loop_queue;
  std::mutex _mtx_dual_frame;

  KeyFrameList _cur_info_list;
  KeyFrameList _ref_info_list;

  std::vector<KeyFrame> _descriptor_info_list_waiting;
  std::deque<CandidateInfo> _candidate_loop_info_list;

  std::unordered_map<int, DualFrameList> _dual_frames;
  std::unordered_map<int, std::vector<LoopClosure>> _loop_queue;

  DetectorConfig _detector_config;
  pcManager *_pc_manager{};
  LoopDetector *_loop_detector{};
  MultiOutlierRejection *_multi_outlier_rejection{};
  BackEnd *_back_end{};
  Visulization _visulization;

  std::string _pcm_matrix_folder;
  std::string _log_folder;
  std::string _pcd_folder;

  std::ofstream _odom_file;
  std::ofstream _descriptor_file;
  std::ofstream _icp_check_file;
  std::ofstream _descriptor_bw_file;
  std::ofstream _pc_bw_file;
  std::ofstream _pose_bw_file;
  std::ofstream _dual_bw_file;

  struct termios orig_termios;

  void paramLoader();
  void initialization();
  void allocateMemory();
  void releaseMemory();
  bool saveTrajectoriesService(multi_proxy::save_traj::Request &req,
                               multi_proxy::save_traj::Response &res);
  void heartbeatCallback(const std_msgs::HeaderConstPtr &msg);
  void checkHeartbeatCallback(const ros::TimerEvent &);
  void laserCloudInfoHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn);
  void localizationInfoHandler(const nav_msgs::OdometryConstPtr &msgIn);
  void descriptorHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn);
  void candidateLoopHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn);
  void verifiedLoopTransHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn);
  void dualPriorHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn);

  bool isKeyframe(KeyFrame &descriptor_info);
  void buildFrameList(KeyFrame &context_in);
  void buildDescriptor();
  void publishBlockedDescriptor();
  void getCandidateLoop();
  void publishBlockedCandidateLoop();
  void publishCandidateLoop(LoopClosure &loop, const RobotID &neighbor);
  void geometricVerification();
  bool getInitialGuess(const pcl::PointCloud<PointType>::Ptr &currentCloud,
                       KeyFrame &sc_current, KeyFrame &sc_reference, Eigen::Affine3f &sc_initial);
  void publishBlockedVerifiedLoop();
  void publishVerifiedLoop(LoopClosure &loop, const RobotID &neighbor);
  void setNonCanonicalMode();
  void resetTerminalMode();
  bool kbhit();
  void monitor();

public:
  int _robot_id_th = 0;
  std::string _robot_name;

  explicit MultiProxyPipeline(ros::NodeHandle &nh)
  {
    node_handler = nh;
    paramLoader();
    initialization();
  }

  ~MultiProxyPipeline()
  {
    releaseMemory();
    std::cout << GREEN << "[SubMapFusion " + _robot_name + "] releasedMemory!" << RESET << std::endl;
    _odom_file.close();
    _descriptor_file.close();
    _icp_check_file.close();
    _descriptor_bw_file.close();
    _pc_bw_file.close();
    _pose_bw_file.close();
    _dual_bw_file.close();
  }

  void publishDescriptorThread();
  void getLoopAndPubCloudThread();
  void geometricVerificationThread();
  void globalOptimizationThread();
  void publishGlobalMapThread();
  void savePoseAndMap();
};

#endif // MULTI_PROXY_MULTIPROXYPIPELINE_H
