//
// Created by jcwang on 24-5-8.
//

#ifndef MULTI_PROXY_BACKEND_H
#define MULTI_PROXY_BACKEND_H

#include "utility.h"
#include "KeyFrameList.h"
#include "DualFrame.h"

//gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#if ROBUSTGTSAM
#include <risam/RISAM2.h>
#endif
//expression graph
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h>

//factor graph
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>
#include <set>

class BackEnd {
  std::string log_file_path = std::string(ROOT_DIR) + "log";
  std::ofstream error_file;
  std::ofstream graph_file;
  int robot_id = 0; // from 0 to N-1
  int max_keyframes = 10000;
  bool rebuild_graph = true;
  bool key_frame_only = false;
  std::string log_folder = std::string(ROOT_DIR) + "log";

  KeyFrameList *cur_info_list;
  KeyFrameList *ref_info_list;
#if ROBUSTGTSAM
  boost::shared_ptr<risam::RISAM2> isam2;
  risam::RISAM2::RISAM2Params parameters;
#else
  boost::shared_ptr<gtsam::ISAM2> isam2;
  gtsam::ISAM2Params parameters;
#endif
  gtsam::NonlinearFactorGraph graph_prior;
  gtsam::NonlinearFactorGraph graph_scan;
  gtsam::NonlinearFactorGraph graph_loop;
  gtsam::NonlinearFactorGraph graph_dual;
  gtsam::NonlinearFactorGraph graph_s2m;
  gtsam::NonlinearFactorGraph distributed_graph_all, decentralized_graph_all;
  gtsam::Values distributed_init_est_all, decentralized_init_est_all, diff_init;
  gtsam::SharedNoiseModel const_noise_prior =
      gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished());
  gtsam::SharedNoiseModel const_noise_s2m;
  gtsam::SharedNoiseModel const_noise_odom;
  gtsam::SharedNoiseModel const_noise_inter_loop;
  gtsam::SharedNoiseModel const_noise_intra_loop;
  gtsam::SharedNoiseModel const_noise_dual;
  std::mutex mtx_isam2;
  std::mutex mtx_map_list;

  std::vector<LoopClosure> loop_in_graph;
  std::unordered_map<int, std::unordered_map<int, PointTypePose>> previous_odom;
  std::map<int32_t, uint16_t> dual_prior_index;
  std::unordered_map<int, PointTypePose> global_map_trans_optimized;
  std::unordered_map<gtsam::Key, std::pair<gtsam::Pose3, gtsam::Pose3>> init_pose;

public:
  int debug_node_id = -1;
  int debug_ban_type = -1;
  bool print_net = false;
  BackEnd(int r_id, int max_keyframes, KeyFrameList *cur_list, KeyFrameList *ref_list);
  ~BackEnd();
  void rebuild() {
    graph_loop.resize(0);
    graph_s2m.resize(0);
    loop_in_graph.clear();
    rebuild_graph = true;
  }
  bool needUpdate() {
    return rebuild_graph;
  }

  void setKeyFrameOnly(bool key_frame_only_init) {
    key_frame_only = key_frame_only_init;
    std::cout << "robot_" << robot_id << " set key frame only: " << key_frame_only << std::endl;
  }

  void resetOptimization();
  void setRobotID(int id) {
    robot_id = id;
    error_file.open(log_file_path + "/robot_" + std::to_string(robot_id) + "_be_err.txt", std::ios::app);
    if (!error_file.is_open()) std::cerr << "Error: cannot open file " << log_file_path << std::endl;
  }
  void setMaxKeyframes(int max) {
    std::cout << "robot_" << robot_id << " setting max key frame: " << max << std::endl;
    max_keyframes = max;
    }
  void setS2MStd(double std) {
    std::cout << "robot_" << robot_id << " setting S2m noise model: " << std << std::endl;
    gtsam::Vector vector6(6);
    vector6 << std * 0.001, std * 0.001, std * 0.001, std * 1, std * 1, std * 1;
    const_noise_s2m = gtsam::noiseModel::Diagonal::Variances(vector6);
  }

  void setOdomStd(double std) {
    std::cout << "robot_" << robot_id << " setting Odom noise model: " << std << std::endl;
    gtsam::Vector vector6(6);
    vector6 << std * 0.001, std * 0.001, std * 0.001, std * 1, std * 1, std * 1;
    const_noise_odom = gtsam::noiseModel::Diagonal::Variances(vector6);
  }

  void setInterLoopStd(double std) {
    std::cout << "robot_" << robot_id << " setting Inter Loop noise model: " << std << std::endl;
    gtsam::Vector vector6(6);
    vector6 << std * 0.001, std * 0.001, std * 0.001, std, std, std;
    const_noise_inter_loop = gtsam::noiseModel::Diagonal::Variances(vector6);
  }

  void setIntraLoopStd(double std) {
    std::cout << "robot_" << robot_id << " setting Intra Loop noise model: " << std << std::endl;
    gtsam::Vector vector6(6);
    vector6 << std * 0.001, std * 0.001, std * 0.001, std, std, std;
    const_noise_intra_loop = gtsam::noiseModel::Diagonal::Variances(vector6);
  }

  void setDualStd(double std) {
    std::cout << "robot_" << robot_id << " setting Dual noise model: " << std << std::endl;
    gtsam::Vector vector6(6);
    vector6 << std * 0.001, std * 0.001, std * 0.001, std, std, std;
    const_noise_dual = gtsam::noiseModel::Diagonal::Variances(vector6);
  }

  void gtsamEgoPgoGraph();
  void gtsamDualFactor(std::unordered_map< int, DualFrameList> &dual_frames_all);
  bool gtsamInterLoop(const std::vector<LoopClosure> &loop_accepted);
  PointTypePose getPoseWrtRefRobotIdx(int robot_idx);
  std::unordered_map< int, PointTypePose> getMapOptimized() {
    std::lock_guard<std::mutex> lock(mtx_map_list);
    std::unordered_map< int, PointTypePose> map_trans;
    for (const auto& itm : global_map_trans_optimized) {
      PointTypePose map = itm.second;
      map_trans.emplace(itm.first, map);
    }
    return map_trans;
  }

  bool isRobotOptimized(int robot_idx) {return global_map_trans_optimized.find(robot_idx) != global_map_trans_optimized.end();}

  int getMinKeyOptimized();

  void gtsamInit();

  void gtsamUpdate(bool arock_flag);
  bool printBayesTree();
  bool printBayesNet();
  // void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph, gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate);
  void setLogFolder(const std::string &folder) {log_folder = folder;}
};


#endif//MULTI_PROXY_BACKEND_H
