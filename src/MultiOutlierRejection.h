//
// Created by jcwang on 24-11-29.
//

#ifndef MULTI_PROXY_MULTIOUTLIERREJECTION_H
#define MULTI_PROXY_MULTIOUTLIERREJECTION_H


#include <map>
#include <vector>
#include <tuple>
#include <gtsam/geometry/Pose3.h>
#include "fast_max-clique_finder/src/findClique.h"
#include "KeyFrameList.h"
#include "utility.h"


class MultiOutlierRejection {
  int pcm_start_threshold = 5;
  int pcm_max_clique_size = 5;
  double pcm_threshold = 0.5;
  double param_k = 0.1;
  double param_L = 5.0;
  double param_x0 = 100;
  std::mutex mtx_loop_accept_idx_queue;
  std::string pcm_matrix_folder;
  int robot_id;
  std::unordered_map<int, std::vector<LoopClosure>> loop_accept_idx_queue;
  std::unordered_map<int, int> prev_matrix_size;
  std::unordered_map<int, int> prev_max_clique_size;
  Eigen::MatrixXd resPCMMat;
  std::unordered_map<int, Eigen::MatrixXd> prev_resPCMMat;

public:
  MultiOutlierRejection() = default;
  bool iPCM(const std::unordered_map<int, std::vector<LoopClosure>> &loop_queue, KeyFrameList &_cur_info_list, KeyFrameList &_ref_info_list);
  Eigen::MatrixXi computePCMMatrix(const std::vector<LoopClosure> &loop_queue_this, KeyFrameList &_cur_info_list, KeyFrameList &_ref_info_list, int last_PCM_size = 0);
  double residualPCM(gtsam::Pose3 inter_kj, gtsam::Pose3 inter_li, gtsam::Pose3 inner_ij, gtsam::Pose3 inner_kl, double intensity);
  void printPCMGraph(Eigen::MatrixXi pcm_matrix, const string &file_name);
  void readPCMGraph(Eigen::MatrixXi &pcm_matrix, const string &file_name);
  std::vector<LoopClosure> getInterLoopAcceptQueue();
  std::vector<LoopClosure> getAllLoopAcceptQueue();
  void clearLoopAcceptQueue(int ref_robot_id);


  void setPcmStartThreshold(int threshold) {pcm_start_threshold = threshold;}
  int getPcmStartThreshold() {return pcm_start_threshold;}
  void setPcmMaxCliqueSize(int size) {pcm_max_clique_size = size;}
  int getPcmMaxCliqueSize() {return pcm_max_clique_size;}
  void setPcmMatrixFolder(const std::string &folder) {pcm_matrix_folder = folder;}
  std::string getPcmMatrixFolder() {return pcm_matrix_folder;}
  void setRobotID(int id) {
    robot_id = id;
  }
  void setPcmThreshold(double threshold) {
    std::cout << "Set PCM threshold to: " << threshold << std::endl;
    pcm_threshold = threshold; }
  double getPcmThreshold() { return pcm_threshold; }
  void resetPrevPCM();
  void writeLoopClosuresToFile(const std::vector<LoopClosure>& loop_queue_this,
                               const Eigen::MatrixXd& resPCMMat,
                               const std::string& filename);
  void writeLoopClosuresToFile(const std::vector<LoopClosure>& loop_queue_this,
                               const Eigen::MatrixXi& resPCMMat,
                               const std::string& filename);

  double sqrtThreholdFunc(int x);
};


#endif//MULTI_PROXY_MULTIOUTLIERREJECTION_H
