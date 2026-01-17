
// Created by jcwang on 24-11-29.
//

#include "MultiOutlierRejection.h"



double MultiOutlierRejection::residualPCM(gtsam::Pose3 inter_kj, gtsam::Pose3 inter_li, gtsam::Pose3 inner_ij, gtsam::Pose3 inner_kl, double intensity){
  gtsam::Pose3 inter_jk = inter_kj.inverse();
  gtsam::Pose3 res_pose = inner_ij * inter_jk * inner_kl * inter_li;
  gtsam::Vector6 res_vec = gtsam::Pose3::Logmap(res_pose);

  Eigen::Matrix< double, 6, 1> v ;
  v << intensity * 1000, intensity * 1000, intensity * 1000,
      intensity, intensity, intensity;
  Eigen::Matrix< double, 6, 6> m_cov = v.array().matrix().asDiagonal();

  return sqrt(res_vec.transpose()* m_cov * res_vec);
}

void MultiOutlierRejection::printPCMGraph(Eigen::MatrixXi pcm_matrix, const std::string& file_name) {
  // Intialization
  int nb_consistent_measurements = 0;

  // Format edges.
  std::stringstream ss;
  for (int i = 0; i < pcm_matrix.rows(); i++) {
    for (int j = i; j < pcm_matrix.cols(); j++) {
      if (pcm_matrix(i,j) == 1) {
        ss << i+1 << " " << j+1 << std::endl;
        nb_consistent_measurements++;
      }
    }
  }

  // Write to file
  std::ofstream output_file;
  output_file.open(file_name, std::ios::out);
  if (!output_file.is_open()) std::cerr << "Error: cannot open file " << file_name << std::endl;
  output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
  output_file << pcm_matrix.rows() << " " << pcm_matrix.cols() << " " << nb_consistent_measurements << std::endl;
  output_file << ss.str();
  output_file.close();
}

void MultiOutlierRejection::readPCMGraph(Eigen::MatrixXi &pcm_matrix, const std::string& file_name) {
  // Open the file.
  std::ifstream input_file(file_name);
  if (!input_file.is_open()) {
    std::cerr << "Error: cannot open file " << file_name << std::endl;
    return;
  }

  // Read the header.
  std::string line;
  std::getline(input_file, line);
  if (line != "%%MatrixMarket matrix coordinate pattern symmetric") {
    std::cerr << "Error: invalid file format" << std::endl;
    return;
  }

  // Read the matrix dimensions and number of non-zero elements.
  int rows, cols, nnz;
  input_file >> rows >> cols >> nnz;

  // Read the matrix entries.
  int i, j;
  for (int k = 0; k < nnz; k++) {
    input_file >> i >> j;
    if (i > rows || j > cols) continue;
    pcm_matrix(i-1, j-1) = 1;
    pcm_matrix(j-1, i-1) = 1;
  }

  // Close the file.
  input_file.close();
}

bool MultiOutlierRejection::iPCM(const std::unordered_map<int, std::vector<LoopClosure>> &loop_queue, KeyFrameList &_cur_info_list, KeyFrameList &_ref_info_list) {
  bool available_flag = false;
  for (const auto& itm : loop_queue) {
    if (itm.first > robot_id) continue;
    int ref_robot_id = itm.first;
    if (itm.second.empty()) continue;

    auto loop_queue_this = itm.second;
    int max_clique_size = 0;
    std::vector<int> max_clique_data;
    if (prev_matrix_size.find(ref_robot_id) == prev_matrix_size.end())
      prev_matrix_size.emplace(ref_robot_id, 0);
    if (prev_max_clique_size.find(ref_robot_id) == prev_max_clique_size.end())
      prev_max_clique_size.emplace(ref_robot_id, 0);
    if (loop_queue_this.empty() ||
        loop_queue_this.size() <= prev_matrix_size[ref_robot_id] ||
        loop_queue_this.size() < pcm_start_threshold) continue;
    //perform pcm for all robot matches
    Eigen::MatrixXi consistency_matrix = computePCMMatrix(loop_queue_this, _cur_info_list, _ref_info_list, prev_matrix_size[ref_robot_id]);
    std::string consistency_matrix_file = pcm_matrix_folder + "/PCM_matrix_robot_" + std::to_string(robot_id) + "_wrt_" + std::to_string(ref_robot_id) + ".clq.mtx";
    if (prev_matrix_size[ref_robot_id] != 0) readPCMGraph(consistency_matrix, consistency_matrix_file);
    printPCMGraph(consistency_matrix, consistency_matrix_file);

    if (prev_resPCMMat.find(ref_robot_id) == prev_resPCMMat.end()) {
      prev_resPCMMat.emplace(ref_robot_id, resPCMMat);
    }
    else {
      resPCMMat.block<Eigen::Dynamic, Eigen::Dynamic>(0, 0, (size_t)prev_resPCMMat[ref_robot_id].rows(), (size_t)prev_resPCMMat[ref_robot_id].cols()) = prev_resPCMMat[ref_robot_id];
      prev_resPCMMat[ref_robot_id] = resPCMMat;
    }

    std::string res_matrix_file = pcm_matrix_folder + "/cost_matrix_robot_" + std::to_string(robot_id) + "_wrt_" + std::to_string(ref_robot_id) + ".txt";
    writeLoopClosuresToFile(loop_queue_this, resPCMMat, res_matrix_file);

    // Compute maximum clique
    FMC::CGraphIO gio;
    if (!gio.readGraph(consistency_matrix_file)) {
      continue;
    }
    gio.CalculateVertexDegrees();

    max_clique_size = FMC::maxCliqueHeuIncremental(gio, loop_queue_this.size() - prev_matrix_size[ref_robot_id],
                                                   prev_max_clique_size[ref_robot_id], max_clique_data);
    prev_matrix_size[ref_robot_id] = (int) loop_queue_this.size();
    prev_max_clique_size[ref_robot_id] = max_clique_size;
    if (max_clique_data.empty() || max_clique_size < pcm_max_clique_size) continue;

    std::vector<LoopClosure> loop_accept_this;
    for (auto idx: max_clique_data) {
      if (idx < 0) {
        continue;
      }
      loop_accept_this.push_back(loop_queue_this[idx]);
    }
    {
      std::lock_guard<std::mutex> lock(mtx_loop_accept_idx_queue);
      if (loop_accept_idx_queue.find(ref_robot_id) == loop_accept_idx_queue.end()){
        loop_accept_idx_queue.emplace(ref_robot_id, loop_accept_this);
        available_flag = true;
        continue;
      }

      loop_accept_idx_queue[ref_robot_id].clear();
      loop_accept_idx_queue[ref_robot_id] = loop_accept_this;
      available_flag = true;
    }
  }
  return available_flag;
}
/*            T3
 * cur  aj<---------ai
 *    T1^           ^ T2
 *      |           |
 * ref  bk--------->bl
 *            T4
 * */

Eigen::MatrixXi MultiOutlierRejection::computePCMMatrix(const std::vector<LoopClosure> &loop_queue_this, KeyFrameList &_cur_info_list, KeyFrameList &_ref_info_list, int last_PCM_size){
  Eigen::MatrixXi PCMMat;
  PCMMat.setZero((size_t)loop_queue_this.size(), (size_t)loop_queue_this.size());
  resPCMMat.setZero((size_t)loop_queue_this.size(), (size_t)loop_queue_this.size());
  int frame_id_0, frame_id_1;
  gtsam::Pose3 z_bk_aj, z_bl_ai;
  gtsam::Pose3 z_ai_aj, z_bk_bl;
  gtsam::Pose3 t_ai, t_aj, t_bk, t_bl;

  for (unsigned int i = 0; i < loop_queue_this.size(); i++){
    frame_id_0 = loop_queue_this[i].c_frame_id;
    frame_id_1 = loop_queue_this[i].r_frame_id;
    z_bk_aj = loop_queue_this[i].pose;
    std::shared_ptr<KeyFrame> cur_keyframe, ref_keyframe;
    cur_keyframe = _cur_info_list.getKeyFrame(frame_id_0);
    ref_keyframe = _ref_info_list.getKeyFrame(frame_id_1);
    if (nullptr == ref_keyframe) ref_keyframe = _cur_info_list.getKeyFrame(frame_id_1);
    t_aj = cur_keyframe->getPoseInit().toPose3();
    t_bk = ref_keyframe->getPoseInit().toPose3();

    for (unsigned int j = (i + 1 > last_PCM_size ? i + 1 : last_PCM_size); j < loop_queue_this.size(); j++){
      frame_id_0 = loop_queue_this[j].c_frame_id;
      frame_id_1 = loop_queue_this[j].r_frame_id;
      z_bl_ai = loop_queue_this[j].pose;
      cur_keyframe = _cur_info_list.getKeyFrame(frame_id_0);
      ref_keyframe = _ref_info_list.getKeyFrame(frame_id_1);
      if (nullptr == ref_keyframe) ref_keyframe = _cur_info_list.getKeyFrame(frame_id_1);
      t_ai = cur_keyframe->getPoseInit().toPose3();
      t_bl = ref_keyframe->getPoseInit().toPose3();
      z_ai_aj = t_ai.inverse() * t_aj;
      z_bk_bl = t_bk.inverse() * t_bl;
      int frame_gap = abs(loop_queue_this[i].c_frame_id - loop_queue_this[j].c_frame_id) +
                      abs(loop_queue_this[i].r_frame_id - loop_queue_this[j].r_frame_id);
      double resi = 0.0;
      if (loop_queue_this[i].r_frame_id > loop_queue_this[i].c_frame_id)
        resi = residualPCM(z_bk_aj.inverse(), z_bl_ai.inverse(), z_bk_bl.inverse(), z_ai_aj.inverse(), sqrtThreholdFunc(frame_gap));
      else
        resi = residualPCM(z_bk_aj, z_bl_ai, z_ai_aj, z_bk_bl, sqrtThreholdFunc(frame_gap));
      if (resi < pcm_threshold)
        PCMMat(i,j) = 1;
      else
        PCMMat(i,j) = 0;
      resPCMMat(i,j) = resi;
      resPCMMat(j,i) = resi;
    }
  }
  return PCMMat;
}

std::vector<LoopClosure> MultiOutlierRejection::getInterLoopAcceptQueue() {
  std::lock_guard<std::mutex> lock(mtx_loop_accept_idx_queue);
  std::vector<LoopClosure> loop_accept_queue;
//        std::cout << GREEN << "[SubMapFusion "  + _robot_name + "] <GlobalOPT> Loop accept size: " << loop_accept_queue_idx.size() << RESET << std::endl;
  for (const auto& accept_queue : loop_accept_idx_queue) {
    if (accept_queue.first == robot_id) continue;
    for (const auto& loop : accept_queue.second) {
      loop_accept_queue.push_back(loop);
    }
  }
  return loop_accept_queue;
}

std::vector<LoopClosure> MultiOutlierRejection::getAllLoopAcceptQueue() {
  std::lock_guard<std::mutex> lock(mtx_loop_accept_idx_queue);
  std::vector<LoopClosure> loop_accept_queue;
  for (const auto& accept_queue : loop_accept_idx_queue) {
    for (const auto& loop : accept_queue.second) {
      loop_accept_queue.push_back(loop);
    }
  }
  return loop_accept_queue;
}

void MultiOutlierRejection::clearLoopAcceptQueue(int ref_robot_id) {
  std::lock_guard<std::mutex> lock(mtx_loop_accept_idx_queue);
  loop_accept_idx_queue[ref_robot_id].clear();
}
void MultiOutlierRejection::resetPrevPCM() {
  prev_matrix_size.clear();
  prev_max_clique_size.clear();

}
void MultiOutlierRejection::writeLoopClosuresToFile(const std::vector<LoopClosure>& loop_queue_this,
                             const Eigen::MatrixXd& resPCMMat,
                             const std::string& filename) {
  std::ofstream outfile(filename);

  if (!outfile.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }
  int count = 0;
  // Write candidates section
  outfile << "Candidates for PCM:\n";
  for (const auto& lc : loop_queue_this) {
    outfile << "index " << count++ << ", c_frame_id: " << lc.c_frame_id
            << ", r_frame_id: " << lc.r_frame_id << ", pose: "
            << lc.frame_trans.x() << ", " << lc.frame_trans.y() << ", " << lc.frame_trans.z() << "\n";
  }

  outfile << "\nCost Matrix for PCM:\n";

  for (int i = 0; i < resPCMMat.rows(); ++i) {
    outfile << std::setw(13) << std::left << "index " << i << ": ";
    for (int j = 0; j < resPCMMat.cols(); ++j) {
      outfile << std::setw(13) << std::left << resPCMMat(i, j);
    }
    outfile << "\n";
  }

  outfile.close();
}
void MultiOutlierRejection::writeLoopClosuresToFile(const std::vector<LoopClosure>& loop_queue_this,
                                                    const Eigen::MatrixXi& resPCMMat,
                                                    const std::string& filename) {
  std::ofstream outfile(filename);

  if (!outfile.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return;
  }
  int count = 0;
  // Write candidates section
  outfile << "Candidates for PCM:\n";
  for (const auto& lc : loop_queue_this) {
    outfile << "index " << count++ << ", c_frame_id: " << lc.c_frame_id
            << ", r_frame_id: " << lc.r_frame_id << ", pose: "
            << lc.pose.x() << ", " << lc.pose.y() << ", " << lc.pose.z() << "\n";
  }

  outfile << "\nCost Matrix for PCM:\n";

  for (int i = 0; i < resPCMMat.rows(); ++i) {
    for (int j = 0; j < resPCMMat.cols(); ++j) {
      outfile << std::setw(13) << std::left << resPCMMat(i, j);
    }
    outfile << "\n";
  }

  outfile.close();
}

double MultiOutlierRejection::sqrtThreholdFunc(int x) {
  return 1.0 / (1.0 + ((double)x));
}
