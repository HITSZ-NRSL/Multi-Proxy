#include "DualFrame.h"

DualFrameList::DualFrameList() {
}

void DualFrameList::setRobotID(int r_id) {
  robot_id = r_id;
}

bool DualFrameList::update_remote(int32_t frame_id, DualFrame dual_frame) {

  auto [it, is_new] = dual_frames_remote.try_emplace(frame_id, dual_frame);

  if (!is_new) {
    it->second.pose = dual_frame.pose;
  }
  return is_new;
}

bool DualFrameList::add_local(int32_t frame_id, DualFrame dual_frame) {
  auto [it, is_new] = dual_frames_local.try_emplace(frame_id, dual_frame);
  return is_new;
}

void DualFrameList::reset_curr_update_cnt() {
  for (auto &dual_frame : dual_frames_local) {
    dual_frame.second.update_cnt = 0;
  }
}

void DualFrameList::local_update(KeyFrameList &curr_frames, KeyFrameList &ref_frames, const std::unordered_map<int, PointTypePose>& global_maps) {
  gtsam::Pose3 global_map_wrt_ref_robot = global_maps.at(ref_robot_id).toPose3();
  bool update_flag = false;

  for (auto &dual_frame_remote : dual_frames_remote) {
    int32_t frame_id = dual_frame_remote.first;
    DualFrame dual_remote = dual_frame_remote.second;

    auto dual_it = dual_frames_local.find(frame_id);
    if (dual_it == dual_frames_local.end()) {
      std::cout << RED <<  "[SubMapFusion robot_" << robot_id << "] dual robot: " << dual_remote.robot_id << " frame_id = " << frame_id << ", no curr dual prior" << RESET << std::endl;
      continue;
    }
    DualFrame dual_local = dual_it->second;
    std::shared_ptr<KeyFrame> frame_now = nullptr;
    gtsam::Pose3 pose_now, init_pose;
    if (frame_id % max_keyframes == 0 && global_maps.find(frame_id / max_keyframes) != global_maps.end()) {
      pose_now = global_map_wrt_ref_robot * global_maps.at(frame_id / max_keyframes).toPose3().inverse();
      init_pose = pose_now;
    }
    else {

      if (frame_id / max_keyframes == robot_id)
        frame_now = curr_frames.getKeyFrame(frame_id);
      else {
        frame_now = ref_frames.getKeyFrame(frame_id);
      }

      if (frame_now == nullptr) {
        std::cout << RED <<  "[SubMapFusion robot_" << frame_id / max_keyframes << "] dual robot: " << dual_local.robot_id
            << " frame_id = " << frame_id << ", dual prior has no corresponding pose" << RESET << std::endl;
        dual_frames_local[frame_id].update_cnt = 0;
        continue;
      }

      if (frame_now->has_optimized) {
        pose_now = global_map_wrt_ref_robot * frame_now->distributed_pose_in_cur_map.toPose3();
        init_pose = global_map_wrt_ref_robot * frame_now->decentralized_pose_in_cur_map.toPose3();
      }
      else {
        // todo delet dual
        continue;
      }
    }
    dual_frames_local[frame_id].update_cnt++;

    gtsam::Pose3 avg_dual = KarcherMeanSE3({dual_local.pose, dual_remote.pose});

    gtsam::Pose3 error_pose = avg_dual.inverse().compose(pose_now);
    gtsam::Pose3 error_init_pose = init_pose * avg_dual.inverse();
    dual_frames_local[frame_id].pose_error = error_pose;
    auto error_dual_tangent = gtsam::Pose3::Logmap(dual_local.pose.inverse() * dual_remote.pose);
    auto error_tangent = gtsam::Pose3::Logmap(error_pose);

    if (error_tangent.norm() < min_error && dual_frames_local[frame_id].update_cnt > max_iters && frame_id % max_keyframes != 0) continue;
#ifdef DEBUG
    double rotation_norm = error_tangent.head<3>().norm();
    double translation_norm = error_tangent.tail<3>().norm();
    if (ita*rotation_norm > M_PI || ita*translation_norm > 10.0) {

      std::cout << RED << "rotation_norm > M_PI || translation_norm > 1.0: 数值不稳定发生！！！！！！ robot: " << robot_id
          << ", ref_robot: " << ref_robot_id << "Frame: " << frame_id << "rotation: "
          << ita*rotation_norm << ", translation: " << ita*translation_norm << RESET << std::endl;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      SEtoQuatT(dual_local.pose.matrix(), q, t);
      Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_frame " << frame_id << " \ndual_local t: \n" << t.transpose() << ", \n q: " << v.transpose() << std::endl;

      SEtoQuatT(dual_remote.pose.matrix(), q, t);
      Eigen::Vector4d r(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_remote t: \n" << t.transpose() << ", \n q: " << r.transpose() << std::endl;

      SEtoQuatT(avg_dual.matrix(), q, t);
      Eigen::Vector4d w(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_avg t: \n" << t.transpose() << ", \n q: " << w.transpose() << std::endl;

      SEtoQuatT(pose_now.matrix(), q, t);
      Eigen::Vector4d g(q.w(), q.x(), q.y(), q.z());
      std::cout << "\npose_now t: \n" << t.transpose() << ", \n q: " << g.transpose() << std::endl;
    }
    if (debug_node_id >= 0 && debug_node_id != frame_id) {
      continue;
    }
    else if (debug_node_id == frame_id) {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      SEtoQuatT(dual_local.pose.matrix(), q, t);
      Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_frame " << frame_id << " \ndual_local t: \n" << t.transpose() << ", \n q: " << v.transpose() << std::endl;

      SEtoQuatT(dual_remote.pose.matrix(), q, t);
      Eigen::Vector4d r(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_remote t: \n" << t.transpose() << ", \n q: " << r.transpose() << std::endl;

      SEtoQuatT(avg_dual.matrix(), q, t);
      Eigen::Vector4d w(q.w(), q.x(), q.y(), q.z());
      std::cout << "\ndual_avg t: \n" << t.transpose() << ", \n q: " << w.transpose() << std::endl;
    }
#endif
    gtsam::Pose3 delta_pose = gtsam::Pose3(gtsam::Pose3::Expmap(ita*error_tangent));
    gtsam::Pose3 new_dual = dual_local.pose.compose(delta_pose);
    dual_frames_local[frame_id].pose = new_dual;
    dual_frames_local[frame_id].updated = true;

    if (dual_frames_local[frame_id].updated && dual_frames_local[frame_id].update_cnt <= max_iters) {
      update_flag = true;
    }
  }
}

bool DualFrameList::saveFrames(int rID, int target, std::unordered_map<int, PointTypePose> global_maps, bool trunc) {

  std::string filename = std::string(ROOT_DIR) + "log" + "/DualFrame_robot_" + std::to_string(rID) + ".txt";
  std::ofstream ofs;
  if (trunc)
    ofs = std::ofstream(filename, std::ios::out | std::ios::trunc);
  else
    ofs = std::ofstream(filename, std::ios::out | std::ios::app);
  if (!ofs.is_open()) {
    std::cerr << "Error: Unable to open file " << filename << std::endl;
    return false;
  }


  gtsam::Pose3 global_map;
  if (global_maps.find(target) != global_maps.end())
    global_map = global_maps[target].toPose3();
  auto T = global_map.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << T(i, j);
      if (j < 3)
        ofs << ", ";
    }
    ofs << "\n";
  }


  for (const auto& pair : dual_frames_local) {
    const DualFrame& df = pair.second;
    ofs << df.robot_id << ", ";
    ofs << pair.first << ", ";


    ofs << " ";
    ofs << target;


    auto pos = df.pose.translation();
    ofs << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << "\n";
  }


  for (const auto& pair : dual_frames_remote) {
    const DualFrame& df = pair.second;
    ofs << df.robot_id << ", ";
    ofs << pair.first << ", ";

    ofs << " ";
    ofs << target;

    auto pos = df.pose.translation();
    ofs << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << "\n";
  }

  ofs.close();
  return true;
}

