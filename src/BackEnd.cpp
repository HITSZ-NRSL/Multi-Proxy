//
// Created by jcwang on 24-5-8.
//

#include "BackEnd.h"

BackEnd::BackEnd(int r_id, int max_keyframes, KeyFrameList *cur_list, KeyFrameList *ref_list) : cur_info_list(cur_list), ref_info_list(ref_list)
{
#if ROBUSTGTSAM
  risam::DoglegLineSearchParams opt_params;
  opt_params.search_type = risam::DoglegLineSearchType::OUTWARD;
  opt_params.init_delta = 1.0;
  opt_params.min_delta = 0.01;
  opt_params.max_delta = 100;
  parameters.value_converge_max_iters = 50;
  parameters.converge_after_new_gnc = true;
  parameters.converge_mu = true;
  parameters.converge_values = true;
  parameters.increment_outlier_mu = true;
  parameters.optimization_params = opt_params;
#else
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  parameters.factorization = gtsam::ISAM2Params::QR;
#endif
  gtsam::Symbol x0('x', r_id * max_keyframes);
#if ROBUSTGTSAM
  auto prior_factor = risam::make_shared_graduated<gtsam::PriorFactor<gtsam::Pose3>>(
      boost::make_shared<risam::SIGKernel>(6), x0, gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)),
      const_noise_prior);
  graph_prior.push_back(prior_factor);
#else
  graph_prior.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::PriorFactor<gtsam::Pose3>(x0,
                                       gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),
                                                    gtsam::Point3(0, 0, 0)),
                                       const_noise_prior)));
#endif
  std::cout << "Robot " << r_id << " backend initialized" << std::endl;
}

BackEnd::~BackEnd()
{
  error_file.close();
}

// init after set parameters
void BackEnd::resetOptimization()
{
  cur_info_list->resetFlag();
  ref_info_list->resetFlag();
  rebuild_graph = false;
}

void BackEnd::gtsamEgoPgoGraph()
{
  gtsam::NonlinearFactorGraph graph_inc_scan;
  if (cur_info_list->size() < 2)
    return;

  gtsam::VariableIndex var_index(graph_scan);
  auto last_data = cur_info_list->at(0);
  for (int i = 0; i < cur_info_list->size(); i++)
  {
    const auto frame_data = cur_info_list->at(i);
    auto cur_p_in_cur = frame_data->getPoseInit().toPose3();
    auto last_p_in_cur = last_data->getPoseInit().toPose3();
    if (i == 0)
    {
      gtsam::Symbol x0('x', robot_id * max_keyframes), x1('x', frame_data->frame_id);
      if (var_index.find(x1.key()) != var_index.end())
      {
        last_data = frame_data;
        continue;
      }

#if ROBUSTGTSAM
      auto first_factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
          boost::make_shared<risam::SIGKernel>(6), x0, x1, cur_p_in_cur, const_noise_prior);
      graph_inc_scan.push_back(first_factor);
#else
      graph_inc_scan.add(gtsam::BetweenFactor<gtsam::Pose3>(x0,
                                                            x1,
                                                            cur_p_in_cur,
                                                            const_noise_prior));
#endif
      if (init_pose.find(x1.key()) == init_pose.end())
      {
        std::pair<gtsam::Pose3, gtsam::Pose3> initPair(cur_p_in_cur, cur_p_in_cur);
        init_pose.emplace(x1.key(), initPair);
      }
      continue;
    }

    if (key_frame_only && !frame_data->is_key_frame)
    {
      continue;
    }

    auto odom_between = last_p_in_cur.inverse() * cur_p_in_cur;
    // add observation of current robot odometry
    gtsam::Symbol x0('x', last_data->frame_id), x1('x', frame_data->frame_id);
    if (var_index.find(x1.key()) != var_index.end())
    {
      last_data = frame_data;
      continue;
    }
    if (debug_ban_type & 0b0001 && debug_node_id == frame_data->frame_id)
    {
      std::cout << "robot_" << robot_id << ": skip odom factor for debug node: " << frame_data->frame_id << "to " << last_data->frame_id << std::endl;
      last_data = frame_data;
      continue;
    }
    if (debug_ban_type & 0b0010 && debug_node_id == last_data->frame_id)
    {
      std::cout << "robot_" << robot_id << ": skip odom factor for debug node: " << frame_data->frame_id << "to " << last_data->frame_id << std::endl;
      last_data = frame_data;
      continue;
    }

#if ROBUSTGTSAM
    auto factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
        boost::make_shared<risam::SIGKernel>(6), x0, x1, odom_between, const_noise_odom);
    graph_inc_scan.push_back(factor);
#else
    graph_inc_scan.add(gtsam::BetweenFactor<gtsam::Pose3>(
        x0, x1, odom_between, const_noise_odom));
#endif
    if (init_pose.find(x1.key()) == init_pose.end())
    {
      std::pair<gtsam::Pose3, gtsam::Pose3> initPair(cur_p_in_cur, cur_p_in_cur);
      init_pose.emplace(x1.key(), initPair);
    }
    last_data = frame_data;
  }

  if (!graph_inc_scan.empty())
  {
    graph_scan += graph_inc_scan;
  }
  graph_inc_scan.resize(0);
}

void BackEnd::gtsamDualFactor(std::unordered_map<int, DualFrameList> &dual_frames_all)
{

  graph_dual.resize(0);
  for (auto &df_list : dual_frames_all)
  {
    if (df_list.second.regenerate)
      continue;
    auto &dual_frames = df_list.second;
    for (auto dual_it = dual_frames.dual_frames_remote.begin();
         dual_it != dual_frames.dual_frames_remote.end();)
    {
      int frame_id = dual_it->first;
      auto &dual = dual_it->second;
      if (dual_frames.dual_frames_local.find(frame_id) == dual_frames.dual_frames_local.end())
      {
        dual_it = dual_frames.dual_frames_remote.erase(dual_it);
        continue;
      }
      else
      {
        ++dual_it;
      }

      if (frame_id % max_keyframes == 0)
      {

        if (!isRobotOptimized(dual_frames.ref_robot_id))
          continue;
        if (!isRobotOptimized(frame_id / max_keyframes))
          continue;
        gtsam::Symbol x0('x', dual_frames.ref_robot_id * max_keyframes), x1('x', frame_id);
#if ROBUSTGTSAM
        auto factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
            boost::make_shared<risam::SIGKernel>(6), dual_frames.second.ref_robot_id * max_keyframes, dual_frame.first, dual_frame.second.pose, const_noise_dual);
        graph_dual.push_back(factor);
#else
        graph_dual.add(gtsam::BetweenFactor(x0, x1, dual.pose, const_noise_s2m));
#endif
      }
      else
      {

        std::shared_ptr<KeyFrame> keyframe;
        if (frame_id / max_keyframes == robot_id)
        {
          keyframe = cur_info_list->getKeyFrame(frame_id);
        }
        else
        {
          keyframe = ref_info_list->getKeyFrame(frame_id);
        }
        if (keyframe != nullptr)
        {
          if (isRobotOptimized(dual_frames.ref_robot_id))
          {

            gtsam::Pose3 cur_map_wrt_ref_map = getPoseWrtRefRobotIdx(dual_frames.ref_robot_id).toPose3();
            gtsam::Pose3 dual_remote_in_cur = cur_map_wrt_ref_map.inverse() * dual.pose;
            gtsam::Pose3 error = keyframe->distributed_pose_in_cur_map.toPose3().inverse() * dual_remote_in_cur;
            dual.pose_error = error;
          }
          gtsam::Symbol x0('x', dual_frames.ref_robot_id * max_keyframes), x1('x', frame_id);
#if ROBUSTGTSAM
          auto factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
              boost::make_shared<risam::SIGKernel>(6), x0, x1, dual.pose, const_noise_dual);
          graph_dual.push_back(factor);
#else
          graph_dual.add(gtsam::BetweenFactor(x0, x1, dual.pose, const_noise_dual));
#endif

          if (init_pose.find(x0.key()) == init_pose.end() &&
              init_pose.find(x1.key()) != init_pose.end())
          {
            gtsam::Pose3 ref_map_wrt_current = init_pose[x1.key()].first * keyframe->getPoseInit().toPose3().inverse();
            std::pair<gtsam::Pose3, gtsam::Pose3> initPair(ref_map_wrt_current, ref_map_wrt_current);
            init_pose.emplace(x0.key(), initPair);
          }
        }
      }
    }
  }
}

bool BackEnd::gtsamInterLoop(const std::vector<LoopClosure> &loop_accepted)
{
  bool rebuild_loop = false;
  gtsam::NonlinearFactorGraph graph_inc_loop;
  // check if all loop closure in graph is in loop queue
  int min_cur_frame_id = std::numeric_limits<int>::max();
  for (const auto &item : loop_in_graph)
  {
    if (item.c_frame_id / max_keyframes < item.r_frame_id / max_keyframes)
    {
      continue;
    }
    bool is_accepted = false;
    for (const auto &loop : loop_accepted)
    {
      if (loop.c_frame_id < min_cur_frame_id)
      {
        min_cur_frame_id = loop.c_frame_id;
      }
      if (!is_accepted && (item.c_frame_id == loop.c_frame_id && item.r_frame_id == loop.r_frame_id))
      {
        is_accepted = true;
      }
    }

    if (!is_accepted)
    {
      std::cout << YELLOW << "[SubMapFusion robot" + std::to_string(robot_id) + "] <BACK END> Loop Graph changed from: "
                << loop_in_graph.size() << ", to: " << loop_accepted.size() << RESET << std::endl;
      rebuild_graph = true;
      break;
    }
  }

  if (rebuild_graph)
  { // if you need rebuild graph, then clear the graph
    std::cout << RED << "[SubMapFusion robot" + std::to_string(robot_id) + "] <BACK END> Loop Graph Reset! " << RESET << std::endl;
    resetOptimization();
    // clear loop
    loop_in_graph.clear();
    graph_loop.resize(0);

    rebuild_loop = true;
  }

  loop_in_graph.clear();
  graph_loop.resize(0);

  for (const auto &loop : loop_accepted)
  {
    if (loop.c_frame_id / max_keyframes < loop.r_frame_id / max_keyframes)
    {
      continue;
    }
    std::shared_ptr<KeyFrame> cur_keyframe;
    cur_keyframe = cur_info_list->getKeyFrame(loop.c_frame_id);
    auto cur_p_in_cur = cur_keyframe->getPoseInit().toPose3();

    bool is_in_graph = false;
    for (const auto &item : loop_in_graph)
    {
      if (item == loop)
      {
        is_in_graph = true;
        break;
      }
    }
    if (is_in_graph)
      continue;
    loop_in_graph.emplace_back(loop);
    gtsam::Symbol x0('x', loop.r_frame_id), x1('x', loop.c_frame_id);

    if (loop.c_frame_id / max_keyframes == loop.r_frame_id / max_keyframes)
    {
      // add intra robot loop closure

#if ROBUSTGTSAM
      auto factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
          boost::make_shared<risam::SIGKernel>(6), loop.r_frame_id,
          loop.c_frame_id, loop.pose,
          const_noise_intra_loop);
      graph_inc_loop.push_back(factor);
#else
      graph_inc_loop.add(gtsam::BetweenFactor<gtsam::Pose3>(x0, x1, loop.pose, const_noise_intra_loop));
#endif
      continue;
    }

    if (debug_ban_type & 0b1000 && (debug_node_id == loop.c_frame_id || debug_node_id == loop.r_frame_id))
    {
      std::cout << "robot_" << robot_id << ": skip loop factor for debug node: " << loop.c_frame_id << "to " << loop.r_frame_id << std::endl;
      continue;
    }
    // add inter robot loop closure
#if ROBUSTGTSAM
    auto factor = risam::make_shared_graduated<gtsam::BetweenFactor<gtsam::Pose3>>(
        boost::make_shared<risam::SIGKernel>(6), loop.r_frame_id,
        loop.c_frame_id, loop.pose,
        const_noise_inter_loop);
    graph_inc_loop.push_back(factor);
#else
    graph_inc_loop.add(gtsam::BetweenFactor<gtsam::Pose3>(x0, x1, loop.pose, const_noise_inter_loop));
#endif
    if (init_pose.find(x0) == init_pose.end())
    {
      std::pair<gtsam::Pose3, gtsam::Pose3> initPair(cur_p_in_cur * loop.pose.inverse(), cur_p_in_cur * loop.pose.inverse());
      init_pose.emplace(x0, initPair);
    }
    else
    {
      init_pose[x0].first = AvgSE3({init_pose[x0].first, cur_p_in_cur * loop.pose.inverse()});
    }
  }

  if (!graph_inc_loop.empty())
  {
    graph_loop += graph_inc_loop;
  }
  graph_inc_loop.resize(0);
  return rebuild_loop;
}

void BackEnd::gtsamInit()
{
  distributed_graph_all.resize(0);
  decentralized_graph_all.resize(0);

  distributed_graph_all += graph_prior;
  decentralized_graph_all += graph_prior;
  distributed_graph_all += graph_scan;
  decentralized_graph_all += graph_scan;
  distributed_graph_all += graph_loop;
  decentralized_graph_all += graph_loop;
  distributed_graph_all += graph_s2m;
  decentralized_graph_all += graph_s2m;
  distributed_graph_all += graph_dual;

  distributed_init_est_all.clear();
  decentralized_init_est_all.clear();
  diff_init.clear();

  gtsam::Symbol x0('x', robot_id * max_keyframes);
  distributed_init_est_all.insert(x0, gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)));
  decentralized_init_est_all.insert(x0, gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)));
  gtsam::VariableIndex distributed_var_index(distributed_graph_all);
  gtsam::VariableIndex decentralized_var_index(decentralized_graph_all);
  for (const auto &item : init_pose)
  {
    if (item.first == x0.key())
      continue;
    gtsam::Symbol x_init(item.first);
    auto initial_pose = item.second.first;
    auto looped_pose = item.second.second;
    if (distributed_var_index.find(item.first) != distributed_var_index.end())
    {
      distributed_init_est_all.insert(x_init, looped_pose);
    }
    if (decentralized_var_index.find(item.first) != decentralized_var_index.end())
    {
      decentralized_init_est_all.insert(x_init, initial_pose);
    }
    else if (distributed_var_index.find(item.first) != distributed_var_index.end())
    {
      diff_init.insert(x_init, looped_pose);
    }
  }
}

void BackEnd::gtsamUpdate(bool arock_flag)
{
  static int count = 0;
  static int opt_count = 0;
  gtsam::Values distributed_estimate, decentralized_estimate; //
  gtsamInit();
  gtsam::LevenbergMarquardtParams lm_params;
  try
  {
    if (opt_count % 100 == 0)
    {
      gtsam::LevenbergMarquardtOptimizer decentralized_optimizer(decentralized_graph_all, decentralized_init_est_all, lm_params);
      decentralized_estimate = decentralized_optimizer.optimize();
    }
    gtsam::LevenbergMarquardtOptimizer distributed_optimizer(distributed_graph_all, distributed_init_est_all, lm_params);
    distributed_estimate = distributed_optimizer.optimize();
    graph_dual.resize(0);
  }
  catch (std::exception &e)
  {
    std::cout << RED << "robot_" << robot_id << ": isam2 update error: " << e.what() << RESET << std::endl;
    error_file << "--------------------------------------------- Time stamp:" << ros::Time::now() << " -------------------------" << std::endl;
    error_file << "<Pc_manger> " + std::to_string(count) + " isam update Error:" << e.what() << std::endl;
    count++;
  }

  // if (print_net) {
  //   printBayesNet();
  //   print_net = false;
  // }

  for (auto &item : init_pose)
  {
    gtsam::Symbol x(item.first);
    if (decentralized_estimate.exists(x))
    {
      item.second.second = decentralized_estimate.at<gtsam::Pose3>(x);
    }
  }

  //  gtsam::Marginals marginals(isam2->getFactorsUnsafe(), distributed_estimate);

  if (arock_flag)
  {

    std::lock_guard<std::mutex> lock(mtx_map_list);

    for (auto sol : distributed_estimate)
    {
      PointTypePose cur_wrt_ref;
      gtsam::Symbol x_tmp(sol.key);
      if (x_tmp.index() % max_keyframes != 0)
        continue;
      cur_wrt_ref.fromPose3(distributed_estimate.at<gtsam::Pose3>(x_tmp).inverse());
      cur_wrt_ref.intensity = 1; // trick to indicate that the result is valid
      if (global_map_trans_optimized.find((int)x_tmp.index() / max_keyframes) == global_map_trans_optimized.end())
      {
        global_map_trans_optimized.emplace((int)x_tmp.index() / max_keyframes, cur_wrt_ref);
      }
      else
      {
        global_map_trans_optimized[(int)x_tmp.index() / max_keyframes] = cur_wrt_ref;
      }
    }

    if (global_map_trans_optimized.find(robot_id) == global_map_trans_optimized.end())
    {
      global_map_trans_optimized.emplace(robot_id, PointTypePose());
    }
    else
    {
      global_map_trans_optimized[robot_id] = PointTypePose();
    }
  }
  else
  {

    std::lock_guard<std::mutex> lock(mtx_map_list);
    global_map_trans_optimized.clear();
    global_map_trans_optimized[robot_id] = PointTypePose();

    std::unordered_map<int, int> min_keyframe_id;
    for (auto sol : distributed_estimate)
    {
      PointTypePose cur_wrt_ref, ref_pose;
      gtsam::Symbol x_tmp(sol.key);
      if (x_tmp.index() % max_keyframes == 0 || x_tmp.index() / max_keyframes == robot_id)
        continue;
      int key_robot_id = (int)x_tmp.index() / max_keyframes;
      if (min_keyframe_id.find(key_robot_id) == min_keyframe_id.end() ||
          x_tmp.index() < min_keyframe_id[key_robot_id])
      {
        min_keyframe_id[key_robot_id] = (int)x_tmp.index();
      }
      else
        continue;
      ref_pose = ref_info_list->getKeyFrame((int)x_tmp.index())->getPoseInit();
      cur_wrt_ref.fromPose3(ref_pose.toPose3() * distributed_estimate.at<gtsam::Pose3>(x_tmp).inverse());
      cur_wrt_ref.intensity = 1; // trick to indicate that the result is valid
      global_map_trans_optimized[(int)x_tmp.index() / max_keyframes] = cur_wrt_ref;
    }
  }

  ref_info_list->resetFlag();
  for (auto i = 0; i < ref_info_list->size(); ++i)
  {
    std::shared_ptr<KeyFrame> ref_frame = ref_info_list->at(i);
    gtsam::Symbol x('x', ref_frame->frame_id);
    if (distributed_estimate.exists(x))
    {
      ref_frame->distributed_pose_in_cur_map.fromPose3(distributed_estimate.at<gtsam::Pose3>(x));
      ref_frame->is_corrected = true;
      ref_frame->has_optimized = true;
    }
    if (decentralized_estimate.exists(x))
    {
      ref_frame->decentralized_pose_in_cur_map.fromPose3(decentralized_estimate.at<gtsam::Pose3>(x));
      ref_frame->get_pose_decentralized = true;
    }
    else if (!ref_frame->get_pose_decentralized)
    {

      ref_frame->decentralized_pose_in_cur_map = ref_frame->distributed_pose_in_cur_map;
      ref_frame->get_pose_decentralized = true;
    }
  }

  cur_info_list->resetFlag();
  for (int i = 0; i < cur_info_list->size(); ++i)
  {
    std::shared_ptr<KeyFrame> cur_frame = cur_info_list->at(i);
    gtsam::Symbol x('x', cur_frame->frame_id);
    if (distributed_estimate.exists(x))
    {
      cur_frame->distributed_pose_in_cur_map.fromPose3(distributed_estimate.at<gtsam::Pose3>(x));
      cur_frame->distributed_pose_in_cur_map.intensity = (float)cur_frame->frame_id; // trick to indicate that the result is valid
      cur_frame->is_corrected = true;
      cur_frame->has_optimized = true;
    }
    if (decentralized_estimate.exists(x))
    {
      cur_frame->decentralized_pose_in_cur_map.fromPose3(decentralized_estimate.at<gtsam::Pose3>(x));
      cur_frame->get_pose_decentralized = true;
    }
    else if (!cur_frame->get_pose_decentralized)
    {

      cur_frame->decentralized_pose_in_cur_map = cur_frame->distributed_pose_in_cur_map;
      cur_frame->get_pose_decentralized = true;
    }
  }
  opt_count++;
}

PointTypePose BackEnd::getPoseWrtRefRobotIdx(int robot_idx)
{
  std::lock_guard<std::mutex> lock(mtx_map_list);
  if (global_map_trans_optimized.find(robot_idx) != global_map_trans_optimized.end())
    return global_map_trans_optimized[robot_idx];
  else
    return {};
}
int BackEnd::getMinKeyOptimized()
{

  std::lock_guard<std::mutex> lock(mtx_map_list);
  auto min_element = std::min_element(global_map_trans_optimized.begin(), global_map_trans_optimized.end(),
                                      [](const auto &lhs, const auto &rhs)
                                      {
                                        return lhs.first < rhs.first;
                                      });

  if (min_element != global_map_trans_optimized.end())
    return min_element->first;
  else
    return robot_id;
}

bool BackEnd::printBayesTree()
{
  std::lock_guard<std::mutex> my_lock(mtx_isam2);
  ROS_ERROR("Saving tree graph!");
  if (isam2->empty())
    return false;
  // save optimized pose graph
  std::string file_name = log_folder + "/robot_" + std::to_string(robot_id) + "_Bayes_tree_graph.dot";
  std::string pdf_file_name = log_folder + "/robot_" + std::to_string(robot_id) + "_Bayes_tree_graph.pdf";
  isam2->saveGraph(file_name);

  std::string cmd = "dot -Tpdf " + file_name + " -o " + pdf_file_name;
  system(cmd.c_str());
  return true;
}

bool BackEnd::printBayesNet()
{
  std::lock_guard<std::mutex> my_lock(mtx_isam2);
  ROS_ERROR("Saving net graph!");
  if (distributed_graph_all.empty())
    return false;
  // save optimized decentralized_pose_in_cur_map graph
  std::string file_name = log_folder + "/robot_" + std::to_string(robot_id) + "_Bayes_Net_graph.dot";
  std::string pdf_file_name = log_folder + "/robot_" + std::to_string(robot_id) + "_Bayes_Net_graph.pdf";
  distributed_graph_all.saveGraph(file_name);

  std::string cmd = "dot -Tpdf " + file_name + " -o " + pdf_file_name;
  system(cmd.c_str());
  return true;
}
