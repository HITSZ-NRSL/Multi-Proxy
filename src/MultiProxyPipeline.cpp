//
// Created by jcwang on 25-9-7.
//

#include "MultiProxyPipeline.h"
void MultiProxyPipeline::paramLoader()
{
  node_handler.param<std::string>("robot_name", _robot_name, "robot_2");
  node_handler.param<int>("robot_id", _robot_id_th, 2);
  node_handler.param<bool>("use_bag_time", _use_bag_time, true);

  // use current package path
  node_handler.param<std::string>("pcm_matrix_folder", _pcm_matrix_folder, string(ROOT_DIR) + "log/pcm_matrix");
  std::cout << "pcm_matrix_folder" << _pcm_matrix_folder << std::endl;
  node_handler.param<std::string>("log_folder", _log_folder, string(ROOT_DIR) + "log");
  node_handler.param<std::string>("pcd_folder", _pcd_folder, string(ROOT_DIR) + "pcd");

  node_handler.getParam("/multiProxy/keyFrameParam/rebuild_thres", _rebuild_thres);
  node_handler.getParam("/multiProxy/keyFrameParam/max_keyframes", _max_keyframes);
  node_handler.getParam("/multiProxy/keyFrameParam/max_robots", _max_robots);
  node_handler.getParam("/multiProxy/keyFrameParam/time_th", _time_th);
  node_handler.getParam("/multiProxy/keyFrameParam/distance_th", _distance_th);
  node_handler.getParam("/multiProxy/keyFrameParam/angle_th", _angle_th);
  node_handler.getParam("/multiProxy/interRobot/key_frame_only", _key_frame_only);
  node_handler.getParam("/multiProxy/interRobot/s2m_std", _s2m_std);
  node_handler.getParam("/multiProxy/interRobot/inter_loop_std", _inter_loop_std);
  node_handler.getParam("/multiProxy/interRobot/intra_loop_std", _intra_loop_std);
  node_handler.getParam("/multiProxy/interRobot/odom_std", _odom_std);
  node_handler.getParam("/multiProxy/interRobot/dual_std", _dual_std);
  node_handler.getParam("/multiProxy/interRobot/downsample_resolution_map", _downsample_resolution_map);
  node_handler.getParam("/multiProxy/interRobot/save_map_resolution", _save_map_resolution);
  node_handler.getParam("/multiProxy/interRobot/pcm_threshold", _pcm_thres);
  node_handler.getParam("/multiProxy/interRobot/icp_threshold", _icp_thres);
  node_handler.getParam("/multiProxy/interRobot/icp_disthr", _icp_disthr);
  node_handler.getParam("/multiProxy/interRobot/icp_maxiter", _icp_maxiter);
  node_handler.getParam("/multiProxy/interRobot/local_odom_topic", _local_odom_topic);
  node_handler.getParam("/multiProxy/interRobot/local_cloud_topic", _local_cloud_topic);
  node_handler.getParam("/multiProxy/interRobot/pcm_start_threshold", _pcm_start_threshold);
  node_handler.getParam("/multiProxy/interRobot/pcm_max_clique_size", _pcm_max_clique_size);
  node_handler.getParam("/multiProxy/interRobot/dual_max_iters", _dual_max_iters);
  node_handler.getParam("/multiProxy/interRobot/dual_min_error", _dual_min_error);
  node_handler.getParam("/multiProxy/interRobot/verification_points", _verification_points);
  node_handler.getParam("/multiProxy/interRobot/use_arock", _use_ARock);
  // std config
  STDConfigSetting config_setting;
  read_std_parameters(node_handler, config_setting);
  _detector_config.config_setting = config_setting;
  ROS_INFO("[subMapFusion] Robot %d ready. use bag time? %d", _robot_id_th, _use_bag_time);
}

void MultiProxyPipeline::initialization()
{
  allocateMemory();
  string init_des_name = _log_folder + "/robot_" + std::to_string(_robot_id_th) + "_init_descriptor.txt";
  string des_file_name = _log_folder + "/robot_" + std::to_string(_robot_id_th) + "_descriptor_matching.txt";
  string icp_file_name = _log_folder + "/robot_" + std::to_string(_robot_id_th) + "_icp_matching.txt";
  string isam_file_name = _log_folder + "/robot_" + std::to_string(_robot_id_th) + "_isam2.txt";
  string descriptor_bw_file_name = _log_folder + "/bw_compare/robot_" + std::to_string(_robot_id_th) + "_descriptor_bw.txt";
  string pc_bw_file_name = _log_folder + "/bw_compare/robot_" + std::to_string(_robot_id_th) + "_pc_bw.txt";
  string pose2pose_bw_file_name = _log_folder + "/bw_compare/robot_" + std::to_string(_robot_id_th) + "_pose2pose_bw.txt";
  string dual_bw_file_name = _log_folder + "/bw_compare/robot_" + std::to_string(_robot_id_th) + "_dual_bw.txt";
  //    string error_file_name = _log_folder + "/robot_" + std::to_string(_robot_id_th) + "_error.txt";
  _loop_detector->setRobotID(_robot_id_th);

  _multi_outlier_rejection->setPcmMatrixFolder(_pcm_matrix_folder);
  _multi_outlier_rejection->setRobotID(_robot_id_th);
  _multi_outlier_rejection->setPcmStartThreshold(_pcm_start_threshold);
  _multi_outlier_rejection->setPcmMaxCliqueSize(_pcm_max_clique_size);
  _multi_outlier_rejection->setPcmThreshold(_pcm_thres);
  std::cout << YELLOW << "pcm_thres: " << _pcm_thres << RESET << std::endl;

  _pc_manager->setPcdFolder(_pcd_folder);
  _pc_manager->setIcpThreshold(_icp_thres);
  _pc_manager->setRobotID(_robot_id_th);
  _pc_manager->setErrorFilePath(_log_folder);

  _back_end->setRobotID(_robot_id_th);
  _back_end->setMaxKeyframes(_max_keyframes);
  _back_end->setS2MStd(_s2m_std);
  _back_end->setOdomStd(_odom_std);
  _back_end->setInterLoopStd(_inter_loop_std);
  _back_end->setIntraLoopStd(_intra_loop_std);
  _back_end->setDualStd(_dual_std);
  _back_end->setKeyFrameOnly(_key_frame_only);
  _back_end->setLogFolder(_log_folder);
  _back_end->resetOptimization();

  _visulization.setRobotID(_robot_id_th);
  _visulization.setRobotName(_robot_name);
  _visulization.setCurKeyFrameList(&_cur_info_list);
  _visulization.setRefKeyFrameList(&_ref_info_list);
  _visulization.setLogFolder(_log_folder);

  _odom_file.open(init_des_name, std::ios::out);
  if (!_odom_file.is_open())
    std::cerr << "Error: cannot open file " << init_des_name << std::endl;
  _descriptor_file.open(des_file_name, std::ios::out);
  if (!_descriptor_file.is_open())
    std::cerr << "Error: cannot open file " << des_file_name << std::endl;
  _icp_check_file.open(icp_file_name, std::ios::out);
  if (!_icp_check_file.is_open())
    std::cerr << "Error: cannot open file " << icp_file_name << std::endl;
  _descriptor_bw_file.open(descriptor_bw_file_name, std::ios::out);
  if (!_descriptor_bw_file.is_open())
    std::cerr << "Error: cannot open file " << descriptor_bw_file_name << std::endl;
  _pc_bw_file.open(pc_bw_file_name, std::ios::out);
  if (!_pc_bw_file.is_open())
    std::cerr << "Error: cannot open file " << pc_bw_file_name << std::endl;
  _pose_bw_file.open(pose2pose_bw_file_name, std::ios::out);
  if (!_pose_bw_file.is_open())
    std::cerr << "Error: cannot open file " << pose2pose_bw_file_name << std::endl;
  _dual_bw_file.open(dual_bw_file_name, std::ios::out);
  if (!_dual_bw_file.is_open())
    std::cerr << "Error: cannot open file " << dual_bw_file_name << std::endl;

  _pc_manager->setMapFilterParam(_downsample_resolution_map, _downsample_resolution_map, _downsample_resolution_map);

  std::cout << "_robot_name=" << _robot_name << ", " << "_robot_id_th=" << _robot_id_th << std::endl;

  _current_frame_count = 0;

  // register
  _srv_save_traj = node_handler.advertiseService("/robot_" + std::to_string(_robot_id_th) + "/multi_proxy/save_traj", &MultiProxyPipeline::saveTrajectoriesService, this);
  _sub_heartbeat = node_handler.subscribe<std_msgs::Header>("/multi_proxy/heartbeat", 1000, &MultiProxyPipeline::heartbeatCallback, this, ros::TransportHints().tcpNoDelay());
  _timer_heartbeat = node_handler.createTimer(ros::Duration(1.0), &MultiProxyPipeline::checkHeartbeatCallback, this);

  _sub_laser_cloud_info = node_handler.subscribe<sensor_msgs::PointCloud2>("/" + _robot_name + "/" + _local_cloud_topic,
                                                                           100, &MultiProxyPipeline::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

  _sub_localization_info = node_handler.subscribe<nav_msgs::Odometry>("/" + _robot_name + "/" + _local_odom_topic,
                                                                      2000, &MultiProxyPipeline::localizationInfoHandler, this, ros::TransportHints().tcpNoDelay());

  std::cout << "[SubMapFusion robot_" << _robot_id_th << "] current robot" << _robot_id_th << std::endl;

  _sub_descriptor_info = node_handler.subscribe<std_msgs::Int8MultiArray>("/robot_" + std::to_string(_robot_id_th) + "/loop/descriptor_info",
                                                                          100, &MultiProxyPipeline::descriptorHandler, this, ros::TransportHints().tcpNoDelay()); // number of buffer may differs for different robot numbers
  _sub_candidate_info = node_handler.subscribe<std_msgs::Int8MultiArray>("/robot_" + std::to_string(_robot_id_th) + "/loop/candidate_info",
                                                                         100, &MultiProxyPipeline::candidateLoopHandler, this, ros::TransportHints().tcpNoDelay()); // number of buffer may differs for different robot numbers
  _sub_verified_info = node_handler.subscribe<std_msgs::Int8MultiArray>("/robot_" + std::to_string(_robot_id_th) + "/loop/verified_loop",
                                                                        20,
                                                                        &MultiProxyPipeline::verifiedLoopTransHandler, this, ros::TransportHints().tcpNoDelay());
  _sub_dual_info = node_handler.subscribe<std_msgs::Int8MultiArray>("/robot_" + std::to_string(_robot_id_th) + "/loop/dual_info",
                                                                    20, &MultiProxyPipeline::dualPriorHandler, this, ros::TransportHints().tcpNoDelay());

  _pub_heartbeat = node_handler.advertise<std_msgs::Header>("/multi_proxy/heartbeat", 10);

  _visulization.pub_fused_map_frame = node_handler.advertise<sensor_msgs::PointCloud2>("/robot_" + std::to_string(_robot_id_th) + "/" + "subFusion/map_registered", 10);
  _visulization.pub_loop_constraint_edge = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/loop_closure_constraints", 10);
  _visulization.pub_key_frame_node = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/key_frame", 10);
  _visulization.pub_dual_viz = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/dual_prior", 10);

  //  std pub
  _pub_STD_current = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/" + "STD_descriptor", 10);
  _pub_STD_received = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/" + "STD_received", 10);
  _pub_STD_matched = node_handler.advertise<visualization_msgs::MarkerArray>("/robot_" + std::to_string(_robot_id_th) + "/" + "stdpair", 10);
}

void MultiProxyPipeline::allocateMemory()
{
  _pc_manager = new pcManager();
  _loop_detector = new LoopDetector(_detector_config);
  _multi_outlier_rejection = new MultiOutlierRejection();
  _back_end = new BackEnd(_robot_id_th, _max_keyframes, &_cur_info_list, &_ref_info_list);
}

void MultiProxyPipeline::releaseMemory()
{

  if (_pc_manager)
  {
    delete _pc_manager;
    _pc_manager = nullptr;
  }
  if (_loop_detector)
  {
    delete _loop_detector;
    _loop_detector = nullptr;
  }
  if (_multi_outlier_rejection)
  {
    delete _multi_outlier_rejection;
    _multi_outlier_rejection = nullptr;
  }
  if (_back_end)
  {
    delete _back_end;
    _back_end = nullptr;
  }
}

bool MultiProxyPipeline::saveTrajectoriesService(multi_proxy::save_traj::Request &req,
                                                 multi_proxy::save_traj::Response &res)
{
  _save_pose_and_map = true;
  res.success = true;
  return true;
}

void MultiProxyPipeline::heartbeatCallback(const std_msgs::HeaderConstPtr &msg)
{
  Utility::TicToc t_heartbeat;
  const std::string &s = msg->frame_id;
  auto pos = s.find('|');
  if (pos == std::string::npos)
  {

    return;
  }

  int id = -1;
  try
  {
    id = std::stoi(s.substr(0, pos));
  }
  catch (const std::exception &e)
  {
    return;
  }

  if (id == _robot_id_th || id == -1)
    return;
  if (_neighbor_id_list.find(id) != _neighbor_id_list.end())
  {
    _neighbor_id_list.at(id).last_heartbeat = msg->stamp.toSec();
  }
  else
  {
    _neighbor_id_list[id].robot_name = s.substr(pos + 1);
    _neighbor_id_list[id].robot_id = id;
    _neighbor_id_list[id].last_heartbeat = msg->stamp.toSec();
    std::string topic_prefix = "/robot_" + std::to_string(_neighbor_id_list[id].robot_id);
    _neighbor_id_list[id].pub_descriptor_info = node_handler.advertise<std_msgs::Int8MultiArray>(topic_prefix + "/loop/descriptor_info", 10);
    _neighbor_id_list[id].pub_candidate_info = node_handler.advertise<std_msgs::Int8MultiArray>(topic_prefix + "/loop/candidate_info", 10);
    _neighbor_id_list[id].pub_verified_info = node_handler.advertise<std_msgs::Int8MultiArray>(topic_prefix + "/loop/verified_loop", 10);
    _neighbor_id_list[id].pub_dual_info = node_handler.advertise<std_msgs::Int8MultiArray>(topic_prefix + "/loop/dual_info", 10);

    _dual_frames[id] = DualFrameList();
    _dual_frames[id].setRobotID(_robot_id_th);
    _dual_frames[id].setRefRobotID(min(id, _robot_id_th));
    _dual_frames[id].setMaxInters(_dual_max_iters);
    _dual_frames[id].setMinError(_dual_min_error);
    _dual_frames[id].setMaxKeyFrames(_max_keyframes);
    if (id > _robot_id_th)
      _neighbor_id_list[id].blocked_descriptor_ids = _cur_info_list.descriptor_dis;
    std::cout << GREEN << "[SubMapFusion robot_" << std::to_string(_robot_id_th) << "] <Heartbeat> Init: " << _neighbor_id_list[id].robot_name << "! Local name robot_" << _neighbor_id_list[id].robot_id << "!" << RESET << std::endl;
  }
  //    std::cout << YELLOW << "[SubMapFusion robot_" << std::to_string(_robot_id_th) << "] <Heartbeat> Process: " << t_heartbeat.toc() << " ms" << RESET << std::endl;
}

void MultiProxyPipeline::checkHeartbeatCallback(const ros::TimerEvent &)
{
  double now = ros::Time::now().toSec();
  double timeout(2.0);

  for (auto &neighbor : _neighbor_id_list)
  {
    const int id = neighbor.second.robot_id;
    double diff = now - neighbor.second.last_heartbeat;
    if (diff > timeout)
    {
      if (neighbor.second.isAlive)
        std::cout << RED << std::fixed << std::setprecision(6) << "robot_" << _robot_id_th << " re: Robot_" << id << " is offline, last heartbeat at " << neighbor.second.last_heartbeat << ", now at " << now << RESET << std::endl;
      neighbor.second.isAlive = false;
    }
    else
    {
      if (!neighbor.second.isAlive)
        std::cout << GREEN << std::fixed << std::setprecision(6) << "robot_" << _robot_id_th << " re: Robot_" << id << " is now online, last heartbeat at " << neighbor.second.last_heartbeat << RESET << std::endl;
      neighbor.second.isAlive = true;
    }
  }

  std_msgs::Header heartbeat_msg;
  heartbeat_msg.seq = (uint32_t)_robot_id_th;
  heartbeat_msg.stamp = ros::Time::now();
  heartbeat_msg.frame_id = std::to_string(_robot_id_th) + "|" + _robot_name;
  _pub_heartbeat.publish(heartbeat_msg);
}

void MultiProxyPipeline::laserCloudInfoHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
{
  Utility::TicToc t_lidar;
  bool found_flag = false;

  KeyFrame frame_lidar(_robot_name, _robot_id_th);

  {
    std::lock_guard<std::mutex> my_lock(_mtx_descriptor_info_list_waiting);
    auto it = _descriptor_info_list_waiting.begin();
    while (it != _descriptor_info_list_waiting.end())
    {
      if (abs(it->cloud_header.stamp.toSec() - msgIn->header.stamp.toSec()) < 0.000001)
      {
        frame_lidar = *it;
        found_flag = true;
        break;
      }
      // if the time of the frame_lidar is larger than the time of msgin, break
      if (it->cloud_header.stamp.toSec() > msgIn->header.stamp.toSec())
      {
        break;
      }
      if (it->cloud_header.stamp.toSec() < msgIn->header.stamp.toSec() - 2.0)
        it = _descriptor_info_list_waiting.erase(it);
      else
        it++;
    }
  }

  pcl::fromROSMsg(*msgIn, *frame_lidar.cloud);
  frame_lidar.time = msgIn->header.stamp.toSec();

  if (!found_flag)
  {
    // load newest data
    std::lock_guard<std::mutex> my_lock(_mtx_descriptor_info_list_waiting);
    frame_lidar.cloud_header = msgIn->header; // new cloud header
    frame_lidar.time = msgIn->header.stamp.toSec();
    _descriptor_info_list_waiting.push_back(frame_lidar);
  }
  else
  {
    // push the bin info into the wait list

    Utility::TicToc t_build;
    frame_lidar.frame_id = _robot_id_th * _max_keyframes + _current_frame_count + 1;
    _current_frame_count++;
    _odom_file << "_____________________________________________________________Time: " << ros::Time::now() << std::endl;
    _odom_file << "frame_id: " << frame_lidar.frame_id << std::endl;
    _odom_file << "time: " << std::fixed << std::setprecision(6) << frame_lidar.time << std::endl;
    _odom_file << "pose_in_cur_map: " << frame_lidar.decentralized_pose_in_cur_map.x << ", " << frame_lidar.decentralized_pose_in_cur_map.y << ", " << frame_lidar.decentralized_pose_in_cur_map.z << ", " << frame_lidar.decentralized_pose_in_cur_map.roll << ", " << frame_lidar.decentralized_pose_in_cur_map.pitch << ", " << frame_lidar.decentralized_pose_in_cur_map.yaw << std::endl;
    _odom_file << "cloud_size: " << frame_lidar.cloud->size() << std::endl;

    isKeyframe(frame_lidar);
    buildFrameList(frame_lidar);
  }
}

// receive the complete localization info from current robot
void MultiProxyPipeline::localizationInfoHandler(const nav_msgs::OdometryConstPtr &msgIn)
{
  Utility::TicToc t_odom;
  bool found_flag = false;
  // put the localization info into the wait list
  KeyFrame frame_lidar(_robot_name, _robot_id_th);

  {
    std::lock_guard<std::mutex> my_lock(_mtx_descriptor_info_list_waiting);
    auto it = _descriptor_info_list_waiting.begin();
    while (it != _descriptor_info_list_waiting.end())
    {
      if (abs(it->cloud_header.stamp.toSec() - msgIn->header.stamp.toSec()) < 0.000001)
      {
        frame_lidar = *it;
        found_flag = true;
        break;
      }
      // if the time of the frame_lidar is larger than the time of msgin, break
      if (it->cloud_header.stamp.toSec() > msgIn->header.stamp.toSec())
      {
        break;
      }
      if (it->cloud_header.stamp.toSec() < msgIn->header.stamp.toSec() - 2.0)
        it = _descriptor_info_list_waiting.erase(it);
      else
        it++;
    }
  }
  if (_use_bag_time)
    _time_lidar_stamp = msgIn->header.stamp;
  else
    _time_lidar_stamp = ros::Time::now();

  frame_lidar.decentralized_pose_in_cur_map.x = (float)msgIn->pose.pose.position.x;
  frame_lidar.decentralized_pose_in_cur_map.y = (float)msgIn->pose.pose.position.y;
  frame_lidar.decentralized_pose_in_cur_map.z = (float)msgIn->pose.pose.position.z;
  Eigen::Quaternionf quater((float)msgIn->pose.pose.orientation.w,
                            (float)msgIn->pose.pose.orientation.x,
                            (float)msgIn->pose.pose.orientation.y,
                            (float)msgIn->pose.pose.orientation.z);
  Eigen::Vector3f eulerAngle = quater.toRotationMatrix().eulerAngles(2, 1, 0);
  frame_lidar.decentralized_pose_in_cur_map.roll = eulerAngle(2);
  frame_lidar.decentralized_pose_in_cur_map.pitch = eulerAngle(1);
  frame_lidar.decentralized_pose_in_cur_map.yaw = eulerAngle(0);
  frame_lidar.setPoseInit(frame_lidar.decentralized_pose_in_cur_map);
  frame_lidar.distributed_pose_in_cur_map = frame_lidar.decentralized_pose_in_cur_map;

  if (!found_flag)
  {
    // load newest data
    std::lock_guard<std::mutex> my_lock(_mtx_descriptor_info_list_waiting);
    frame_lidar.cloud_header = msgIn->header; // new cloud header
    frame_lidar.time = msgIn->header.stamp.toSec();
    _descriptor_info_list_waiting.push_back(frame_lidar);
  }
  else
  {
    // push the bin info into the wait list
    Utility::TicToc t_build;
    frame_lidar.frame_id = _robot_id_th * _max_keyframes + _current_frame_count + 1;
    _current_frame_count++;
    _odom_file << "_____________________________________________________________Time: " << ros::Time::now() << std::endl;
    _odom_file << "frame_id: " << frame_lidar.frame_id << std::endl;
    _odom_file << "time: " << std::fixed << std::setprecision(6) << frame_lidar.time << std::endl;
    _odom_file << "decentralized_pose_in_cur_map: " << frame_lidar.decentralized_pose_in_cur_map.x << ", " << frame_lidar.decentralized_pose_in_cur_map.y << ", " << frame_lidar.decentralized_pose_in_cur_map.z << ", " << frame_lidar.decentralized_pose_in_cur_map.roll << ", " << frame_lidar.decentralized_pose_in_cur_map.pitch << ", " << frame_lidar.decentralized_pose_in_cur_map.yaw << std::endl;
    _odom_file << "cloud_size: " << frame_lidar.cloud->size() << std::endl;

    isKeyframe(frame_lidar);
    buildFrameList(frame_lidar);
  }
}

void MultiProxyPipeline::descriptorHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn)
{
  Utility::TicToc t_descriptor;
  const std_msgs::Int8MultiArray &descriptor_msg = *msgIn;
  KeyFrame sc_info(descriptor_msg);

  buildFrameList(sc_info); // add candidate frame to the descriptor matching
}

// receive the candidate loop with initial pose and scan pointcloud
void MultiProxyPipeline::candidateLoopHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn)
{
  Utility::TicToc t_candidate;
  CandidateInfo candidate_info_input;

  vector<int8_t, allocator<int8_t>> data = msgIn->data;
  size_t offset = 0;

  auto readString = [&](std::string &outStr)
  {
    uint32_t len = *reinterpret_cast<const uint32_t *>(&data[offset]);
    offset += sizeof(uint32_t);
    outStr = std::string(data.begin() + offset, data.begin() + offset + len);
    offset += len;
  };

  auto readFloat = [&]()
  {
    float f = *reinterpret_cast<const float *>(&data[offset]);
    offset += sizeof(float);
    return f;
  };

  auto readUint8 = [&]()
  {
    uint8_t val = *reinterpret_cast<const uint8_t *>(&data[offset]);
    offset += sizeof(uint8_t);
    return val;
  };

  auto readUint32 = [&]()
  {
    uint32_t val = *reinterpret_cast<const uint32_t *>(&data[offset]);
    offset += sizeof(uint32_t);
    return val;
  };

  auto readInt32 = [&]()
  {
    int32_t val = *reinterpret_cast<const int32_t *>(&data[offset]);
    offset += sizeof(int32_t);
    return val;
  };

  // 1. Strings
  readString(candidate_info_input.robotName);
  candidate_info_input.robotID = readInt32();

  candidate_info_input.candidateFrameId = readInt32();

  candidate_info_input.pose.x = readFloat();
  candidate_info_input.pose.y = readFloat();
  candidate_info_input.pose.z = readFloat();
  candidate_info_input.pose.roll = readFloat();
  candidate_info_input.pose.pitch = readFloat();
  candidate_info_input.pose.yaw = readFloat();
  candidate_info_input.pose.intensity = readFloat();

  candidate_info_input.loopFrameId = readInt32();

  candidate_info_input.initial_guess.x = readFloat();
  candidate_info_input.initial_guess.y = readFloat();
  candidate_info_input.initial_guess.z = readFloat();
  candidate_info_input.initial_guess.roll = readFloat();
  candidate_info_input.initial_guess.pitch = readFloat();
  candidate_info_input.initial_guess.yaw = readFloat();

  uint32_t cloud_size = readUint32();
  candidate_info_input.scanCloud.data.resize(cloud_size);
  candidate_info_input.scanCloud.height = readUint32();
  candidate_info_input.scanCloud.width = readUint32();
  candidate_info_input.scanCloud.point_step = readUint32();
  candidate_info_input.scanCloud.row_step = readUint32();
  candidate_info_input.scanCloud.is_dense = readUint8();

  uint32_t num_fields = readUint32();
  candidate_info_input.scanCloud.fields.resize(num_fields);
  for (uint32_t i = 0; i < num_fields; ++i)
  {
    readString(candidate_info_input.scanCloud.fields[i].name);
    candidate_info_input.scanCloud.fields[i].offset = readUint32();
    candidate_info_input.scanCloud.fields[i].datatype = readUint8();
    candidate_info_input.scanCloud.fields[i].count = readUint32();
  }
  std::copy(data.begin() + offset, data.begin() + offset + cloud_size, candidate_info_input.scanCloud.data.begin());
  offset += cloud_size;

  KeyFrame sc_info(candidate_info_input);

  buildFrameList(sc_info); // add candidate frame to the descriptor matching
  // if the data is sent by another robot, add its data size
  {
    std::lock_guard<std::mutex> my_lock(_mtx_candidate_verification);
    _candidate_loop_info_list.push_back(candidate_info_input);
  }
}

void MultiProxyPipeline::verifiedLoopTransHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn)
{
  Utility::TicToc t_inter_loop;
  vector<int8_t, allocator<int8_t>> data = msgIn->data;
  size_t offset = 0;

  auto readFloat = [&]()
  {
    float f = *reinterpret_cast<const float *>(&data[offset]);
    offset += sizeof(float);
    return f;
  };

  auto readInt32 = [&]()
  {
    int32_t val = *reinterpret_cast<const int32_t *>(&data[offset]);
    offset += sizeof(int32_t);
    return val;
  };

  int root_id = readInt32();
  int leaf_id = readInt32();

  if (_robot_id_th != leaf_id / _max_keyframes)
  {
    return;
  }

  std::shared_ptr<KeyFrame> sc_current = _cur_info_list.getKeyFrame(leaf_id);
  int robot_reference_id = root_id / _max_keyframes;

  PointTypePose cur_p_in_ref_p;
  cur_p_in_ref_p.x = readFloat();
  cur_p_in_ref_p.y = readFloat();
  cur_p_in_ref_p.z = readFloat();
  cur_p_in_ref_p.roll = readFloat();
  cur_p_in_ref_p.pitch = readFloat();
  cur_p_in_ref_p.yaw = readFloat();
  cur_p_in_ref_p.intensity = readFloat();

  gtsam::Pose3 gt_cur_p_in_ref_p = cur_p_in_ref_p.toPose3();
  gtsam::Matrix cov = gtsam::Matrix::Zero(6, 6);
  if (!sc_current->is_degenerate)
  {
    std::lock_guard<std::mutex> lock(_mtx_loop_queue);
    auto ite = _loop_queue.find(robot_reference_id);
    gtsam::Pose3 cur_pose, ref_pose;
    cur_pose = sc_current->getPoseInit().toPose3();
    ref_pose = _ref_info_list.getKeyFrame(root_id)->getPoseInit().toPose3();

    LoopClosure new_loop_closure(root_id, leaf_id, gt_cur_p_in_ref_p, ref_pose * gt_cur_p_in_ref_p * cur_pose.inverse(), cov);
    if (ite == _loop_queue.end())
    {
      _loop_queue.emplace(robot_reference_id, std::vector<LoopClosure>({new_loop_closure}));
    }
    else
    {
      _loop_queue[robot_reference_id].emplace_back(new_loop_closure);
    }
  }
}

void MultiProxyPipeline::dualPriorHandler(const std_msgs::Int8MultiArrayConstPtr &msgIn)
{
  Utility::TicToc t_dual_prior;
  vector<int8_t, allocator<int8_t>> data = msgIn->data;
  size_t offset = 0;

  auto readFloat = [&]()
  {
    float f = *reinterpret_cast<const float *>(&data[offset]);
    offset += sizeof(float);
    return f;
  };

  auto readInt8 = [&]()
  {
    int8_t val = *reinterpret_cast<const int8_t *>(&data[offset]);
    offset += sizeof(int8_t);
    return val;
  };

  auto readInt32 = [&]()
  {
    int32_t val = *reinterpret_cast<const int32_t *>(&data[offset]);
    offset += sizeof(int32_t);
    return val;
  };

  int robot_id = (int)readInt8();
  int regenerate = (int)readInt8();
  if (_dual_frames[robot_id].regenerate)
    return;
  std::lock_guard<std::mutex> lock(_mtx_dual_frame);

  if (regenerate && !_dual_frames[robot_id].regenerate)
  {
    _dual_frames[robot_id].dual_frames_local.clear();
    _dual_frames[robot_id].dual_frames_remote.clear();
    std::cout << YELLOW << "Robot " << _robot_id_th << ", Ref robot:" << robot_id << ", clear local dual " << RESET << std::endl;
  }

  int frame_num = readInt32();
  std::vector<int> frame_id(frame_num);
  std::vector<float> x(frame_num), y(frame_num), z(frame_num),
      roll(frame_num), pitch(frame_num), yaw(frame_num);
  for (int i = 0; i < frame_num; i++)
  {
    frame_id[i] = readInt32();
    x[i] = readFloat();
    y[i] = readFloat();
    z[i] = readFloat();
    roll[i] = readFloat();
    pitch[i] = readFloat();
    yaw[i] = readFloat();
  }

  for (int i = 0; i < frame_num; i++)
  {
    gtsam::Pose3 dual_pose_in_min_robot(gtsam::Rot3::RzRyRx(roll[i], pitch[i], yaw[i]),
                                        gtsam::Point3(x[i], y[i], z[i]));
    if (frame_id[i] >= 0)
    {

      DualFrame dual_in(robot_id, dual_pose_in_min_robot);
      _dual_frames[robot_id].update_remote(frame_id[i], dual_in);
    }

    if (std::abs(frame_id[i]) / _max_keyframes == _robot_id_th && std::abs(frame_id[i]) % _max_keyframes != 0 &&
        _dual_frames[robot_id].dual_frames_local.find(std::abs(frame_id[i])) == _dual_frames[robot_id].dual_frames_local.end())
    {
      std::shared_ptr<KeyFrame> cur_keyframe = _cur_info_list.getKeyFrame(std::abs(frame_id[i]));
      gtsam::Pose3 cur_p;
      if (cur_keyframe->has_optimized)
      {
        cur_p = cur_keyframe->decentralized_pose_in_cur_map.toPose3();
      }
      else
      {
        cur_p = cur_keyframe->getPoseInit().toPose3();
      }

      if (frame_id[i] >= 0)
      {
        std::cout << RED << "Error! Robot " << _robot_id_th << ", Ref robot:" << robot_id << ", receive wrong dual frame id: " << frame_id[i] << RESET << std::endl;
      }
      DualFrame dual_new_curr(_robot_id_th, cur_p);
      _dual_frames[robot_id].add_local(std::abs(frame_id[i]), dual_new_curr);
    }
    else if (std::abs(frame_id[i]) / _max_keyframes != _robot_id_th && std::abs(frame_id[i]) % _max_keyframes != 0 &&
             _dual_frames[robot_id].dual_frames_local.find(std::abs(frame_id[i])) == _dual_frames[robot_id].dual_frames_local.end())
    {

      DualFrame dual_in(robot_id, dual_pose_in_min_robot);
      _dual_frames[robot_id].add_local(std::abs(frame_id[i]), dual_in);
      if (frame_id[i] < 0)
      {
        std::cout << RED << "Error! Robot " << _robot_id_th << ", Ref robot:" << robot_id << ", receive wrong dual frame id: " << frame_id[i] << RESET << std::endl;
      }

      if (_dual_frames[robot_id].dual_frames_local.find(-std::abs(frame_id[i])) != _dual_frames[robot_id].dual_frames_local.end())
      {
        _dual_frames[robot_id].dual_frames_local.erase(-std::abs(frame_id[i]));
      }
    }
  }
}

bool MultiProxyPipeline::isKeyframe(KeyFrame &descriptor_info)
{
  static double last_key_time = 0;
  static PointTypePose last_pose;

  if (_cur_info_list.empty() && descriptor_info.robot_id == _robot_id_th)
    descriptor_info.is_key_frame = true;
  else if (descriptor_info.robot_id == _robot_id_th)
  {
    if ((descriptor_info.time - last_key_time > _time_th &&
         descriptor_info.getPoseInit().distance(last_pose) > _distance_th) ||
        descriptor_info.frame_id % _max_keyframes < _detector_config.config_setting.sub_frame_num_)
    {
      descriptor_info.is_key_frame = true;
    }
  }
  if (descriptor_info.is_key_frame)
  {
    last_key_time = descriptor_info.time;
    last_pose = descriptor_info.getPoseInit();
  }

  return descriptor_info.is_key_frame;
}

// build two frame_list, one is for the corrent robot, the other is for the other robots
void MultiProxyPipeline::buildFrameList(KeyFrame &context_in)
{
  auto &target_list = (context_in.robot_name == _robot_name) ? _cur_info_list : _ref_info_list;

  if (!target_list.getKeyFrame(context_in.frame_id))
  {
    target_list.addKeyFrame(std::make_shared<KeyFrame>(context_in));
  }
}

void MultiProxyPipeline::buildDescriptor()
{
  static int count = 0;
  Utility::TicToc t_std;
  if (_cur_info_list.size() <= count + _detector_config.config_setting.sub_frame_num_ / 2)
    return;
  t_std.tic();
  std::shared_ptr<KeyFrame> sc_current = _cur_info_list.at(count);
  if (sc_current->is_key_frame)
  {
    pcl::PointCloud<PointType>::Ptr sub_map_temp(new pcl::PointCloud<PointType>());
    _cur_info_list.buildMapWrtLocalMap(sc_current->frame_id, sub_map_temp,
                                       _detector_config.config_setting.sub_frame_num_);
    _loop_detector->makeDescriptorsSTD(sub_map_temp, sc_current);
    sc_current->publishSTD(_pub_STD_current);

    // ADD to STD manager
    _loop_detector->addHistoryDescriptorSTD(sc_current);
    sub_map_temp->clear();

    int loop_idx;
    PointTypePose cur_map_error;
    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    if ((!sc_current->stdesc.empty() || !sc_current->std_corner->empty()) &&
        _loop_detector->searchCandidateFrameSTD(sc_current, cur_map_error, loop_idx, loop_std_pair))
    {
      sc_current->is_checkdes = true;
      PointTypePose cur_p_in_loop_p_init;
      std::shared_ptr<KeyFrame> sc_loop = _cur_info_list.getKeyFrame(loop_idx);
      if (nullptr != sc_loop)
      {
        std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <Make Descriptor> single robot loop: " << loop_idx << " <-> " << sc_current->frame_id << RESET << std::endl;
        cur_p_in_loop_p_init = sc_loop->getPoseInit().inverse() * cur_map_error.inverse() * sc_current->getPoseInit();
        CandidateInfo candidate_loop;
        candidate_loop.robotName = _robot_name;
        candidate_loop.robotID = _robot_id_th;

        candidate_loop.candidateFrameId = sc_current->frame_id;
        candidate_loop.loopFrameId = loop_idx;
        candidate_loop.initial_guess.x = cur_p_in_loop_p_init.x;
        candidate_loop.initial_guess.y = cur_p_in_loop_p_init.y;
        candidate_loop.initial_guess.z = cur_p_in_loop_p_init.z;
        candidate_loop.initial_guess.roll = cur_p_in_loop_p_init.roll;
        candidate_loop.initial_guess.pitch = cur_p_in_loop_p_init.pitch;
        candidate_loop.initial_guess.yaw = cur_p_in_loop_p_init.yaw;
        _cur_info_list.buildMapNB(sc_current->frame_id, sub_map_temp, 10);
        _pc_manager->randomFilter(sub_map_temp, sub_map_temp, 5000);
        pcl::toROSMsg(*sub_map_temp, candidate_loop.scanCloud);
        {
          std::lock_guard<std::mutex> my_lock(_mtx_candidate_verification);
          _candidate_loop_info_list.push_back(candidate_loop);
        }
        sub_map_temp->clear();
      }
    }
    sc_current->stdesc.clear();
    _pc_manager->randomFilter(sc_current->cloud, sc_current->cloud, 3000);
    // publish scan context info to other robots
    for (auto &neighbor : _neighbor_id_list)
    {
      if (_robot_id_th >= neighbor.second.robot_id)
        continue;
      neighbor.second.blocked_descriptor_ids.push_back(sc_current->frame_id);
    }
    _cur_info_list.descriptor_dis.push_back(sc_current->frame_id);
  }
  count++;
}

void MultiProxyPipeline::publishBlockedDescriptor()
{

  for (auto &neighbor : _neighbor_id_list)
  {
    if (_robot_id_th > neighbor.second.robot_id)
      continue;
    if (!neighbor.second.isAlive)
      continue;
    if (neighbor.second.blocked_descriptor_ids.empty())
      continue;
    int first_id = neighbor.second.blocked_descriptor_ids.front();
    neighbor.second.blocked_descriptor_ids.pop_front();
    std::shared_ptr<KeyFrame> frame = _cur_info_list.getKeyFrame(first_id);
    frame->publishDescriptorStream(neighbor.second.pub_descriptor_info, _descriptor_bw_file);

    if (neighbor.second.blocked_descriptor_ids.empty())
      continue;
    int last_id = neighbor.second.blocked_descriptor_ids.back();
    neighbor.second.blocked_descriptor_ids.pop_back();
    frame = _cur_info_list.getKeyFrame(last_id);
    frame->publishDescriptorStream(neighbor.second.pub_descriptor_info, _descriptor_bw_file);
  }
}

//     get the initial guess of with scancontext registration

void MultiProxyPipeline::getCandidateLoop()
{
  // Write to file
  int candidate_loop_id;
  Utility::TicToc t_loop;
  static double acc_time = 0;
  static int count = 0;
  static int ref_check_count = 0;
  static int ref_check_init = 0;
  if (_ref_info_list.size() > ref_check_init)
  {
    ref_check_count = _ref_info_list.size();
    ref_check_init = ref_check_count;
  }
  --ref_check_count;
  if (ref_check_count < 0)
    ref_check_count = ref_check_init - 1 < 0 ? 0 : ref_check_init - 1;

  if (!_cur_info_list.empty() && !_ref_info_list.empty())
  {

    if (!_ref_info_list.at(ref_check_count)->is_checkdes &&
        _robot_id_th > _ref_info_list.at(ref_check_count)->robot_id &&
        !_ref_info_list.at(ref_check_count)->std_corner->empty() &&
        _ref_info_list.at(ref_check_count)->checkCount() &&
        _ref_info_list.at(ref_check_count)->is_key_frame)
    {
      PointTypePose cur_map_in_ref_map;
      PointTypePose visual_pose;
      std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
      shared_ptr<KeyFrame> sc_reference = _ref_info_list.at(ref_check_count);
      if (nullptr == sc_reference)
      {
        std::cout << RED << "sc_reference is nullptr!!!!" << RESET << std::endl;
        return;
      }
      if (sc_reference->stdesc.empty() && sc_reference->std_corner->empty())
        return;
      if (_back_end->isRobotOptimized(sc_reference->robot_id))
        visual_pose = _back_end->getPoseWrtRefRobotIdx(sc_reference->robot_id);
      sc_reference->publishSTD(_pub_STD_received);
      if (_loop_detector->searchCandidateFrameSTD(sc_reference, cur_map_in_ref_map, candidate_loop_id, loop_std_pair))
      {
        sc_reference->is_checkdes = true;
        PointTypePose cur_p_in_ref_p_init;

        std::shared_ptr<KeyFrame> sc_current = _cur_info_list.getKeyFrame(candidate_loop_id);
        if (nullptr == sc_current)
        {
          std::cout << RED << "sc_current is nullptr!!!!" << RESET << std::endl;
          return;
        }
        cur_p_in_ref_p_init = sc_reference->getPoseInit().inverse() * cur_map_in_ref_map * sc_current->getPoseInit();
        LoopClosure candidate_loop(sc_reference->frame_id, sc_current->frame_id,
                                   cur_p_in_ref_p_init.toPose3(), sc_current->getPoseInit().toPose3() * cur_p_in_ref_p_init.toPose3().inverse() * sc_reference->getPoseInit().toPose3().inverse(),
                                   gtsam::Matrix::Zero(6, 6));
        _neighbor_id_list.at(sc_reference->robot_id).blocked_candidate_ids.push_back(candidate_loop);
        _loop_detector->pubSTDPairs(sc_reference->robot_name, loop_std_pair,
                                    visual_pose, _pub_STD_matched);
        std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <LoopDetector> inter loop: " << candidate_loop_id << " <-> " << sc_reference->frame_id << RESET << std::endl;
      }
      else
        sc_reference->punishment();
    }
  }
}

void MultiProxyPipeline::publishBlockedCandidateLoop()
{

  for (auto &neighbor : _neighbor_id_list)
  {
    if (_robot_id_th <= neighbor.second.robot_id)
      continue;
    if (!neighbor.second.isAlive)
      continue;
    if (neighbor.second.blocked_candidate_ids.empty())
      continue;
    LoopClosure first_loop = neighbor.second.blocked_candidate_ids.front();
    neighbor.second.blocked_candidate_ids.pop_front();
    publishCandidateLoop(first_loop, neighbor.second);

    if (neighbor.second.blocked_candidate_ids.empty())
      continue;                                                           // 如果没有被阻塞的帧，跳过
    LoopClosure last_loop = neighbor.second.blocked_candidate_ids.back(); // 取出最后一个被阻塞的帧
    neighbor.second.blocked_candidate_ids.pop_back();
    publishCandidateLoop(last_loop, neighbor.second);
  }
}

void MultiProxyPipeline::publishCandidateLoop(LoopClosure &loop, const RobotID &neighbor)
{
  std::shared_ptr<KeyFrame> first_frame = _cur_info_list.getKeyFrame(loop.c_frame_id);
  pcl::PointCloud<PointType>::Ptr sub_map_temp(new pcl::PointCloud<PointType>());
  _cur_info_list.buildMapNB(first_frame->frame_id, sub_map_temp, _detector_config.config_setting.sub_frame_num_);
  _pc_manager->randomFilter(sub_map_temp, sub_map_temp, _verification_points);
  PointTypePose cur_p_wrt_ref_p;
  cur_p_wrt_ref_p.fromPose3(loop.pose);
  first_frame->publishCandidateStream(neighbor.pub_candidate_info, _pc_bw_file, sub_map_temp,
                                      loop.r_frame_id, cur_p_wrt_ref_p);
}

void MultiProxyPipeline::geometricVerification()
{
  static int count = 0;
  static auto begin_time = clock();
  static double acc_time = 0;
  if (_candidate_loop_info_list.empty())
    return;
  //    std::cout << "[SubMapFusion "  + _robot_name + "] <geometricVerification> candidate size:" << _candidate_loop_info_list.size() << std::endl;

  CandidateInfo candidate_loop_info; // 含有点云的描述子信息
  {
    std::lock_guard<std::mutex> my_lock(_mtx_candidate_verification);
    candidate_loop_info = _candidate_loop_info_list.front();
    _candidate_loop_info_list.pop_front();
  }

  std::shared_ptr<KeyFrame> sc_candidate = nullptr;
  if (candidate_loop_info.candidateFrameId / _max_keyframes == _robot_id_th)
    sc_candidate = _cur_info_list.getKeyFrame(candidate_loop_info.candidateFrameId);
  else
    sc_candidate = _ref_info_list.getKeyFrame(candidate_loop_info.candidateFrameId);

  if (sc_candidate == nullptr)
  {
    std::cout << RED << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <geometricVerification> sc_candidate is nullptr: " << candidate_loop_info.candidateFrameId << RESET << std::endl;
    return;
  }

  if (sc_candidate->cloud->empty())
  {
    pcl::fromROSMsg(candidate_loop_info.scanCloud, *sc_candidate->cloud);
    std::cout << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <geometricVerification> update cloud!" << std::endl;
  }

  auto stamp_time = clock();
  _icp_check_file << "---------------------------------------------------------------Time stamp:" << ros::Time::now() << std::endl;
  _icp_check_file << "count:" << count << std::endl;
  _icp_check_file << "loop frame id: " << sc_candidate->frame_id << "-->" << candidate_loop_info.loopFrameId << std::endl;
  _icp_check_file << "initpos: (" << candidate_loop_info.initial_guess.x << ", " << candidate_loop_info.initial_guess.y << ", " << candidate_loop_info.initial_guess.z << "); initrot: ("
                  << candidate_loop_info.initial_guess.roll << ", " << candidate_loop_info.initial_guess.pitch << ", " << candidate_loop_info.initial_guess.yaw << ")" << std::endl;

  if (candidate_loop_info.loopFrameId > 0 && candidate_loop_info.loopFrameId / _max_keyframes == _robot_id_th)
  {
    PointTypePose trans_init{candidate_loop_info.initial_guess.x, candidate_loop_info.initial_guess.y, candidate_loop_info.initial_guess.z,
                             candidate_loop_info.initial_guess.roll, candidate_loop_info.initial_guess.pitch, candidate_loop_info.initial_guess.yaw, 0, 0};
    Eigen::Affine3f sc_initial = trans_init.toAffine();
    int cur_frame_id = (int)candidate_loop_info.loopFrameId;
    std::shared_ptr<KeyFrame> sc_current = _cur_info_list.getKeyFrame(cur_frame_id);
    pcl::PointCloud<PointType>::Ptr cloud_current(new pcl::PointCloud<PointType>());
    _cur_info_list.buildMapNB(cur_frame_id, cloud_current, 20);
    // 这里会发布一次回环结果
    if (!getInitialGuess(cloud_current, *sc_current, *sc_candidate, sc_initial))
    {
      _icp_check_file << "empty point current frame: " << std::endl;
      return;
    }
    _icp_check_file << "time: " << sc_candidate->time << " " << sc_current->time << std::endl;
  } // if has matched another frame

  count++;
  acc_time += (double)(clock() - stamp_time) * 1000 / CLOCKS_PER_SEC;
  _icp_check_file << "icp & s2m time :" << acc_time / count << "/" << (double)(clock() - begin_time) * 1000 / CLOCKS_PER_SEC << std::endl;
}

// icp计算函数输入两个点云，输出第一片点云对齐第二块点云需要进行的仿射变换右乘原机器人1pose在机器人2基坐标系的仿射变换，也就是优化后的pose1在tar机器人基座标中的T,pose_initial是一个东西
// 现在大id的机器人会进到这里来
bool MultiProxyPipeline::getInitialGuess(const pcl::PointCloud<PointType>::Ptr &currentCloud, KeyFrame &sc_current, KeyFrame &sc_reference, Eigen::Affine3f &sc_initial)
{
  PointTypePose ref_p_in_cur_p_init, target_pose, current_pose;
  int iterations = 0;
  float distance = 0.1;
  static int count = 0;
  int robot_reference_id = sc_reference.robot_id;
  Utility::TicToc t_icp;
  if (currentCloud->empty())
    return false;

  // get initial guess from scancontext
  target_pose = sc_reference.getPoseInit();
  current_pose = sc_current.getPoseInit();
  // icp: source to target
  //  icp计算函数输入两个点云，输出第一片点云对齐第二块点云需要进行的仿射变换右乘原机器人1pose在机器人2基坐标系的仿射变换，也就是优化后的pose1在tar机器人基座标中的T,pose_initial是一个东西
  PointTypePose ref_p_in_cur_p_map{0, 0, 0, 0, 0, 0, 100000, 0}, ref_p_in_cur_p{0, 0, 0, 0, 0, 0, 100000, 0};
  // find the decentralized_pose_in_cur_map constrain

  // 计算参考机器人,用描述子匹配结果再次计算
  ref_p_in_cur_p_init.fromAffine(sc_initial);
  _pc_manager->icpIter.Maxiterate = _icp_maxiter;
  _pc_manager->icpIter.distance_threshold = _icp_disthr;
#if USE_CUDA
  t_icp.tic();
  _pc_manager->cuICP(KeyFrame::transformPointCloud(sc_reference.cloud, ref_p_in_cur_p_init),
                     currentCloud, ref_p_in_cur_p_init, ref_p_in_cur_p);
//    std::cout << "CUPCL ICP by Time: " << t_icp.toc() << " ms. " << "FGICP cost: " << ref_p_in_cur_p.intensity << std::endl;
#else
  t_icp.tic();
  _pc_manager->GICP(KeyFrame::transformPointCloud(_pc_manager->filterInvalidPoints(sc_reference.cloud), ref_p_in_cur_p_init),
                    _pc_manager->filterInvalidPoints(currentCloud), ref_p_in_cur_p_init, ref_p_in_cur_p);
  _pc_manager->printPointCloud(sc_reference.transformPointCloud(ref_p_in_cur_p_init), currentCloud,
                               "robot_" + std::to_string(_robot_id_th) + "/" + std::to_string(sc_reference.frame_id) + "to" + std::to_string(sc_current.frame_id) + "_" + std::to_string((int)(ref_p_in_cur_p.intensity * 1000)) + "_sc");
#endif
  if (ref_p_in_cur_p.intensity == -1 && ref_p_in_cur_p_map.intensity != -1)
  {
    ref_p_in_cur_p = ref_p_in_cur_p_map;
  }
  else if (ref_p_in_cur_p_map.intensity != -1 && ref_p_in_cur_p.intensity != -1)
  {
    ref_p_in_cur_p = ref_p_in_cur_p_map.intensity > ref_p_in_cur_p.intensity ? ref_p_in_cur_p : ref_p_in_cur_p_map;
  }

  _icp_check_file << "icp_cost: " << ref_p_in_cur_p.intensity << std::endl;
  if (ref_p_in_cur_p.intensity == -1 || ref_p_in_cur_p.intensity > _icp_thres)
  {
    //      std::cout << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <getInitialGuess> ICP failed: " << ref_p_in_cur_p.intensity << " / " << _icp_thres << std::endl;
    return false;
  }
#if POINTCLOUD_VISUALIZATION
  _pc_manager->printPointCloud(sc_reference.transformPointCloud(ref_p_in_cur_p), currentCloud,
                               "robot_" + std::to_string(_robot_id_th) + "/" + std::to_string(sc_reference.frame_id) + "to" +
                                   std::to_string(sc_current.frame_id) + "_" + std::to_string((int)(ref_p_in_cur_p.intensity * 1000)) + "_after_icp");
#endif
  _icp_check_file << "icp_pos: (" << ref_p_in_cur_p.x << ", " << ref_p_in_cur_p.y << ", " << ref_p_in_cur_p.z << "); icp_rot: ("
                  << ref_p_in_cur_p.roll << ", " << ref_p_in_cur_p.pitch << ", " << ref_p_in_cur_p.yaw << ")" << std::endl;

  gtsam::Matrix cov;
  LoopClosure new_loop_closure(sc_reference.frame_id,
                               sc_current.frame_id,
                               ref_p_in_cur_p.toPose3().inverse(),
                               target_pose.toPose3() * ref_p_in_cur_p.toPose3().inverse() * current_pose.toPose3().inverse(),
                               cov);

  // 机器人间回环发送会相关机器人, 当前机器人回环则直接保存到队列等待外点滤除
  if (robot_reference_id != _robot_id_th)
  {
    _neighbor_id_list.at(robot_reference_id).blocked_verified_ids.push_back(new_loop_closure);
  }
  else
  {
    // Intra robot loop
    std::lock_guard<std::mutex> lock(_mtx_loop_queue);
    auto ite = _loop_queue.find(robot_reference_id);
    if (ite == _loop_queue.end())
    {
      _loop_queue.emplace(robot_reference_id, std::vector<LoopClosure>({new_loop_closure}));
    }
    else
    {
      _loop_queue[robot_reference_id].emplace_back(new_loop_closure);
    }
  }
  return true;
}

void MultiProxyPipeline::publishBlockedVerifiedLoop()
{
  // publish verified loop info to other robots
  for (auto &neighbor : _neighbor_id_list)
  {
    if (_robot_id_th >= neighbor.second.robot_id)
      continue;
    if (!neighbor.second.isAlive)
      continue;
    if (neighbor.second.blocked_verified_ids.empty())
      continue;                                                                     // 如果没有被验证的帧，跳过
    LoopClosure first_verified_loop = neighbor.second.blocked_verified_ids.front(); // 取出第一个被验证的帧
    neighbor.second.blocked_verified_ids.pop_front();
    publishVerifiedLoop(first_verified_loop, neighbor.second);

    if (neighbor.second.blocked_verified_ids.empty())
      continue;                                                                   // 如果没有被验证的帧，跳过
    LoopClosure last_verified_loop = neighbor.second.blocked_verified_ids.back(); // 取出最后一个被验证的帧
    neighbor.second.blocked_verified_ids.pop_back();
    publishVerifiedLoop(last_verified_loop, neighbor.second);
  }
}

void MultiProxyPipeline::publishVerifiedLoop(LoopClosure &loop, const RobotID &neighbor)
{
  PointTypePose cur_p_wrt_ref_p;
  cur_p_wrt_ref_p.fromPose3(loop.pose.inverse());
  std_msgs::Int8MultiArray loop_msg;
  std::vector<signed char> buffer;

  auto appendFloat = [&](float f)
  {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&f), reinterpret_cast<uint8_t *>(&f) + sizeof(float));
  };

  auto appendInt32 = [&](int32_t val)
  {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&val), reinterpret_cast<uint8_t *>(&val) + sizeof(int32_t));
  };

  appendInt32(loop.c_frame_id);
  appendInt32(loop.r_frame_id);
  appendFloat(cur_p_wrt_ref_p.x);
  appendFloat(cur_p_wrt_ref_p.y);
  appendFloat(cur_p_wrt_ref_p.z);
  appendFloat(cur_p_wrt_ref_p.roll);
  appendFloat(cur_p_wrt_ref_p.pitch);
  appendFloat(cur_p_wrt_ref_p.yaw);
  appendFloat(cur_p_wrt_ref_p.intensity);

  loop_msg.data = buffer;
  neighbor.pub_verified_info.publish(loop_msg);

  uint32_t msg_size = ros::serialization::serializationLength(loop_msg);
  double time_stamp = ros::Time::now().toSec();
  _pose_bw_file << std::fixed << std::setprecision(6) << time_stamp;
  _pose_bw_file << " " << msg_size << std::endl;
}

// 设置非阻塞终端模式
void MultiProxyPipeline::setNonCanonicalMode()
{
  struct termios new_termios{};
  tcgetattr(STDIN_FILENO, &orig_termios); // 获取当前终端设置
  new_termios = orig_termios;
  new_termios.c_lflag &= ~(ICANON | ECHO);        // 禁用规范模式和回显
  tcsetattr(STDIN_FILENO, TCSANOW, &new_termios); // 立即应用新设置
}

// 恢复原始终端设置
void MultiProxyPipeline::resetTerminalMode()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

// 非阻塞检测键盘输入
bool MultiProxyPipeline::kbhit()
{
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);                        // 监听标准输入
  return select(1, &fds, nullptr, nullptr, &tv) > 0; // 返回是否有数据可读
}

void MultiProxyPipeline::monitor()
{
  if (_cur_info_list.getMaxError() > _rebuild_thres)
  {
    pcl::PointCloud<PointType>::Ptr tempCloud_c;
    _cur_info_list.rebuildMap();
    tempCloud_c = _cur_info_list.getMap();
    if (tempCloud_c != nullptr && !tempCloud_c->empty())
    {
      _pc_manager->filterMap(tempCloud_c, tempCloud_c);
      _cur_info_list.setMap(tempCloud_c);
    }
  }
}

// 描述子生成线程，针对当前机器人关键帧生成描述子并发布
void MultiProxyPipeline::publishDescriptorThread()
{
  ros::Rate rate(20);
  while (ros::ok())
  {
    rate.sleep();
    buildDescriptor();
    publishBlockedDescriptor();
  }
}

// 查找匹配成功的关键帧并针对性的发布其点云
void MultiProxyPipeline::getLoopAndPubCloudThread()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {
    rate.sleep();
    getCandidateLoop();
    publishBlockedCandidateLoop();
  }
  std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] loop detected finished" << RESET << std::endl;
}

// 几何校验线程，针对候选回环进行精确配准
void MultiProxyPipeline::geometricVerificationThread()
{
  ros::Rate rate(100);
  while (ros::ok())
  {
    rate.sleep();
    geometricVerification();
    publishBlockedVerifiedLoop();
  }
  std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] geometric verification finished" << RESET << std::endl;
}

void MultiProxyPipeline::globalOptimizationThread()
{
  ros::Rate rate(10);
  auto begin_time = clock();
  double acc_time = 0;
  int clear_count = 0;
  Utility::TicToc t_opt;
  bool dual_update = true;

  setNonCanonicalMode(); // 启动非阻塞输入模式
  while (ros::ok())
  {
    rate.sleep();
    t_opt.tic();

    if (clear_count > 0)
      clear_count--;
    // 检测键盘输入
    if (clear_count <= 0 && kbhit())
    {
      char c = getchar(); // 读取字符（非阻塞）
      if (c == 'r' || c == 'R')
      { // clear dual frames
        std::lock_guard<std::mutex> lock(_mtx_dual_frame);
        for (auto &df_list : _dual_frames)
        {
          df_list.second.clear_local_dual();
          df_list.second.clear_remote_dual();
        }
        clear_count = 50;
        std::cout << YELLOW << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> Received 'r' key, clear..." << RESET << std::endl;
        continue;
      }
      if (c == 'u' || c == 'U')
      { // close dual in graph
        if (_use_ARock)
        {
          std::cout << YELLOW << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> Received 'u' key, close dual..." << RESET << std::endl;
        }
        else
        {
          std::cout << YELLOW << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> Received 'u' key, open dual..." << RESET << std::endl;
        }
        std::lock_guard<std::mutex> lock(_mtx_dual_frame);
        for (auto &df_list : _dual_frames)
        {
          df_list.second.clear_local_dual();
          df_list.second.clear_remote_dual();
        }
        clear_count = 50;
        _use_ARock = !_use_ARock;
        continue;
      }
    }

    //      std::cout << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> OPT candidate:" << _loop_queue[robot_reference_id].size() << std::endl;
    std::vector<LoopClosure> loop_accept_queue;
    std::unordered_map<int, std::vector<LoopClosure>> loop_queue_tmp;
    {
      std::lock_guard<std::mutex> lock(_mtx_loop_queue);
      loop_queue_tmp = _loop_queue;
    }

    _multi_outlier_rejection->iPCM(loop_queue_tmp, _cur_info_list, _ref_info_list);
    loop_accept_queue = _multi_outlier_rejection->getAllLoopAcceptQueue();

    double time_pcm, time_graph_odom, time_graph_loop, time_graph_dual, time_graph_update, time_dual_update;
    time_pcm = t_opt.toc();
    t_opt.tic();
    if (_use_ARock && clear_count <= 0)
    {
      // 修改对偶因子，重建isam2
      _back_end->gtsamEgoPgoGraph();
      time_graph_odom = t_opt.toc();
      t_opt.tic();
      {
        std::lock_guard<std::mutex> lock(_mtx_dual_frame);
        if (_back_end->gtsamInterLoop(loop_accept_queue))
        {
          for (auto &df_list : _dual_frames)
          {
            df_list.second.clear_local_dual();
            df_list.second.clear_remote_dual();
          }
          _ref_info_list.resetPose();
          _cur_info_list.resetPose();
        }
        time_graph_loop = t_opt.toc();
        t_opt.tic();
        _back_end->gtsamDualFactor(_dual_frames);
      }
      time_graph_dual = t_opt.toc();
      t_opt.tic();
      _back_end->gtsamUpdate(true);
      time_graph_update = t_opt.toc();
      t_opt.tic();

      // 加入小id机器人关键帧点的一致对偶帧
      {
        auto global_map_after_graph = _back_end->getMapOptimized();
        std::lock_guard<std::mutex> lock(_mtx_dual_frame);
        for (const LoopClosure &loop_closure : loop_accept_queue)
        {
          int target_robot = loop_closure.r_frame_id / _max_keyframes;
          if (target_robot >= _robot_id_th)
            continue; // 只生成小id机器人的对偶变量，大id机器人自己的对偶在回调函数中初始化
          if (_dual_frames[target_robot].dual_frames_local.find(loop_closure.r_frame_id) != _dual_frames[target_robot].dual_frames_local.end())
            continue; // 已经存在对偶变量，跳过
          // 如果对方没反馈（丢包），并在回调函数里创建这个本地对偶，这里会一直添加
          DualFrame dual_new_ref(_robot_id_th, gtsam::Pose3());
          _dual_frames[target_robot].add_local(-loop_closure.r_frame_id, dual_new_ref);
        }

        for (const auto &df_list : _dual_frames)
        { // map 对偶会一直发不用担心丢包收不到
          for (const auto &itm : global_map_after_graph)
          {
            int robot_to = df_list.first;
            if (itm.first == df_list.second.ref_robot_id)
              continue; // 参考机器人在参考机器人中的对偶不需要发布 因为是单位阵
            if (global_map_after_graph.find(df_list.second.ref_robot_id) == global_map_after_graph.end())
              continue;
            if (itm.first > robot_to)
              continue;
            gtsam::Pose3 cur_map_wrt_ref_map = global_map_after_graph[df_list.second.ref_robot_id].toPose3();
            gtsam::Pose3 map_in_ref = cur_map_wrt_ref_map * itm.second.toPose3().inverse();
            DualFrame dual_new_map(_robot_id_th, map_in_ref);
            // 添加本地对偶变量
            _dual_frames[robot_to].add_local(itm.first * _max_keyframes, dual_new_map);
          }
        }

        for (auto &df_list : _dual_frames)
        {
          int tar_id = df_list.first;
          DualFrameList &dual_frames = df_list.second;
          std_msgs::Int8MultiArray stream_msg;
          std::vector<signed char> buffer;

          // step1: 更新本地对偶变量, 在重新生成本地对偶的时候先不更新
          if (!dual_frames.regenerate && global_map_after_graph.find(dual_frames.ref_robot_id) != global_map_after_graph.end())
            dual_frames.local_update(_cur_info_list, _ref_info_list, global_map_after_graph);

          // step2: 发布对偶
          auto appendFloat = [&](float f)
          {
            buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&f), reinterpret_cast<uint8_t *>(&f) + sizeof(float));
          };

          auto appendInt8 = [&](int8_t val)
          {
            buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&val), reinterpret_cast<uint8_t *>(&val) + sizeof(int8_t));
          };

          auto appendInt32 = [&](int32_t val)
          {
            buffer.insert(buffer.end(), reinterpret_cast<uint8_t *>(&val), reinterpret_cast<uint8_t *>(&val) + sizeof(int32_t));
          };

          int frame_num = 0;

          for (auto &dual_frame_curr : dual_frames.dual_frames_local)
          {
            if (!dual_frame_curr.second.updated)
              continue; // 只发布更新过的对偶变量
            frame_num++;
          }
          if (frame_num == 0 && !dual_frames.regenerate)
            continue;

          appendInt8((int8_t)_robot_id_th);
          appendInt8((int8_t)dual_frames.regenerate);
          dual_frames.regenerate = false; // 发布后重置
          appendInt32(frame_num);

          for (auto &dual_frame_curr : dual_frames.dual_frames_local)
          {
            int32_t frame_id = dual_frame_curr.first;
            DualFrame dual_curr = dual_frame_curr.second;
            if (!dual_curr.updated)
              continue; // 只发布更新过的对偶变量
            PointTypePose dual_curr_in_ref_vec;
            dual_curr_in_ref_vec.fromPose3(dual_curr.pose);

            appendInt32(frame_id);
            appendFloat(dual_curr_in_ref_vec.x);
            appendFloat(dual_curr_in_ref_vec.y);
            appendFloat(dual_curr_in_ref_vec.z);
            appendFloat(dual_curr_in_ref_vec.roll);
            appendFloat(dual_curr_in_ref_vec.pitch);
            appendFloat(dual_curr_in_ref_vec.yaw);
            if (frame_id >= 0)
              dual_frame_curr.second.updated = false; // 负帧号一直更新，直到在回调函数里收到反馈清除
          }

          stream_msg.data = buffer;

          if (!_neighbor_id_list.at(tar_id).isAlive)
            continue;
          _neighbor_id_list.at(tar_id).pub_dual_info.publish(stream_msg);

          uint32_t msg_size = ros::serialization::serializationLength(stream_msg);
          double time_stamp = ros::Time::now().toSec();
          _dual_bw_file << std::fixed << std::setprecision(6) << time_stamp;
          _dual_bw_file << " " << msg_size << std::endl;
        }
      }

      time_dual_update = t_opt.toc();
      t_opt.tic();
      //        std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> odom factor time : " << time_graph_odom << " ms, " << "loop factor time: " << time_graph_loop << " ms, " << "Dual factor time: " << time_graph_dual << " ms, " << "Update time: " << time_graph_update << " ms" << RESET << std::endl;
    }
    else if (clear_count <= 0)
    {
      std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] <GlobalOPT> Update without Dual! : " << RESET << std::endl;
      _back_end->gtsamEgoPgoGraph();
      _back_end->gtsamInterLoop(loop_accept_queue);
      _back_end->gtsamUpdate(false);
    }
  }

  resetTerminalMode(); // 退出前恢复终端设置
  std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "]  <GlobalOPT> Global optimization finished" << RESET << std::endl;
  std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "]  <GlobalOPT> Backend odom std: " << _odom_std
            << ", intra_loop: " << _intra_loop_std << ", inter_loop: " << _inter_loop_std << ", dual std: " << _dual_std << RESET << std::endl;
}

void MultiProxyPipeline::publishGlobalMapThread()
{
  ros::Rate rate(1);
  ros::Time last_time_lidar_stamp = _time_lidar_stamp;
  ros::Time last_current_stamp = ros::Time::now();
  while (ros::ok())
  {
    if (last_time_lidar_stamp == _time_lidar_stamp && _use_bag_time)
    {
      _time_lidar_stamp.fromSec(last_time_lidar_stamp.toSec() + ros::Time::now().toSec() - last_current_stamp.toSec());
    }
    last_time_lidar_stamp = _time_lidar_stamp;
    last_current_stamp = ros::Time::now();

    rate.sleep();
    int ref_robot_id = _back_end->getMinKeyOptimized();
    auto current2global = _back_end->getPoseWrtRefRobotIdx(ref_robot_id);
    _visulization.publishMapGlobalCurrent(_time_lidar_stamp, ref_robot_id, current2global);
    std::vector<LoopClosure> accepted_loops = _multi_outlier_rejection->getAllLoopAcceptQueue();
    _visulization.visualizeLoopClosure(_time_lidar_stamp, accepted_loops);
    _visulization.visualizeKeyFrame(_time_lidar_stamp);
    std::unordered_map<int, DualFrameList> dual_lists;
    {
      std::lock_guard<std::mutex> lock(_mtx_dual_frame);
      dual_lists = _dual_frames;
    }
    _visulization.visualizeDualFrames(_time_lidar_stamp, dual_lists);
    //      Utility::TicToc t_mapping;
    monitor();
    //      std::cout << CYAN <<  "[SubMapFusion "  + _robot_name + "] Mapping Time: " << t_mapping.toc() << RESET << std::endl;
  }
  //    std::cout << GREEN <<  "[SubMapFusion "  + _robot_name + "] globalMapping finished" << RESET << std::endl;
}

void MultiProxyPipeline::savePoseAndMap()
{
  ros::Rate rate(100);
  while (ros::ok())
  {
    rate.sleep();
    if (!_save_pose_and_map)
      continue;

    ROS_INFO("Saving trajectories to files: %s ... 10s", _log_folder.c_str());
    _back_end->print_net = true;

    {
      std::lock_guard<std::mutex> lock(_mtx_dual_frame);
      for (auto &df_list : _dual_frames)
      {
        df_list.second.clear_local_dual();
        df_list.second.clear_remote_dual();
      }
    }

    // sleep for 10 seconds to ensure all data is processed
    ros::Duration(10.0).sleep();

    std::vector<PointTypePose> traj;
    _cur_info_list.getOptimizedTraj(_robot_id_th, traj);
    std::ofstream tum_file;
    tum_file.open(_log_folder + "/robot_" + std::to_string(_robot_id_th) + ".tum");
    tum_file.setf(ios::fixed);
    tum_file.precision(10);

    int ref_robot_id = _back_end->getMinKeyOptimized();
    PointTypePose ref_pose = _back_end->getPoseWrtRefRobotIdx(ref_robot_id);
    for (auto &pose : traj)
    {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      SEtoQuatT((ref_pose.toPose3() * pose.toPose3()).matrix(), q, t);
      tum_file << pose.time << " " << t.x() << " " << t.y() << " " << t.z() << " "
               << q.x() << " " << q.y() << " " << q.z() << " " << q.w() /* << " "  << pose.intensity*/ << std::endl;
    }
    tum_file.close();

    pcl::PointCloud<PointType>::Ptr tempCloud_c;
    tempCloud_c = _cur_info_list.rebuildMap(_save_map_resolution);
    tempCloud_c = KeyFrame::transformPointCloud(tempCloud_c, ref_pose);

    pcl::io::savePCDFileASCII<PointType>(_log_folder + "/robot_" + std::to_string(_robot_id_th) + "_pcd_wrt_global.pcd", *tempCloud_c);
    _save_pose_and_map = false;
  }
  std::cout << GREEN << "[SubMapFusion robot_" + std::to_string(_robot_id_th) + "] geometric verification finished" << RESET << std::endl;
}
