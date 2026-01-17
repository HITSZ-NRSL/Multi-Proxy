//
// Created by jcwang on 24-10-19.
//

#include "KeyFrame.h"


#define DESCRIPTOR_VISUALIZATION 0



KeyFrame::KeyFrame(std::string robot_name_init, int id, int frame){
  time = 0;
  robot_name = std::move(robot_name_init);
  robot_id = id;
  frame_id = frame;
  pair_robot_id = -1;
  get_pose_decentralized = false;
  has_optimized = false;
  is_degenerate = false;
  is_corrected = false;
  is_key_frame = false;
  is_checkdes = false;
  cloud.reset(new pcl::PointCloud<PointType>());
  std_corner.reset(new pcl::PointCloud<PointTypeNormal>());
  noiseMatrix = gtsam::I_6x6;
}

KeyFrame::KeyFrame(const KeyFrame& contextWithPose_in){
  robot_name = contextWithPose_in.robot_name;
  robot_id = contextWithPose_in.robot_id;
  frame_id = contextWithPose_in.frame_id;
  pair_robot_id = contextWithPose_in.pair_robot_id;
  get_pose_decentralized = contextWithPose_in.get_pose_decentralized;
  has_optimized = contextWithPose_in.has_optimized;
  is_degenerate = contextWithPose_in.is_degenerate;
  is_corrected = contextWithPose_in.is_corrected;
  is_key_frame = contextWithPose_in.is_key_frame;
  is_checkdes = contextWithPose_in.is_checkdes;
  cloud_header = contextWithPose_in.cloud_header;
  time = contextWithPose_in.time;
  decentralized_pose_in_cur_map = contextWithPose_in.decentralized_pose_in_cur_map;
  pose_init = contextWithPose_in.pose_init;
  distributed_pose_in_cur_map = contextWithPose_in.distributed_pose_in_cur_map;
  //
  degenerateData = contextWithPose_in.degenerateData;
  cloud = contextWithPose_in.cloud;
  std_corner = contextWithPose_in.std_corner;
  stdesc = contextWithPose_in.stdesc;
  noiseMatrix = contextWithPose_in.noiseMatrix;
  uwb_map = contextWithPose_in.uwb_map;
}

KeyFrame::KeyFrame(const CandidateInfo& candidate_info_input){
  robot_name = candidate_info_input.robotName;
  robot_id = candidate_info_input.robotID;
  frame_id = candidate_info_input.candidateFrameId;
  pair_robot_id = -1;
  get_pose_decentralized = false;
  has_optimized = false;
  is_degenerate = false;
  is_corrected = false;
  is_key_frame = true;
  is_checkdes = false;

  decentralized_pose_in_cur_map.x = candidate_info_input.pose.x;
  decentralized_pose_in_cur_map.y = candidate_info_input.pose.y;
  decentralized_pose_in_cur_map.z = candidate_info_input.pose.z;
  decentralized_pose_in_cur_map.roll  = candidate_info_input.pose.roll;
  decentralized_pose_in_cur_map.pitch = candidate_info_input.pose.pitch;
  decentralized_pose_in_cur_map.yaw   = candidate_info_input.pose.yaw;
//        std::cout << "sc_yaw:" << sc_info.decentralized_pose_in_cur_map.yaw << std::endl;
  decentralized_pose_in_cur_map.intensity = candidate_info_input.pose.intensity;
  pose_init = decentralized_pose_in_cur_map;
  cloud.reset(new pcl::PointCloud<PointType>());
  std_corner.reset(new pcl::PointCloud<PointTypeNormal>());
  if (!candidate_info_input.scanCloud.data.empty())
    pcl::fromROSMsg(candidate_info_input.scanCloud, *cloud);
}

KeyFrame::KeyFrame(const std_msgs::Int8MultiArray& descriptor_msg){
  std::vector<int8_t, std::allocator<int8_t>> data = descriptor_msg.data;
  size_t offset = 0;

  auto readString = [&](std::string& outStr) {
    uint32_t len = *reinterpret_cast<const uint32_t*>(&data[offset]);
    offset += sizeof(uint32_t);
    outStr = std::string(data.begin() + offset, data.begin() + offset + len);
    offset += len;
  };

  auto readFloat = [&]() {
    float f = *reinterpret_cast<const float*>(&data[offset]);
    offset += sizeof(float);
    return f;
  };

  auto readUint8 = [&]() {
    uint8_t val = *reinterpret_cast<const uint8_t*>(&data[offset]);
    offset += sizeof(uint8_t);
    return val;
  };

  auto readUint32 = [&]() {
    uint32_t val = *reinterpret_cast<const uint32_t*>(&data[offset]);
    offset += sizeof(uint32_t);
    return val;
  };

  auto readInt32 = [&]() {
    int32_t val = *reinterpret_cast<const int32_t*>(&data[offset]);
    offset += sizeof(int32_t);
    return val;
  };
  // 1. Strings
  readString(robot_name);
  robot_id = readInt32();

  // 2. Frame id
  frame_id = readInt32();

  // 3. Pose
  decentralized_pose_in_cur_map.x = readFloat();
  decentralized_pose_in_cur_map.y = readFloat();
  decentralized_pose_in_cur_map.z = readFloat();
  decentralized_pose_in_cur_map.roll = readFloat();
  decentralized_pose_in_cur_map.pitch = readFloat();
  decentralized_pose_in_cur_map.yaw = readFloat();
  decentralized_pose_in_cur_map.intensity = readFloat();
  pose_init = decentralized_pose_in_cur_map;

  // 4. PointCloud2
  sensor_msgs::PointCloud2 ros_cloud;
  uint32_t cloud_size = readUint32();
  ros_cloud.data.resize(cloud_size);
  ros_cloud.height = readUint32();
  ros_cloud.width = readUint32();
  ros_cloud.point_step = readUint32();
  ros_cloud.row_step = readUint32();
  ros_cloud.is_dense = readUint8();

  uint32_t num_fields = readUint32();
  ros_cloud.fields.resize(num_fields);
  for (uint32_t i = 0; i < num_fields; ++i) {
    readString(ros_cloud.fields[i].name);
    ros_cloud.fields[i].offset = readUint32();
    ros_cloud.fields[i].datatype = readUint8();
    ros_cloud.fields[i].count = readUint32();
  }
  std::copy(data.begin() + offset, data.begin() + offset + cloud_size, ros_cloud.data.begin());
  offset += cloud_size;

  cloud.reset(new pcl::PointCloud<PointType>());
  std_corner.reset(new pcl::PointCloud<PointTypeNormal>());

  if (!ros_cloud.data.empty())
    pcl::fromROSMsg(ros_cloud, *std_corner);

  pair_robot_id = -1;
  get_pose_decentralized = false;
  has_optimized = false;
  is_degenerate = false;
  is_corrected = false;
  is_key_frame = true;
  is_checkdes = false;
}

KeyFrame::~KeyFrame() = default;

void KeyFrame::publishDescriptorStream(const ros::Publisher &thisPub, std::ofstream &log_file) {
  std_msgs::Int8MultiArray descriptor_msg;
  std::vector<signed char> buffer;

  auto appendString = [&](const std::string& str) {
    uint32_t len = str.length();
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&len), reinterpret_cast<uint8_t*>(&len) + sizeof(uint32_t));
    buffer.insert(buffer.end(), str.begin(), str.end());
  };

  auto appendFloat = [&](float f) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&f), reinterpret_cast<uint8_t*>(&f) + sizeof(float));
  };

  auto appendUint8 = [&](uint8_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(uint8_t));
  };

  auto appendUint32 = [&](uint32_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(uint32_t));
  };

  auto appendInt32 = [&](int32_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(int32_t));
  };

  // 1. Strings
  appendString(robot_name);
  appendInt32(robot_id);

  // 2. Frame id Int
  appendInt32(frame_id);

  // 3. Pose floats
  appendFloat(pose_init.x);
  appendFloat(pose_init.y);
  appendFloat(pose_init.z);
  appendFloat(pose_init.roll);
  appendFloat(pose_init.pitch);
  appendFloat(pose_init.yaw);
  appendFloat(pose_init.intensity);

  // 4. Point cloud size
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*std_corner, ros_cloud);
  uint32_t cloud_size = ros_cloud.data.size();
  appendUint32(cloud_size);
  appendUint32(ros_cloud.height);
  appendUint32(ros_cloud.width);
  appendUint32(ros_cloud.point_step);
  appendUint32(ros_cloud.row_step);
  appendUint8(static_cast<uint8_t>(ros_cloud.is_dense));

  appendUint32(ros_cloud.fields.size());
  for (const auto& f : ros_cloud.fields) {
    appendString(f.name);
    appendUint32(f.offset);
    appendUint8(f.datatype);
    appendUint32(f.count);
  }
  buffer.insert(buffer.end(), ros_cloud.data.begin(), ros_cloud.data.end());

  // Final assignment
  descriptor_msg.data = buffer;
  thisPub.publish(descriptor_msg);

  uint32_t msg_size = ros::serialization::serializationLength(descriptor_msg);
  double time_stamp = ros::Time::now().toSec();
  log_file << std::fixed << std::setprecision(6) << time_stamp;
  log_file << " " << msg_size << std::endl;
}

void KeyFrame::publishCandidateStream(const ros::Publisher &thisPub, std::ofstream &log_file,
                                      pcl::PointCloud<PointType>::Ptr &cloud_in,
                                      int loop_frame_id, PointTypePose trans) {
  std_msgs::Int8MultiArray candidate_msg;
  std::vector<signed char> buffer;
  auto appendString = [&](const std::string& str) {
    uint32_t len = str.length();
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&len), reinterpret_cast<uint8_t*>(&len) + sizeof(uint32_t));
    buffer.insert(buffer.end(), str.begin(), str.end());
  };

  auto appendFloat = [&](float f) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&f), reinterpret_cast<uint8_t*>(&f) + sizeof(float));
  };

  auto appendUint8 = [&](uint8_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(uint8_t));
  };

  auto appendUint32 = [&](uint32_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(uint32_t));
  };

  auto appendInt32 = [&](int32_t val) {
    buffer.insert(buffer.end(), reinterpret_cast<uint8_t*>(&val), reinterpret_cast<uint8_t*>(&val) + sizeof(int32_t));
  };

  // 1. Strings
  appendString(robot_name);
  appendInt32(robot_id);

  // 2. Candidate Frame id Int
  appendInt32(frame_id);

  // 3. Pose floats
  appendFloat(pose_init.x);
  appendFloat(pose_init.y);
  appendFloat(pose_init.z);
  appendFloat(pose_init.roll);
  appendFloat(pose_init.pitch);
  appendFloat(pose_init.yaw);
  appendFloat(pose_init.intensity);

  // 4. Loop Frame id Int
  appendInt32(loop_frame_id);

  // 5. Trans floats
  appendFloat(trans.x);
  appendFloat(trans.y);
  appendFloat(trans.z);
  appendFloat(trans.roll);
  appendFloat(trans.pitch);
  appendFloat(trans.yaw);

  // 6. Point cloud size
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud_in, ros_cloud);
  uint32_t cloud_size = ros_cloud.data.size();
  appendUint32(cloud_size);
  appendUint32(ros_cloud.height);
  appendUint32(ros_cloud.width);
  appendUint32(ros_cloud.point_step);
  appendUint32(ros_cloud.row_step);
  appendUint8(static_cast<uint8_t>(ros_cloud.is_dense));

  appendUint32(ros_cloud.fields.size());
  for (const auto& f : ros_cloud.fields) {
    appendString(f.name);
    appendUint32(f.offset);
    appendUint8(f.datatype);
    appendUint32(f.count);
  }
  buffer.insert(buffer.end(), ros_cloud.data.begin(), ros_cloud.data.end());

  // Final assignment
  candidate_msg.data = buffer;
  thisPub.publish(candidate_msg);

  uint32_t msg_size = ros::serialization::serializationLength(candidate_msg);
  double time_stamp = ros::Time::now().toSec();
  log_file << std::fixed << std::setprecision(6) << time_stamp;
  log_file << " " << msg_size << std::endl;
}

pcl::PointCloud<PointType>::Ptr KeyFrame::transformPointCloud(const pcl::PointCloud<PointType>::Ptr& cloudIn, PointTypePose &transformIn)
{
  int cloud_width = cloudIn->width;
  int cloud_height = cloudIn->height;
  int cloudSize = cloudIn->size();
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>(cloud_width, cloud_height));

  Eigen::Affine3f transCur = transformIn.toAffine();
  if (cloudSize != cloud_width*cloud_height) std::cout << RED << "WTF????" << RESET << std::endl;
#pragma omp parallel for num_threads(4)
  for (int i = 0; i < cloudSize; ++i)
  {
//            std::cout << "current id: " << omp_get_thread_num() << endl;
    PointType pointFrom = cloudIn->points[i];
    cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
    cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
    cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
}

pcl::PointCloud<PointType>::Ptr KeyFrame::transformPointCloud(PointTypePose &transformIn)
{
  int cloud_width = cloud->width;
  int cloud_height = cloud->height;
  int cloudSize = cloud->size();
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>(cloud_width, cloud_height));

  Eigen::Affine3f transCur = transformIn.toAffine();
  if (cloudSize != cloud_width*cloud_height) std::cout << RED << "WTF????" << RESET << std::endl;
#pragma omp parallel for num_threads(4)
  for (int i = 0; i < cloudSize; ++i)
  {
//            std::cout << "current id: " << omp_get_thread_num() << endl;
    PointType pointFrom = cloud->points[i];
    cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
    cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
    cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
}

bool KeyFrame::checkRefFrameId(int frame_id_input) {
  if (candidate_inter_loop.find(frame_id_input) != candidate_inter_loop.end()) return true;
  return false;
}
void KeyFrame::setRefFrameId(int frame_id_input, PointTypePose pose_in_ref_frame) {
  candidate_inter_loop[frame_id_input] = pose_in_ref_frame;
}

void KeyFrame::publishSTD(const ros::Publisher &std_publisher) {
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = robot_name + "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = robot_name + "/map";
  m_line.id = 0;
  int max_pub_cnt = 1;
  for (auto std : stdesc) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    m_line.color.r = 138.0 / 255;
    m_line.color.g = 226.0 / 255;
    m_line.color.b = 52.0 / 255;
    geometry_msgs::Point p;
    p.x = std.vertex_A_[0];
    p.y = std.vertex_A_[1];
    p.z = std.vertex_A_[2];
    Eigen::Vector3d t_p;
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = std.vertex_B_[0];
    p.y = std.vertex_B_[1];
    p.z = std.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = std.vertex_C_[0];
    p.y = std.vertex_C_[1];
    p.z = std.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = std.vertex_B_[0];
    p.y = std.vertex_B_[1];
    p.z = std.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = std.vertex_C_[0];
    p.y = std.vertex_C_[1];
    p.z = std.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = std.vertex_A_[0];
    p.y = std.vertex_A_[1];
    p.z = std.vertex_A_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();

  }

  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

void KeyFrame::clearPointCloud() {
  if (!cloud->empty()) {
    cloud->clear();
  }
}

double KeyFrame::getDataSize() {
  int data_size = 0;
  data_size += sizeof(KeyFrame);
  data_size += cloud->size() * sizeof(PointType);
  data_size += std_corner->size() * sizeof(PointTypeNormal);
  data_size += stdesc.size() * sizeof(STDesc);
  data_size += noiseMatrix.rows() * noiseMatrix.cols() * sizeof(float);
  data_size += posConstraintMatrix.rows() * posConstraintMatrix.cols() * sizeof(float);
  data_size += oriConstraintMatrix.rows() * oriConstraintMatrix.cols() * sizeof(float);
  data_size += candidate_inter_loop.size() * (sizeof(int) + sizeof(PointTypePose));
  return (double)data_size / 1024.0;
}
