//
// Created by jcwang on 24-10-19.
//

#include "KeyFrameList.h"

KeyFrameList::KeyFrameList() {
  lidarmap_current.reset(new pcl::PointCloud<PointType>());
}

void KeyFrameList::addKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
  if (nullptr != getKeyFrame(keyframe->frame_id)) return;
  {
    std::lock_guard<std::mutex> lock(mtx_keyframe_list);
    keyframe_list.push_back(keyframe);
  }
}

std::shared_ptr<KeyFrame> KeyFrameList::at(int index){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  if (index >= keyframe_list.size()) std::cout << RED << "KeyFrameList::at() index out of range:" << index << "/" << keyframe_list.size() << RESET << std::endl;
  return keyframe_list.at(index);
}

std::shared_ptr<KeyFrame> KeyFrameList::back() {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  return keyframe_list.back();
}

int KeyFrameList::size(){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  return (int)keyframe_list.size();
}

bool KeyFrameList::buildMapNB(int frame_id, const pcl::PointCloud<PointType>::Ptr& pointCloud, int frame_num){

  int num = (int)keyframe_list.size();
  int frame_index = getKeyFrameId(frame_id);
  PointTypePose frame_pose = keyframe_list[frame_index]->getPoseInit();
//  std::cout << "[SubMapFusion] <MapNB> frame_id: " << frame_id << " frame_index: " << frame_index << " num: " << num << std::endl;
  if (frame_index >= 0 && frame_index < num){
    std::vector<std::shared_ptr<KeyFrame>> keyframe_list_copy;
    {
      std::lock_guard<std::mutex> lock(mtx_keyframe_list);
      for (int i = std::max(0, frame_index - frame_num/2); i < std::min(num, frame_index + frame_num/2); ++i) {
        keyframe_list_copy.push_back(keyframe_list[i]);
      }
    }
    for (auto & keyframe : keyframe_list_copy) {
      PointTypePose pose_in_frame = frame_pose.poseBetween(keyframe->getPoseInit());
      *pointCloud += *(keyframe->transformPointCloud(pose_in_frame));
    }
    return true;
  }
  else return false;
}

bool KeyFrameList::buildMapWrtLocalMap(int frame_id, const pcl::PointCloud<PointType>::Ptr& pointCloud, int frame_num){

  int num = (int)keyframe_list.size();
  int frame_index = getKeyFrameId(frame_id);
  if (frame_index >= 0 && frame_index < num){
    std::vector<std::shared_ptr<KeyFrame>> keyframe_list_copy;
    {
      std::lock_guard<std::mutex> lock(mtx_keyframe_list);
      for (int i = std::max(0, frame_index - (frame_num + 1) / 2); i < std::min(num, frame_index + (frame_num + 1) / 2); ++i) {
        keyframe_list_copy.push_back(keyframe_list[i]);
      }
    }
    for (auto & keyframe : keyframe_list_copy) {

      PointTypePose pose_wrt_local_map = keyframe->getPoseInit();

      pcl::PointCloud<PointType>::Ptr transformed_cloud = keyframe->transformPointCloud(pose_wrt_local_map);

      *pointCloud += *transformed_cloud;
    }
    return true;
  }
  else return false;
}


int KeyFrameList::getKeyFrameId(int frame_id){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (int i = 0; i < keyframe_list.size(); i++) {
    if (keyframe_list[i]->frame_id == frame_id)
      return i;
  }
  return -1;
}


std::shared_ptr<KeyFrame> KeyFrameList::getKeyFrame(int frame_id){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & i : keyframe_list) {
    if (i->frame_id == frame_id)
      return i;
  }
  return nullptr;
}

bool KeyFrameList::empty() {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  if (keyframe_list.empty())
    return true;
  else
    return false;
}

void KeyFrameList::setMap(const pcl::PointCloud<PointType>::Ptr &pointCloud) {
  std::lock_guard<std::mutex> lock(mtx_map);
  lidarmap_current->clear();
  pcl::copyPointCloud(*pointCloud, *lidarmap_current);
}

void KeyFrameList::buildMap(const pcl::PointCloud<PointType>::Ptr &pointCloud) {
  std::lock_guard<std::mutex> lock(mtx_map);
  *lidarmap_current += *pointCloud;
}

void KeyFrameList::rebuildMap() {
  std::lock_guard<std::mutex> lock(mtx_map);
  std::vector<std::shared_ptr<KeyFrame>> keyframe_list_copy;
  {
    std::lock_guard<std::mutex> lock_frame(mtx_keyframe_list);
    keyframe_list_copy = keyframe_list;
  }
  lidarmap_current->clear();
  for (auto & key_frame : keyframe_list_copy) {
    if (!key_frame->is_key_frame) continue;
//    if (!key_frame->has_optimized) continue;
    if (key_frame->cloud->empty()) continue;
    pcl::PointCloud<PointType>::Ptr tmp_cloud;
    tmp_cloud = key_frame->transformPointCloud(key_frame->distributed_pose_in_cur_map);
    for (auto &point : tmp_cloud->points) {
      point.intensity = (float)key_frame->time;
      point.data[3] = (float)key_frame->time;
    }

    *lidarmap_current += *tmp_cloud;
  }
}


struct VoxelKey {
  int x, y, z;
  bool operator==(VoxelKey const& o) const {
    return x==o.x && y==o.y && z==o.z;
  }
};


namespace std {
  template<> struct hash<VoxelKey> {
    size_t operator()(VoxelKey const& k) const noexcept {

      size_t hx = std::hash<int>()(k.x);
      size_t hy = std::hash<int>()(k.y);
      size_t hz = std::hash<int>()(k.z);
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };
} // namespace std

pcl::PointCloud<PointType>::Ptr KeyFrameList::rebuildMap(double voxel_size) {
  std::lock_guard<std::mutex> lock(mtx_map);
  std::vector<std::shared_ptr<KeyFrame>> keyframe_list_copy;
  {
    std::lock_guard<std::mutex> lock_frame(mtx_keyframe_list);
    keyframe_list_copy = keyframe_list;
  }


  std::unordered_map<VoxelKey, PointType> voxel_map;
  voxel_map.reserve(keyframe_list_copy.size() * 1000);


  for (auto & key_frame : keyframe_list_copy) {
    if (!key_frame->is_key_frame)   continue;
//    if (!key_frame->has_optimized)  continue;
    if (key_frame->cloud->empty())  continue;


    pcl::PointCloud<PointType>::Ptr tmp_cloud =
        key_frame->transformPointCloud(key_frame->distributed_pose_in_cur_map);


    for (auto &pt : tmp_cloud->points) {

      int ix = static_cast<int>(std::floor(pt.x / voxel_size));
      int iy = static_cast<int>(std::floor(pt.y / voxel_size));
      int iz = static_cast<int>(std::floor(pt.z / voxel_size));
      VoxelKey key{ix, iy, iz};


      double cx = (ix + 0.5) * voxel_size;
      double cy = (iy + 0.5) * voxel_size;
      double cz = (iz + 0.5) * voxel_size;


      double dist2 =
          (pt.x - cx)*(pt.x - cx) +
          (pt.y - cy)*(pt.y - cy) +
          (pt.z - cz)*(pt.z - cz);

      auto it = voxel_map.find(key);
      if (it == voxel_map.end()) {

        pt.intensity = static_cast<float>(key_frame->time);
        pt.data[3]    = static_cast<float>(key_frame->time);
        voxel_map[key] = pt;
      } else {

        const PointType &old_pt = it->second;
        double old_dist2 =
            (old_pt.x - cx)*(old_pt.x - cx) +
            (old_pt.y - cy)*(old_pt.y - cy) +
            (old_pt.z - cz)*(old_pt.z - cz);
        if (dist2 < old_dist2) {
          PointType new_pt = pt;
          new_pt.intensity = static_cast<float>(key_frame->time);
          new_pt.data[3]    = static_cast<float>(key_frame->time);
          it->second = new_pt;
        }
      }
    }
  }


  pcl::PointCloud<PointType>::Ptr aggregated(new pcl::PointCloud<PointType>(1, voxel_map.size()));
  for (auto &kv : voxel_map) {
    aggregated->push_back(kv.second);
  }
  return aggregated;
}

pcl::PointCloud<PointType>::Ptr KeyFrameList::getMap() {
  std::lock_guard<std::mutex> lock(mtx_map);
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>(*lidarmap_current));
  return cloud;
}

void KeyFrameList::resetFlag(){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & i : keyframe_list) {
    i->has_optimized = false;
    i->is_corrected = false;
  }
}

void KeyFrameList::resetPose(){
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & i : keyframe_list) {
    i->distributed_pose_in_cur_map = i->decentralized_pose_in_cur_map;
  }
}

double KeyFrameList::getListSize() {
  double data_size = 0;
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & item : keyframe_list) {
    data_size += item->getDataSize();
  }
  return data_size;
}
void KeyFrameList::clearPointCloud() {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  int size = keyframe_list.size();
  if (size < 15) return;
  for (int i = 0; i < size - 15; i++) {
    if (keyframe_list[i]->is_key_frame) continue;
    keyframe_list[i]->clearPointCloud();
  }
}

float KeyFrameList::getMaxError() {
  float max_error = 0;
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & item : keyframe_list) {
    if (!item->is_key_frame) continue;
    if (!item->has_optimized) continue;
    float dist = item->getPoseInit().distance(item->distributed_pose_in_cur_map);
    max_error = std::max(max_error, dist);
  }
  return max_error;
}

bool KeyFrameList::getOptimizedTraj(int robot_id, std::vector<PointTypePose> &poses) {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & item : keyframe_list) {
    if (item->robot_id == robot_id) {
      if (!item->is_key_frame) continue;
      if (!item->has_optimized) continue;
      poses.push_back(item->getPoseOptimized());
    }
  }
  if (poses.empty()) return false;
  return true;
}

bool KeyFrameList::getInitialTraj(int robot_id, std::vector<PointTypePose> &poses) {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & item : keyframe_list) {
    if (item->frame_id == robot_id) {
      poses.push_back(item->getPoseInit());
    }
  }
  if (poses.empty()) return false;
  return true;
}

bool KeyFrameList::getTrajInCurrentOdom(int robot_id, std::vector<PointTypePose> &poses) {
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  for (auto & item : keyframe_list) {
    if (item->frame_id == robot_id) {
      poses.push_back(item->distributed_pose_in_cur_map);
    }
  }
  if (poses.empty()) return false;
  return true;
}

int KeyFrameList::getNearestFrameIdByTime(double time, int robot_id) {
  int frame_id = -1;
  double frame_time = 0.0;
   std::lock_guard<std::mutex> lock(mtx_keyframe_list);
   for (int i = keyframe_list.size() - 1; i >= 0; i--) {
     auto item = keyframe_list[i];
     if (item->robot_id != robot_id) continue;
     if (item->time < time) break;
     frame_id = item->frame_id;
     frame_time = item->time;
   }

  return frame_id;
}

PointTypePose KeyFrameList::getInterpolatePoseByTime(double time, int robot_id, int frame_id) {
  double frame_time = 0.0;
  std::lock_guard<std::mutex> lock(mtx_keyframe_list);
  PointTypePose pose0;
  PointTypePose pose1;
  PointTypePose pose;
  int idx0 = getKeyFrameId(frame_id);
  int idx1 = getKeyFrameId(frame_id + 1);
  std::shared_ptr<KeyFrame> keyframe0 = keyframe_list[idx0];
  std::shared_ptr<KeyFrame> keyframe1 = keyframe_list[idx1];
  pose0 = keyframe0->distributed_pose_in_cur_map;
  pose1 = keyframe1->distributed_pose_in_cur_map;
  double time0 = keyframe0->time;
  double time1 = keyframe1->time;

  if (time < time1 || time > time0) {
    ROS_ERROR("<get time> Time out of range!");
  }

  double ratio = (time - time0) / (time1 - time0);


  Eigen::Quaternionf q0(pose0.toAffine().rotation());
  Eigen::Quaternionf q1(pose1.toAffine().rotation());


  Eigen::Vector3f t0 = pose0.toAffine().translation();
  Eigen::Vector3f t1 = pose1.toAffine().translation();


  Eigen::Quaternionf q = q0.slerp(ratio, q1);


  Eigen::Vector3f t = (1 - ratio) * t0 + ratio * t1;


  Eigen::Affine3f A = Eigen::Affine3f::Identity();
  A.linear() = q.toRotationMatrix();
  A.translation() = t;
  pose.fromAffine(A);
  return pose;
}

//void KeyFrameList::saveG2oGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom) {
//  std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
//  g2o_outfile.precision(15);
//  // g2o_outfile << std::fixed << std::setprecision(9);
//
//  for (int i = 0; i < keyframePosesOdom.size(); i++) {
//    nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
//    double time = odometry.header.stamp.toSec();
//
//    g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
//    g2o_outfile << odometry.pose.pose.position.x << " ";
//    g2o_outfile << odometry.pose.pose.position.y << " ";
//    g2o_outfile << odometry.pose.pose.position.z << " ";
//    g2o_outfile << odometry.pose.pose.orientation.x << " ";
//    g2o_outfile << odometry.pose.pose.orientation.y << " ";
//    g2o_outfile << odometry.pose.pose.orientation.z << " ";
//    g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
//  }
//  //  LOG(INFO) << "WRITE G2O VERTICES: " << keyframePosesOdom.size();
//  g2o_outfile.close();
//}