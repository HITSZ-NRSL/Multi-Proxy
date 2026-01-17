//
// Created by jcwang on 25-3-11.
//

#include "LoopDetector.h"

LoopDetector::LoopDetector(DetectorConfig config){
  std::cout << "LoopDetector created" << std::endl;
  {
    std_manager_ptr = new STDescManager(config.config_setting);
  }
  std::cout<<config.config_setting.skip_near_num_<<std::endl;

}


bool LoopDetector::makeDescriptorsSTD(pcl::PointCloud<PointType>::Ptr sub_map_temp, std::shared_ptr<KeyFrame> &keyFrame){
  std::vector<STDesc> stds_desc;
  if (!sub_map_temp->empty()){

    std_manager_ptr->GenerateSTDescs(sub_map_temp, keyFrame->std_corner);
    return true;
  }
  return false;
}

// Todo
void LoopDetector::addHistoryDescriptorSTD(std::shared_ptr<KeyFrame> &keyFrame){
  if (keyFrame->stdesc.empty() && !keyFrame->std_corner->empty()) {
    // step4, generate stable triangle descriptors
    keyFrame->stdesc.clear();
    std_manager_ptr->build_stdesc(keyFrame->std_corner, keyFrame->stdesc, keyFrame->frame_id);
    if (keyFrame->stdesc.empty()) {
      keyFrame->std_corner->clear();
      return;
    }
  }

  std_manager_ptr->AddSTDescs(keyFrame->stdesc);
}

// search candidate frame std method
bool LoopDetector::searchCandidateFrameSTD(std::shared_ptr<KeyFrame> &keyFrame, PointTypePose &cur_map_in_ref_map_init, int &current_idx,
                                          std::vector<std::pair<STDesc, STDesc>> &loop_std_pair){
  std::pair<int, double> search_result(-1, 0);
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
  loop_transform.first << 0, 0, 0;
  loop_transform.second = Eigen::Matrix3d::Identity();

  if (keyFrame->stdesc.empty() && !keyFrame->std_corner->empty()) {
    // step4, generate stable triangle descriptors
    keyFrame->stdesc.clear();
    std_manager_ptr->build_stdesc(keyFrame->std_corner, keyFrame->stdesc, keyFrame->frame_id);
    if (keyFrame->stdesc.empty()) {
      keyFrame->std_corner->clear();
      return false;
    }
    // std::cout << "[Description] stds size:" << stds_vec.size() << std::endl;
  }

  std_manager_ptr->SearchLoop(keyFrame->stdesc, keyFrame->frame_id, search_result, loop_transform,
                              loop_std_pair);

  if (search_result.first > 0) {
    current_idx = search_result.first;
    Eigen::Affine3f sc_initial = Eigen::Affine3f::Identity();
    sc_initial.translation() = loop_transform.first.cast<float>();
    sc_initial.linear() = loop_transform.second.cast<float>();

    cur_map_in_ref_map_init.fromAffine(sc_initial.inverse());
    return true;
  }
  return false;
}

void LoopDetector::pubSTDPairs(const std::string& ref_robot_name, std::vector<std::pair<STDesc, STDesc>> &loop_std_pair,
                               PointTypePose& cur_wrt_ref_map, const ros::Publisher &stdpair_pub) {
  tf::StampedTransform transform = cur_wrt_ref_map.toStampedTransform(ref_robot_name+ "/map", "robot_" + std::to_string(robot_id) + "/map");
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;

  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.1;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = ref_robot_name + "/map";
  m_line.id = 0;
  int max_pub_cnt = 1;
  for (auto var : loop_std_pair) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 0.5;
    m_line.points.clear();
    m_line.color.r = 250.0 / 255;
    m_line.color.g = 24.0 / 255;
    m_line.color.b = 24.0 / 255;
    geometry_msgs::Point p;
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    p = transPoint(transform, p);
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // another
    m_line.points.clear();
    m_line.color.r = 0.01;
    m_line.color.g = 0.01;
    m_line.color.b = 0.99;
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  stdpair_pub.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}

geometry_msgs::Point LoopDetector::transPoint(
    tf::StampedTransform transform,
    geometry_msgs::Point p) {
  tf::Vector3 t_p(p.x, p.y, p.z);
  tf::Vector3 t_out = transform * t_p;
  geometry_msgs::Point p_out;
  p_out.x = t_out.x();
  p_out.y = t_out.y();
  p_out.z = t_out.z();
  return p_out;
}