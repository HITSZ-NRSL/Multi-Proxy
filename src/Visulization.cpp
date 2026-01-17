//
// Created by jcwang on 25-9-7.
//

#include "Visulization.h"

void Visulization::publishMapGlobalCurrent(ros::Time time_lidar_stamp, int ref_robot_id, PointTypePose current2global) {
  std::string reference_frame = robot_name + "/map";
  static double last_time = 0.0;
  Eigen::Affine3f t_current2global = Eigen::Affine3f::Identity();
  if (ref_robot_id != robot_id_th) {
    reference_frame = "robot_" + std::to_string(ref_robot_id) + "/map";
  }

  if (pub_fused_map_frame.getNumSubscribers() != 0)
  {
    pcl::PointCloud<PointType>::Ptr tempCloud_c;
    sensor_msgs::PointCloud2 tempCloud;
    tempCloud_c = cur_info_list->getMap();
    pcl::toROSMsg(*tempCloud_c, tempCloud);
    tempCloud.header.stamp = time_lidar_stamp;
    tempCloud.header.frame_id = robot_name + "/map";
    pub_fused_map_frame.publish(tempCloud);
  }

  if (time_lidar_stamp.toSec() > last_time) {
    // send map to world
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current2global.x, current2global.y, current2global.z));

    tf::Quaternion q = tf::createQuaternionFromRPY(current2global.roll, current2global.pitch, current2global.yaw);
    transform.setRotation( q );
    if (ref_robot_id == robot_id_th || ref_robot_id == 0) {
      br.sendTransform( tf::StampedTransform( transform, time_lidar_stamp,
                                              "world",
                                              robot_name + "/map"));
    }
    else {
      br.sendTransform( tf::StampedTransform( transform, time_lidar_stamp,
                                              reference_frame,
                                              robot_name + "/map"));
    }

    if (robot_name + "/map" != "robot_" + std::to_string(robot_id_th) + "/map") {
      // send robot_name to robot_id
      transform.setOrigin(tf::Vector3(0, 0, 0));
      tf::Quaternion q_identity = tf::createQuaternionFromRPY(0, 0, 0);
      transform.setRotation(q_identity);
      br.sendTransform( tf::StampedTransform( transform, time_lidar_stamp,
                                              robot_name + "/map",
                                              "robot_" + std::to_string(robot_id_th) + "/map"));
    }

    last_time = time_lidar_stamp.toSec();
  }
}

void Visulization::visualizeLoopClosure(ros::Time time_lidar_stamp, std::vector<LoopClosure> &accepted_loops) {
  if (accepted_loops.empty()) return;
  visualization_msgs::MarkerArray markerArray;
  // loop nodes
  visualization_msgs::Marker markerNodeCur;
  visualization_msgs::Marker markerNodeRef;
  markerNodeCur.header.frame_id = robot_name + "/map";
  markerNodeCur.header.stamp = time_lidar_stamp;
  markerNodeCur.action = visualization_msgs::Marker::ADD;
  markerNodeCur.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNodeCur.ns = robot_name + "_loop_frame_cur_nodes";
  markerNodeCur.id = 0;
  markerNodeCur.pose.orientation.w = 1;
  markerNodeCur.scale.x = 0.75;
  markerNodeCur.scale.y = 0.75;
  markerNodeCur.scale.z = 0.75;
  markerNodeCur.color.r = 1;
  markerNodeCur.color.g = 0.2;
  markerNodeCur.color.b = 0;
  markerNodeCur.color.a = 1;
  markerNodeRef.header.frame_id = robot_name + "/map";
  markerNodeRef.header.stamp = time_lidar_stamp;
  markerNodeRef.action = visualization_msgs::Marker::ADD;
  markerNodeRef.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNodeRef.ns = robot_name + "_loop_frame_ref_nodes";
  markerNodeRef.id = 1;
  markerNodeRef.pose.orientation.w = 1;
  markerNodeRef.scale.x = 0.75;
  markerNodeRef.scale.y = 0.75;
  markerNodeRef.scale.z = 0.75;
  markerNodeRef.color.r = 0;
  markerNodeRef.color.g = 0.2;
  markerNodeRef.color.b = 1;
  markerNodeRef.color.a = 1;

  // loop edges
  visualization_msgs::Marker markerEdge;
  markerEdge.header.frame_id = robot_name + "/map";
  markerEdge.header.stamp = time_lidar_stamp;
  markerEdge.action = visualization_msgs::Marker::ADD;
  markerEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge.ns = robot_name + "_loop_frame_edges";
  markerEdge.id = 2;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.4;
  markerEdge.color.r = 0.9;
  markerEdge.color.g = 0.9;
  markerEdge.color.b = 0;
  markerEdge.color.a = 1;
  std::string accept_loop_file_name = log_folder + "/" + robot_name + "_outlier_reject.txt";
  for (const auto& loop : accepted_loops) {
    geometry_msgs::Point p_cur, p_ref;
    std::shared_ptr<KeyFrame> sc_cur, sc_ref;
    sc_cur = cur_info_list->getKeyFrame(loop.c_frame_id);
    sc_ref = ref_info_list->getKeyFrame(loop.r_frame_id);
    if (nullptr == sc_ref) sc_ref = cur_info_list->getKeyFrame(loop.r_frame_id);
    if (!sc_ref->has_optimized) {
      continue;
    }
    p_cur.x = sc_cur->distributed_pose_in_cur_map.x;
    p_cur.y = sc_cur->distributed_pose_in_cur_map.y;
    p_cur.z = sc_cur->distributed_pose_in_cur_map.z;
    markerNodeCur.points.push_back(p_cur);
    markerEdge.points.push_back(p_cur);
    p_ref.x = sc_ref->distributed_pose_in_cur_map.x;
    p_ref.y = sc_ref->distributed_pose_in_cur_map.y;
    p_ref.z = sc_ref->distributed_pose_in_cur_map.z;
    markerNodeRef.points.push_back(p_ref);
    markerEdge.points.push_back(p_ref);
  }

  markerArray.markers.push_back(markerNodeCur);
  markerArray.markers.push_back(markerNodeRef);
  markerArray.markers.push_back(markerEdge);
  pub_loop_constraint_edge.publish(markerArray);
}


void Visulization::visualizeKeyFrame(ros::Time time_lidar_stamp) {
  visualization_msgs::MarkerArray markerArray;
  // key nodes current
  visualization_msgs::Marker markerNode;
  markerNode.header.frame_id = robot_name + "/map";
  markerNode.header.stamp = time_lidar_stamp;
  markerNode.action = visualization_msgs::Marker::ADD;
  markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode.ns = robot_name + "_Key_frames";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.5;
  markerNode.scale.y = 0.5;
  markerNode.scale.z = 0.5;
  markerNode.color.a = 1.0;
  // odom edge
  visualization_msgs::Marker markerEdge;
  markerEdge.header.frame_id = robot_name + "/map";
  markerEdge.header.stamp = time_lidar_stamp;
  markerEdge.action = visualization_msgs::Marker::ADD;
  markerEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge.ns = robot_name + "_frame_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.25;

  if (robot_id_th == 0) {
    markerEdge.color.r = 0.6;
    markerEdge.color.g = 0;
    markerEdge.color.b = 0.6;
    markerEdge.color.a = 0.8;
  } else if (robot_id_th == 1) {
    markerEdge.color.r = 0.6;
    markerEdge.color.g = 0.6;
    markerEdge.color.b = 0;
    markerEdge.color.a = 0.8;
  } else {
    markerEdge.color.r = 0;
    markerEdge.color.g = 0.6;
    markerEdge.color.b = 0.6;
    markerEdge.color.a = 0.8;
  }


  visualization_msgs::Marker textMarker;
  textMarker.header.frame_id = robot_name + "/map";
  textMarker.header.stamp = time_lidar_stamp;
  textMarker.ns = robot_name + "_Key_frame_ids";
  textMarker.action = visualization_msgs::Marker::ADD;
  textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  textMarker.pose.orientation.w = 1.0;
  textMarker.scale.z = 0.5;
  textMarker.color.a = 1.0;
  textMarker.color.r = 0.0;
  textMarker.color.g = 0.0;
  textMarker.color.b = 0.0;

  geometry_msgs::Point p_last;
  int degenerate_last_idx = 0;
  for (int i = 0; i < cur_info_list->size(); ++i){
    std::shared_ptr<KeyFrame> sc_cur;
    sc_cur = cur_info_list->at(i);
    if (sc_cur->is_key_frame) {
      geometry_msgs::Point p_opted, p_init;
      std_msgs::ColorRGBA c;
      p_opted.x = sc_cur->distributed_pose_in_cur_map.x;
      p_opted.y = sc_cur->distributed_pose_in_cur_map.y;
      p_opted.z = sc_cur->distributed_pose_in_cur_map.z;

      p_init.x = sc_cur->getPoseInit().x;
      p_init.y = sc_cur->getPoseInit().y;
      p_init.z = sc_cur->getPoseInit().z;

      if (sc_cur->is_degenerate && sc_cur->is_corrected) {
        c.a = 0.9;
        c.r = 0;
        c.g = 0;
        c.b = 1.0;
        degenerate_last_idx = i;
      } else if (sc_cur->is_degenerate) {
        c.a = 0.9;
        c.r = 1.0;
        c.g = 0;
        c.b = 0;
        degenerate_last_idx = i;
      } else {
        c.a = 0.9;
        c.r = 0;
        c.g = 1.0;
        c.b = 0;
      }
      markerNode.points.push_back(p_opted);
      markerNode.colors.push_back(c);
      if (i > 0) {
        markerEdge.points.push_back(p_last);
        markerEdge.points.push_back(p_opted);
      }


      visualization_msgs::Marker text = textMarker;
      text.id = sc_cur->frame_id;
      text.pose.position = p_opted;
      text.pose.position.z += 0.5;
      text.text = std::to_string(sc_cur->frame_id);

      markerArray.markers.push_back(text);

      p_last = p_opted;
    }
  }

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  pub_key_frame_node.publish(markerArray);
}

void Visulization::visualizeDualFrames(ros::Time time_lidar_stamp, std::unordered_map< int, DualFrameList> &dual_lists) {
  visualization_msgs::MarkerArray marker_array;


  int count = 0;
  for (auto &df_list : dual_lists) {
    DualFrameList &dual_frames = df_list.second;

    int min_robot_id = dual_frames.ref_robot_id;
    if (!dual_frames.dual_frames_remote.empty()) {

      visualization_msgs::Marker scatter_marker;
      scatter_marker.header.frame_id = "robot_" + std::to_string(min_robot_id) + "/map";
      scatter_marker.header.stamp = time_lidar_stamp;
      scatter_marker.action = visualization_msgs::Marker::ADD;
      scatter_marker.type = visualization_msgs::Marker::CUBE_LIST;
      scatter_marker.ns = robot_name + "_dual_prior_remote";
      scatter_marker.id = 0;
      scatter_marker.pose.orientation.w = 1.0;
      scatter_marker.scale.x = 0.1;
      scatter_marker.scale.y = 0.1;
      scatter_marker.scale.z = 0.1;
      if (robot_id_th == 0) {
        scatter_marker.color.r = 0.8;
        scatter_marker.color.g = 0;
        scatter_marker.color.b = 0.8;
        scatter_marker.color.a = 0.6;
      } else if (robot_id_th == 1) {
        scatter_marker.color.r = 0.8;
        scatter_marker.color.g = 0.8;
        scatter_marker.color.b = 0;
        scatter_marker.color.a = 0.6;
      } else {
        scatter_marker.color.r = 0;
        scatter_marker.color.g = 0.8;
        scatter_marker.color.b = 0.8;
        scatter_marker.color.a = 0.6;
      }

      for (const auto &it: dual_frames.dual_frames_remote) {
        int32_t frame_id = it.first;
        const auto &dual_frame = it.second;

        geometry_msgs::Point p;
        p.x = dual_frame.pose.translation().x();
        p.y = dual_frame.pose.translation().y();
        p.z = dual_frame.pose.translation().z();
        scatter_marker.points.push_back(p);
      }
      marker_array.markers.push_back(scatter_marker);
    }

    if (!dual_frames.dual_frames_local.empty()) {

      visualization_msgs::Marker scatter_marker_local;
      scatter_marker_local.header.frame_id = "robot_" + std::to_string(min_robot_id) + "/map";
      scatter_marker_local.header.stamp = time_lidar_stamp;
      scatter_marker_local.action = visualization_msgs::Marker::ADD;
      scatter_marker_local.type = visualization_msgs::Marker::SPHERE_LIST;
      scatter_marker_local.ns = robot_name + "_dual_prior_local";
      scatter_marker_local.id = 0;
      scatter_marker_local.pose.orientation.x = 0.0;
      scatter_marker_local.pose.orientation.y = 0.0;
      scatter_marker_local.pose.orientation.z = 0.0;
      scatter_marker_local.pose.orientation.w = 1.0;
      scatter_marker_local.scale.x = 0.1;
      scatter_marker_local.scale.y = 0.1;
      scatter_marker_local.scale.z = 0.1;
      if (robot_id_th == 0) {
        scatter_marker_local.color.r = 0.8;
        scatter_marker_local.color.g = 0.2;
        scatter_marker_local.color.b = 0.8;
        scatter_marker_local.color.a = 0.6;
      } else if (robot_id_th == 1) {
        scatter_marker_local.color.r = 0.8;
        scatter_marker_local.color.g = 0.8;
        scatter_marker_local.color.b = 0.2;
        scatter_marker_local.color.a = 0.6;
      } else {
        scatter_marker_local.color.r = 0.2;
        scatter_marker_local.color.g = 0.8;
        scatter_marker_local.color.b = 0.8;
        scatter_marker_local.color.a = 0.6;
      }

      for (const auto &it: dual_frames.dual_frames_local) {
        int32_t frame_id = it.first;
        const auto &dual_frame = it.second;

        geometry_msgs::Point p;
        p.x = dual_frame.pose.translation().x();
        p.y = dual_frame.pose.translation().y();
        p.z = dual_frame.pose.translation().z();
        scatter_marker_local.points.push_back(p);

        visualization_msgs::Marker arror;
        arror.header.frame_id = "robot_" + std::to_string(min_robot_id) + "/map";
        arror.header.stamp = time_lidar_stamp;
        arror.action = visualization_msgs::Marker::ADD;
        arror.type = visualization_msgs::Marker::ARROW;
        arror.ns = robot_name + "_dual_error_arrow";
        arror.id = ++count + 1;
        arror.scale.x = 0.05;// shaft diameter
        arror.scale.y = 0.1;// head diameter
        arror.scale.z = 0.1;// head length
        arror.pose.orientation.w = 1.0;
        arror.pose.orientation.x = 0.0;
        arror.pose.orientation.y = 0.0;
        arror.pose.orientation.z = 0.0;
        arror.pose.position.x = 0.0;
        arror.pose.position.y = 0.0;
        arror.pose.position.z = 0.0;

        gtsam::Pose3 pose_end = dual_frame.pose * dual_frame.pose_error;
        geometry_msgs::Point end;
        end.x = p.x + (pose_end.translation().x() - p.x) * 40;
        end.y = p.y + (pose_end.translation().y() - p.y) * 40;
        end.z = p.z + (pose_end.translation().z() - p.z) * 40;
        arror.points.push_back(p);
        arror.points.push_back(end);
        arror.color.a = 1.0f;
        if (robot_id_th == 0) {
          arror.color.r = 0.8;
          arror.color.g = 0.2;
          arror.color.b = 0.8;
        } else if (robot_id_th == 1) {
          arror.color.r = 0.8;
          arror.color.g = 0.8;
          arror.color.b = 0.2;
        } else {
          arror.color.r = 0.2;
          arror.color.g = 0.8;
          arror.color.b = 0.8;
        }
        marker_array.markers.push_back(arror);
        arror.ns = robot_name + "_dual_direction_arrow";
        arror.id = count + 1;
        arror.scale.x = 0.01;// shaft diameter
        arror.scale.y = 0.02;// head diameter
        arror.scale.z = 0.02;// head length
        arror.points.clear();
        gtsam::Pose3 pose_end_direction = dual_frame.pose * gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0.2, 0, 0));
        end.x = pose_end_direction.translation().x();
        end.y = pose_end_direction.translation().y();
        end.z = pose_end_direction.translation().z();
        arror.points.push_back(p);
        arror.points.push_back(end);
        marker_array.markers.push_back(arror);
      }
      marker_array.markers.push_back(scatter_marker_local);
    }
  }

  pub_dual_viz.publish(marker_array);
}

