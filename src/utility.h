//
// Created by jcwang on 24-10-26.
//

#ifndef MULTI_PROXY_UTILITY_H
#define MULTI_PROXY_UTILITY_H

//#include <pcl/common/common.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <utility>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/Range.h>
#include <gtsam/geometry/Pose3.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#define ROBUSTGTSAM 0
#define POINTCLOUD_VISUALIZATION 1
#define STD_DEBUG 1
#define USE_CUDA 0
#define USE_scan2map 1
#define USE_UWB_in_pgo 1
#define PUB_STREAM 1
#define BACKWARD 0
#define USE_IKDTREE 1
#define USE_MAP_GRAPH 0

// the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

typedef pcl::PointXYZI  PointType;

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  double time = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  [[nodiscard]] PointXYZIRPYT poseBetween(const PointXYZIRPYT& p) const {
    PointXYZIRPYT res{};
    Eigen::Affine3f affine_this = toAffine();
    Eigen::Affine3f affine_p = p.toAffine();
    Eigen::Affine3f affine_res = affine_this.inverse() * affine_p;
    pcl::getTranslationAndEulerAngles(affine_res, res.x, res.y, res.z, res.roll, res.pitch, res.yaw);
    return res;
  }
  PointXYZIRPYT operator*(const PointXYZIRPYT& p) const {
    PointXYZIRPYT res{};
    Eigen::Affine3f affine_this = toAffine();
    Eigen::Affine3f affine_p = p.toAffine();
    Eigen::Affine3f affine_res = affine_this * affine_p;
    pcl::getTranslationAndEulerAngles(affine_res, res.x, res.y, res.z, res.roll, res.pitch, res.yaw);
    return res;
  }

  PointXYZIRPYT(float x, float y, float z, float roll, float pitch, float yaw, float intensity, double time)
      : x(x), y(y), z(z), intensity(intensity), roll(roll), pitch(pitch), yaw(yaw), time(time) {}

  PointXYZIRPYT(){
    x = 0.0;
    y = 0.0;
    z = 0.0;
    intensity = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    time = 0.0;
  }
  PointXYZIRPYT(PointXYZIRPYT const &p)
      : x(p.x), y(p.y), z(p.z), intensity(p.intensity), roll(p.roll), pitch(p.pitch), yaw(p.yaw), time(p.time) {}

  [[nodiscard]] PointXYZIRPYT inverse() const {
    PointXYZIRPYT res{};
    Eigen::Affine3f affine_this = toAffine();
    Eigen::Affine3f affine_res = affine_this.inverse();
    pcl::getTranslationAndEulerAngles(affine_res, res.x, res.y, res.z, res.roll, res.pitch, res.yaw);
    return res;
  }

  [[nodiscard]] Eigen::Affine3f toAffine() const {
    Eigen::Affine3f res = Eigen::Affine3f::Identity();
    res = pcl::getTransformation(x, y, z, roll, pitch, yaw);
    return res;
  }

  [[nodiscard]] gtsam::Pose3 toPose3() const {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
  }
  void fromPose3(const gtsam::Pose3 &pose) {
    x = (float)pose.x();
    y = (float)pose.y();
    z = (float)pose.z();
    roll = (float)pose.rotation().roll();
    pitch = (float)pose.rotation().pitch();
    yaw = (float)pose.rotation().yaw();
  }
  void fromAffine(const Eigen::Affine3f &affine) {
    pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);
  }

  [[nodiscard]] tf::StampedTransform toStampedTransform(const std::string& frame, const std::string& child_frame) const {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    tf::StampedTransform stampedTransform(transform, ros::Time::now(), frame, child_frame);
    return stampedTransform;
  }

  float distance(PointXYZIRPYT &p) const {
    return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z));
  }
  [[nodiscard]] float norm() const {
    return sqrt(x * x + y * y + z * z);
  }

};

inline std::ostream& operator<<(std::ostream& os, const PointXYZIRPYT& point)
{
  os << " x: " << point.x
     << " y: " << point.y
     << " z: " << point.z
     << " intensity: " << point.intensity
     << " roll: " << point.roll
     << " pitch: " << point.pitch
     << " yaw: " << point.yaw
     << " time: " << point.time;
  return os;
}


inline PointXYZIRPYT operator+(const PointXYZIRPYT& p1, const PointXYZIRPYT& p2) {
  return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z, p1.roll + p2.roll, p1.pitch + p2.pitch, p1.yaw + p2.yaw, 0, -1};}

inline PointXYZIRPYT operator-(const PointXYZIRPYT& p1, const PointXYZIRPYT& p2) {
  return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z, p1.roll - p2.roll, p1.pitch - p2.pitch, p1.yaw - p2.yaw, 0, -1};}

inline PointXYZIRPYT operator*(const double a, const PointXYZIRPYT& p1) {
  return {static_cast<float>(a*p1.x), static_cast<float>(a*p1.y), static_cast<float>(a*p1.z),
          static_cast<float>(a*p1.roll), static_cast<float>(a*p1.pitch), static_cast<float>(a*p1.yaw), 0, -1};}

struct LoopClosure {
  int r_frame_id = 0;
  int c_frame_id = 0;
  gtsam::Pose3 pose; //always from c_frame_id wrt r_frame_id
  gtsam::Pose3 frame_trans; //always from c_map wrt r_map
  gtsam::Matrix cov;
  LoopClosure() = default;
  LoopClosure(int r_frame_id, int c_frame_id, const gtsam::Pose3& pose3, const gtsam::Pose3& map_trans, gtsam::Matrix matrix)
      : r_frame_id(r_frame_id), c_frame_id(c_frame_id), pose(pose3), frame_trans(map_trans), cov(std::move(matrix)){}
  LoopClosure(int r_frame_id, int c_frame_id, const gtsam::Pose3& pose3, const gtsam::Pose3& map_trans, boost::array<double, 36> cov_array)
      : r_frame_id(r_frame_id), c_frame_id(c_frame_id), pose(pose3), frame_trans(map_trans), cov(gtsam::Matrix::Map(cov_array.data(), 6, 6)){}

  bool operator==(const LoopClosure &other) const {
    return r_frame_id == other.r_frame_id && c_frame_id == other.c_frame_id && pose.equals(other.pose, 1e-1);

  }
};

struct RobotID {
  std::string robot_name;
  int robot_id = -1;
  bool isAlive = true;
  double last_heartbeat = 0.0; // last heartbeat time in seconds
  ros::Publisher pub_descriptor_info;
  ros::Publisher pub_candidate_info;
  ros::Publisher pub_verified_info;
  ros::Publisher pub_dual_info;
  std::deque<int> blocked_descriptor_ids;
  std::deque<LoopClosure> blocked_candidate_ids;
  std::deque<LoopClosure> blocked_verified_ids;
  RobotID() = default;
  RobotID(std::string name, int id) : robot_name(std::move(name)), robot_id(id) {}
  RobotID(std::string name, int id, bool alive, double heartbeat)
      : robot_name(std::move(name)), robot_id(id), isAlive(alive), last_heartbeat(heartbeat) {}
};

typedef PointXYZIRPYT  PointTypePose;

typedef pcl::PointXYZINormal PointTypeNormal;
namespace Utility {
typedef std::lock_guard<std::recursive_mutex> Guard;

  template <typename scalar>
  inline bool isValidFloat(scalar value) {
    if (std::isnan(value) || std::isinf(value) || !std::isfinite(value))
      return false;
    return true;
  }

class TicToc {
public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() { // ms
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}

inline Eigen::Matrix4d QuatTtoSE3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = q.toRotationMatrix();
  T.block<3,1>(0,3) = t;
  return T;
}

inline void SEtoQuatT(const Eigen::Matrix4d& T, Eigen::Quaterniond& q, Eigen::Vector3d& t) {
  q = Eigen::Quaterniond(T.block<3,3>(0,0));
  q.normalize();
  t = T.block<3,1>(0,3);
}


inline gtsam::Pose3 AvgSE3(const std::vector<gtsam::Pose3>& poses) {
  if (poses.empty()) throw std::runtime_error("Empty SE3 list");
  if (poses.size() == 1) return poses.front();

  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  Eigen::Vector3d avg_t = Eigen::Vector3d::Zero();
  for (const auto& pose : poses) {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      SEtoQuatT(pose.matrix(), q, t);
      Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
      M += v * v.transpose();
      avg_t += t;
  }
  avg_t /= poses.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(M);
  Eigen::Vector4d dominant_eigenvec = solver.eigenvectors().col(3);

  Eigen::Quaterniond avg_q(
      dominant_eigenvec[0],
      dominant_eigenvec[1],
      dominant_eigenvec[2],
      dominant_eigenvec[3]
  );
  gtsam::Pose3 pose_res = gtsam::Pose3(QuatTtoSE3(avg_q.normalized(), avg_t));
  return pose_res;
}

inline gtsam::Pose3 KarcherMeanSE3(const std::vector<gtsam::Pose3>& poses, int max_iter = 10, double tol = 1e-6) {
  if (poses.empty()) throw std::runtime_error("Empty poses");


  gtsam::Pose3 mean = poses[0];

  for (int iter = 0; iter < max_iter; ++iter) {
    Eigen::Matrix<double, 6, 1> delta_sum = Eigen::Matrix<double, 6, 1>::Zero();


    for (const auto& pose : poses) {

      Eigen::Matrix<double, 6, 1> delta = gtsam::Pose3::Logmap(pose.between(mean));
      delta_sum += delta;
    }


    Eigen::Matrix<double, 6, 1> update = -delta_sum / poses.size();
    mean = mean * gtsam::Pose3::Expmap(update);


    if (update.norm() < tol) break;
  }

  return mean;
}

inline gtsam::Pose3 gtsamMeanSE3(const std::vector<gtsam::Pose3>& poses) {
  gtsam::NonlinearFactorGraph graph_meature;
  gtsam::Values initial_estimate;

  gtsam::SharedNoiseModel noise_prior =
      gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-0, 1e-0, 1e-0).finished());

  for (const auto & pose : poses) {
    graph_meature.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, noise_prior));
  }
  initial_estimate.insert(0, poses.front());
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("ERROR");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_meature, initial_estimate, params);
  gtsam::Values result = optimizer.optimize();
  if (result.exists(0)) {
    return result.at<gtsam::Pose3>(0);
  } else {
    throw std::runtime_error("Optimization failed to converge.");
  }
}


struct pair_hash {
  template <typename T1, typename T2>
  std::size_t operator ()(const std::pair<T1, T2>& p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
  }
};

#endif //MULTI_PROXY_UTILITY_H
