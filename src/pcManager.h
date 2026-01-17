//
// Created by jcwang on 24-10-25.
//

#ifndef MULTI_PROXY_PCMANAGER_H
#define MULTI_PROXY_PCMANAGER_H

#include "utility.h"
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include "KeyFrame.h"

//#define checkCudaErrors(status)                                   \
//{                                                                 \
//  if (status != 0)                                                \
//  {                                                               \
//    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
//              << " at line " << __LINE__                          \
//              << " in file " << __FILE__                          \
//              << " error status: " << status                      \
//              << std::endl;                                       \
//              abort();                                            \
//    }                                                             \
//}
struct Iter_para //Interation paraments
{
  [[maybe_unused]] constexpr static int PCountN = 35947;// control count of N
  int Maxiterate;//Maximum iteration count
  double threshold;//threshold for distance Error
  double acceptrate;//accept rate
  float distance_threshold = 0.5;  // max distance between source point and its closest target point
  float relative_mse;      // icp.setEuclideanFitnessEpsilon
};

class pcManager {
private:
  int robot_id;
  std::string pcd_folder = std::string(ROOT_DIR) + "pcd";
  std::string error_file_path = std::string(ROOT_DIR) + "log";
  std::mutex mtx_filter;
  std::ofstream error_file;

  //parameters for GICP
  float icp_thres{};

public:
  pcl::VoxelGrid<PointType> downsize_filtermap;
  pcl::RandomSample<PointType> random_sample;
  Iter_para icpIter;
  pcl::PointCloud<PointType>::Ptr laserCloudIn;
  pcl::PointCloud<PointType>::Ptr cloud_temp;


  pcManager();

  ~pcManager() = default;

  void GICP(const pcl::PointCloud<PointType>::Ptr& source_in,
           const pcl::PointCloud<PointType>::Ptr& target_in,
           PointTypePose& pose_source_init, PointTypePose& pose_source_optimized);

  static pcl::PointCloud<PointTypePose>::Ptr transformPointCloud(pcl::PointCloud<PointTypePose>::Ptr cloudIn, PointTypePose &transformIn);

  void printPointCloud(const pcl::PointCloud<PointType>::Ptr& source_cloud, const pcl::PointCloud<PointType>::Ptr& target_cloud, const std::string file_name);
  void printPointCloud(const pcl::PointCloud<PointType>::Ptr &source_cloud, const std::string file_name);

  static void print4x4Matrix(const Eigen::Matrix4f & matrix);

  void setMapFilterParam(float voxelX, float voxelY, float voxelZ);

  void filterMap(const pcl::PointCloud<PointType>::Ptr& input, const pcl::PointCloud<PointType>::Ptr& output);

  void randomFilter(const pcl::PointCloud<PointType>::Ptr &input, const pcl::PointCloud<PointType>::Ptr &output, int sample);
  void setPcdFolder(const std::string &folder) { pcd_folder = folder; }

  void setIcpThreshold(float thres) { icp_thres = thres; }

  void setRobotID(int id) {
    robot_id = id;
    error_file.open(error_file_path + "/robot_" + std::to_string(robot_id) + "_pc_err.txt", std::ios::app);
    if (!error_file.is_open()) std::cerr << "Error: cannot open file " << error_file_path << std::endl;
  }
  void setErrorFilePath(const std::string &path) { error_file_path = path; }


  pcl::PointCloud<PointType>::Ptr filterInvalidPoints(const pcl::PointCloud<PointType>::Ptr &pc);
  bool isValidPoint(PointType p);
};

#endif//MULTI_PROXY_PCMANAGER_H
