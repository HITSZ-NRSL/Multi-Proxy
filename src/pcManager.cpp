//
// Created by jcwang on 24-10-25.
//

#include "pcManager.h"

pcManager::pcManager(){
  downsize_filtermap.setLeafSize(0.2, 0.2, 0.2);
  random_sample.setSample(6000);
  icpIter.Maxiterate = 50;
  icpIter.threshold = 1e-8;
  icpIter.acceptrate = 1.0;
  icpIter.distance_threshold = 0.5;
  icpIter.relative_mse = 1e-4;
  laserCloudIn.reset(new pcl::PointCloud<PointType>());
  cloud_temp.reset(new pcl::PointCloud<PointType>(100000,1));
  pcd_folder = std::string(ROOT_DIR) + "pcd";
  error_file_path = std::string(ROOT_DIR) + "log";
}

void pcManager::GICP(const pcl::PointCloud<PointType>::Ptr& source_in,
                    const pcl::PointCloud<PointType>::Ptr& target_in,
                    PointTypePose& pose_source_init, PointTypePose& pose_source_optimized)
{
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setMaxCorrespondenceDistance(icpIter.distance_threshold);
  gicp.setMaximumIterations(icpIter.Maxiterate);
  gicp.setTransformationEpsilon(icpIter.threshold);
  gicp.setEuclideanFitnessEpsilon(icpIter.relative_mse);
  gicp.setRANSACIterations(1);

  pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());

  randomFilter(source_in, source, (int)fmax(source_in->size() / 10, 5000));
  randomFilter(target_in, target, (int)fmax(target_in->size() / 10, 5000));

  cloud_temp->clear();

  gicp.setInputSource(source);
  gicp.setInputTarget(target);
  gicp.align(*cloud_temp);

  if (gicp.hasConverged() == false){
    pose_source_optimized.intensity = -1;
    return;
  }

  Eigen::Affine3f correctionLidarFrame;

  correctionLidarFrame = gicp.getFinalTransformation();  // get transformation in camera frame

  // transform from world origin to wrong pose
  Eigen::Affine3f tWrong = pose_source_init.toAffine();
  // transform from world origin to corrected pose
  Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;

  // pre-multiplying -> successive rotation about a fixed frame
  pose_source_optimized.fromAffine(tCorrect);
  pose_source_optimized.intensity = (float)gicp.getFitnessScore();
}

void pcManager::print4x4Matrix(const Eigen::Matrix4f & matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %f %f %f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %f %f %f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %f %f %f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %f, %f, %f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

pcl::PointCloud<PointTypePose>::Ptr pcManager::transformPointCloud(pcl::PointCloud<PointTypePose>::Ptr cloudIn, PointTypePose &transformIn)
{
  pcl::PointCloud<PointTypePose>::Ptr cloudOut(new pcl::PointCloud<PointTypePose>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = transformIn.toAffine();

#pragma omp parallel for num_threads(2)
  for (int i = 0; i < cloudSize; ++i)
  {
//            std::cout << "current id: " << omp_get_thread_num() << endl;
    PointTypePose pointFrom = cloudIn->points[i];
    Eigen::Affine3f transOdometry = pointFrom.toAffine();
    pcl::getTranslationAndEulerAngles(transCur * transOdometry, cloudOut->points[i].x, cloudOut->points[i].y, cloudOut->points[i].z, cloudOut->points[i].roll, cloudOut->points[i].pitch, cloudOut->points[i].yaw);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }
  return cloudOut;
}

void pcManager::printPointCloud(const pcl::PointCloud<PointType>::Ptr& source_cloud, const pcl::PointCloud<PointType>::Ptr& target_cloud, std::string file_name)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudALL (new pcl::PointCloud<pcl::PointXYZRGB>());
  // Fill in the CloudIn data
  for (int i = 0; i < source_cloud->size(); i++)
  {
    pcl::PointXYZRGB pointsrc;
    pointsrc.x = source_cloud->points[i].x;
    pointsrc.y = source_cloud->points[i].y;
    pointsrc.z = source_cloud->points[i].z;
    pointsrc.r = 255;
    pointsrc.g = 0;
    pointsrc.b = 0;
    cloudALL->push_back(pointsrc);
  }
  for (int i = 0; i < target_cloud->size(); i++)
  {
    pcl::PointXYZRGB pointtar;
    pointtar.x = target_cloud->points[i].x;
    pointtar.y = target_cloud->points[i].y;
    pointtar.z = target_cloud->points[i].z;
    pointtar.r = 0;
    pointtar.g = 255;
    pointtar.b = 255;
    cloudALL->push_back(pointtar);
  }
  pcl::io::savePCDFileASCII<pcl::PointXYZRGB> (pcd_folder + "/" + file_name + ".pcd", *cloudALL);
}

void pcManager::printPointCloud(const pcl::PointCloud<PointType>::Ptr& source_cloud, const std::string file_name)
{
  pcl::io::savePCDFileASCII<PointType> (pcd_folder + "/" + file_name + ".pcd", *source_cloud);
}

pcl::PointCloud<PointType>::Ptr pcManager::filterInvalidPoints(const pcl::PointCloud<PointType>::Ptr& pc) {
  pcl::PointCloud<PointType>::Ptr valid_pc(new pcl::PointCloud<PointType>());
  for (auto p : pc->points) {
    if (!isValidPoint(p)) continue;

//    float dis = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    // if (pt.norm() > lidar_range_max_) continue;

    valid_pc->push_back(p);
  }
  return valid_pc;
}

bool pcManager::isValidPoint(PointType p) {
  if (!Utility::isValidFloat(p.x) || !Utility::isValidFloat(p.y) ||
      !Utility::isValidFloat(p.z))
    return false;
  return true;
}

void pcManager::setMapFilterParam(float voxelX, float voxelY, float voxelZ) {
  downsize_filtermap.setLeafSize(voxelX, voxelY, voxelZ);
}

void pcManager::filterMap(const pcl::PointCloud<PointType>::Ptr& input, const pcl::PointCloud<PointType>::Ptr& output) {
  downsize_filtermap.setInputCloud(input);
  downsize_filtermap.filter(*output);
}

void pcManager::randomFilter(const pcl::PointCloud<PointType>::Ptr& input, const pcl::PointCloud<PointType>::Ptr& output, int sample) {
  static int count;
  try {
    std::lock_guard<std::mutex> lock(mtx_filter);
    random_sample.setSample(sample);
    random_sample.setInputCloud(input);
    random_sample.filter(*output);
  } catch (std::exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
    error_file << "--------------------------------------------- Time stamp:" << ros::Time::now() << " -------------------------" << std::endl;
    printPointCloud(input, "error_filter_pointcloud" + std::to_string(count));
    error_file << "<Pc_manger> " + std::to_string(count) + " filter Error:" << e.what() << std::endl;
    count++;
  }
}
