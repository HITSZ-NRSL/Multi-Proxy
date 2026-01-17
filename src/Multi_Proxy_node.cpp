
// Created by wjc on 25-3-3.
//

#include "Eigen/Core"

//
#include <thread>
//inner
#include "MultiProxyPipeline.h"



#if BACKWARD
#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}
#endif

int main(int argc, char** argv)
{
#if BACKWARD
  backward::SignalHandling sh;
#endif
  ros::init(argc, argv, "multi_proxy");
  ros::NodeHandle nh("~");

  MultiProxyPipeline pipeline(nh);
//    KeyFrame sc("init", 10, 10, 10);
  ROS_INFO("\033[1;32m----> Multi Proxy robot_%d Started.\033[0m", pipeline._robot_id_th);
  std::thread pubDesOtherThread(&MultiProxyPipeline::publishDescriptorThread, &pipeline);
  std::thread pubLoopThread(&MultiProxyPipeline::getLoopAndPubCloudThread, &pipeline);
  std::thread GeometricVerificationThread(&MultiProxyPipeline::geometricVerificationThread, &pipeline);
  std::thread OptThread(&MultiProxyPipeline::globalOptimizationThread, &pipeline);
  std::thread pubMapThread(&MultiProxyPipeline::publishGlobalMapThread, &pipeline);

  std::thread savePoseAndMapThread(&MultiProxyPipeline::savePoseAndMap, &pipeline);

  ros::spin();

  pubDesOtherThread.join();
  pubLoopThread.join();
  GeometricVerificationThread.join();
  OptThread.join();
  pubMapThread.join();

  savePoseAndMapThread.join();

  return 0;
}
