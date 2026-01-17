set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.5) # 所需CMake版本
project(multi_proxy) # 软件包名称

#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#	message("OPENMP FOUND")
#endif ()

#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17" )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread -fopenmp")
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings") # 关闭pcl烦人的警告
# set(PYTHON_EXECUTABLE /usr/bin/python3)
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
include_directories(BEFORE SYSTEM "/usr/local/include/eigen3")
#set(CUDA_BIN_PATH /usr/local/cuda/)
# 查找构建所需的其他CMake / Catkin软件包
find_package(catkin REQUIRED COMPONENTS
		tf
		roscpp
		rospy
		std_msgs
		sensor_msgs
		geometry_msgs
		nav_msgs
		message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
	message(STATUS "Eigen3 found at: ${EIGEN3_INCLUDE_DIRS}")
endif()

if(GTSAM_FOUND)
	message(STATUS "GTSAM found at: ${GTSAM_INCLUDE_DIRS}")
endif()
# 服务和消息
add_service_files(
		FILES
		save_traj.srv
)

# 调用消息生成
generate_messages(
		DEPENDENCIES
		geometry_msgs
		std_msgs
		nav_msgs
		sensor_msgs
)

# 指定package build info export  CATKIN_DEPENDS指定，编译需要依赖的其他ros功能包
catkin_package(
		INCLUDE_DIRS
		DEPENDS PCL GTSAM
		CATKIN_DEPENDS
		std_msgs
		nav_msgs
		geometry_msgs
		sensor_msgs
		message_runtime
		message_generation
)

# include directories 用于添加头文件搜索路径,如果都catkin了但是还是找不到头文件，那么就是没有在这里添加头文件搜索路径
include_directories(
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${GTSAM_INCLUDE_DIR}
		third_parties
)

# link directories 用于添加额外的库路径，但是不推荐这样做。
# 所有catkin和CMake软件包在find_packaged时都会自动添加链接信息。 只需链接到
link_directories(
		${PCL_LIBRARY_DIRS}
		${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Maximum Clique Solver 用于指定要构建的库。target_link_libraries()中调用
add_library(fast_max-clique_finder
		third_parties/fast_max-clique_finder/src/findClique.h
		third_parties/fast_max-clique_finder/src/graphIO.h
		third_parties/fast_max-clique_finder/src/findClique.cpp
		third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
		third_parties/fast_max-clique_finder/src/findCliqueHeuInc.cpp
		third_parties/fast_max-clique_finder/src/utils.cpp
		third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

add_library(STDesc
		third_parties/STDesc/STDesc.cpp
		third_parties/STDesc/STDesc.h
)

target_link_libraries(STDesc ${PCL_LIBRARIES}
		${CERES_LIBRARIES})

# Map Fushion
add_executable(${PROJECT_NAME}_node src/Multi_Proxy_node.cpp
		src/BackEnd.cpp
		src/DualFrame.cpp
		src/KeyFrame.cpp
		src/KeyFrameList.cpp
		src/LoopDetector.cpp
		src/MultiOutlierRejection.cpp
		src/pcManager.cpp
        src/Visulization.cpp
        src/Visulization.h
		src/MultiProxyPipeline.cpp
		src/MultiProxyPipeline.h
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		gtsam fast_max-clique_finder STDesc
#		dw
)
