cmake_minimum_required(VERSION 2.8.3)
project(cluster_extraction)

find_package(
	catkin REQUIRED COMPONENTS
	pcl_ros REQUIRED
	cmake_modules
	roscpp
	trajectory_msgs
	std_msgs
	message_runtime
	phoebe_planning_msgs
	sensor_msgs
	cv_bridge
  	image_transport
  	geometry_msgs
)

find_package(
	PCL REQUIRED
)

find_package(
	OpenCV REQUIRED
)

 add_message_files(
   FILES
   manipulatorAdjustment.msg
   objectDistance.msg
 )

 generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
    trajectory_msgs
    shape_msgs
 )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${pcl_ros_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable (cluster_extraction src/cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_ros_LIBRARIES})

add_executable (cluster_transform src/cluster_transform.cpp)
target_link_libraries (cluster_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_ros_LIBRARIES})

add_executable (hand_transform src/hand_transform.cpp)
target_link_libraries (hand_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_ros_LIBRARIES})

add_executable (cluster_controller src/cluster_controller.cpp)
target_link_libraries (cluster_controller ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${pcl_ros_LIBRARIES})

add_executable( manipulator_adjuster src/manipulator_adjuster.cpp src/Corner.cpp src/Corner.h)
target_link_libraries ( manipulator_adjuster ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )