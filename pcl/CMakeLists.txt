cmake_minimum_required(VERSION 2.8.3)
project(pcl)

set(CMAKE_CXX_STANDARD 11)
###################################
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	std_msgs
	pcl_conversions
	pcl_ros
	#nodelet
	message_generation
	jsk_recognition_msgs
	)

find_package(cv_bridge)
find_package(image_transport)
find_package(image_geometry)
find_package(camera_info_manager)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

############################ SET
#set(OpenCV_INCLUDE_DIRS /usr/local/include)
#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
#set(OpenCV_LIBS /usr/local/lib)
#set(INCLUDE_DIRS /home/bluerov2/bluerov2/src/pcl)

#set(OpenCV_INCLUDE_DIRS /home/bluerov/opencv3/include)
#set(OpenCV_LIBS /home/bluerov/opencv3/lib)
#set(INCLUDE_DIRS /home/bluerov2/bluerov2/src/pcl)

############################ service
add_service_files(FILES ComputeRMSE.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

############################ include
include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

############################ libraries
add_library(KalmanFilter lib/KalmanFilter.cpp)
target_link_libraries(KalmanFilter )

add_library(Utility lib/Utility.cpp)
target_link_libraries(Utility 
	${catkin_LIBRARIES} 
)

add_library(PointcloudFilter lib/PointcloudFilter.cpp)
target_link_libraries(PointcloudFilter )

add_library(CloudDetection lib/CloudDetection.cpp)
target_link_libraries(CloudDetection  )
add_dependencies(CloudDetection ${PROJECT_NAME}_generate_messages_cpp)

add_library(CascadeDetection lib/CascadeDetection.cpp)
target_link_libraries(CascadeDetection 	
	${OpenCV_LIBS}	
	${cv_bridge_LIBRARIES} 
	${image_transport_LIBRARIES} 
	${image_geometry_LIBRARIES} 	
	)


############################ executables
add_executable(cloud_detection_node 
	src/cloud_detection_node.cpp
)
target_link_libraries(cloud_detection_node 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
	CloudDetection 
	KalmanFilter 
	PointcloudFilter
	Utility
)

add_executable(cascade_detection_node 
	src/cascade_detection_node.cpp
)
target_link_libraries(cascade_detection_node 
	CascadeDetection
	KalmanFilter
	Utility
)

add_executable(video_to_rosmsg 
	src/video_to_rosmsg.cpp
)
target_link_libraries(video_to_rosmsg 
	${OpenCV_LIBS}	
	${cv_bridge_LIBRARIES} 
	${image_transport_LIBRARIES} 
	${camera_info_manager_LIBRARIES}
)

############################ message
message("")
message("======================================================")
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    OpenCV_DIRS: ${OpenCV_DIRS}")

message("")
message(STATUS "PCL library status:")
message(STATUS "    version: ${PCL_VERSION}")
#message(STATUS "    libraries: ${PCL_LIBRARIES}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")

message("")
message("-- catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("-- catkin_LIBRARIES: ${catkin_LIBRARIES}")
message("-- INCLUDE_DIRS: ${INCLUDE_DIRS}")

message("")
message("-- cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
message("-- cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")

message("")
message("-- image_geometry_INCLUDE_DIRS: ${image_geometry_INCLUDE_DIRS}")
message("-- image_geometry_LIBRARIES: ${image_geometry_LIBRARIES}")

message("")
message("-- image_transport_INCLUDE_DIRS: ${image_transport_INCLUDE_DIRS}")
message("-- image_transport_LIBRARIES: ${image_transport_LIBRARIES}")
message("")
message("======================================================")
message("")
