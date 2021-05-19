cmake_minimum_required(VERSION 2.8.3)

project(haullot_start_10)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
sensor_msgs
tf
rviz
)
find_package(roslaunch)


include_directories(
include
${catkin_INCLUDE_DIRS}
./src/
)

add_library(${PROJECT_NAME}
  src/haullot_robot_control.cpp
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

catkin_package()


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



add_executable(test_pub_robot_command src/test_pub_robot_command.cpp)

target_link_libraries(test_pub_robot_command
${PROJECT_NAME}
)
