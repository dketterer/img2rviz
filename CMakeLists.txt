cmake_minimum_required(VERSION 2.8.3)
project(img2rviz)

find_package(catkin REQUIRED COMPONENTS
  rospy ros_numpy cv_bridge sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy ros_numpy cv_bridge sensor_msgs
)

install(PROGRAMS
  scripts/img2rviz_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
