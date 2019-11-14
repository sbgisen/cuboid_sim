cmake_minimum_required(VERSION 2.8.3)
project(cuboid_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(DEPENDS gazebo_ros)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
# TODO: Check names of system library include directories (gazebo_ros, joint_state_publisher, robot_state_publisher, nodelet, laser_filters, pcl_ros, gazebo_ros_pkgs, gazebo_ros_control, velodyne_gazebo_plugins, cuboid_navigation, cuboid_description)
)

install(DIRECTORY launch worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
