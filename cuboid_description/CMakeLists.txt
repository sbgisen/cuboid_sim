cmake_minimum_required(VERSION 2.8.3)
project(cuboid_description)


find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package(
		   CATKIN_DEPENDS urdf xacro
		   )

install(DIRECTORY robots meshes urdf launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_test_urdf test/test_urdf.cpp)
endif()
