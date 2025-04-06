cmake_minimum_required(VERSION 2.8.3)
project(spatial6r_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
	message_generation
	tf
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spatial3r_kinematics
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(../../../IR_library/include)
LINK_DIRECTORIES(../../../IR_library/lib)



add_executable(spatial6r_kinematics_ballMover src/ballMover.cpp)
target_link_libraries(spatial6r_kinematics_ballMover ${catkin_LIBRARIES} IRLibrary) 
add_executable(spatial6r_kinematics_armTraj src/armTraj.cpp)
target_link_libraries(spatial6r_kinematics_armTraj ${catkin_LIBRARIES} IRLibrary)
add_executable(spatial6r_kinematics_geometry_shapes src/geometry_shapes.cpp)
target_link_libraries(spatial6r_kinematics_geometry_shapes ${catkin_LIBRARIES})


