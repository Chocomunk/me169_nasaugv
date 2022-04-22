cmake_minimum_required(VERSION 3.0.2)
project(depthtolaserscan)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  sensor_msgs
  geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depthtolaserscan
#  CATKIN_DEPENDS roscpp rospy tf2_ros sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context.
## Make sure the target names don't collide across packages.
add_executable(depthtolaserscan src/depthtolaserscan.cpp)
add_executable(depthtolaserscan2 src/depthtolaserscan2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(depthtolaserscan ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a executable target against
target_link_libraries(depthtolaserscan ${catkin_LIBRARIES})
target_link_libraries(depthtolaserscan2 ${catkin_LIBRARIES})
