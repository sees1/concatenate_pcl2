cmake_minimum_required(VERSION 3.0.2)
project(concatenate_pcl2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  laser_geometry
  tf2
  tf2_ros
  tf2_msgs
  std_msgs
  tf2_sensor_msgs
  geometry_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/coordinates.cfg
)


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
  INCLUDE_DIRS
  LIBRARIES concatenate_pcl2
  CATKIN_DEPENDS roscpp sensor_msgs tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  sensor_msgs
  pcl_ros
)

link_directories( ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(concatenate_pcl2 src/concatenate_pcl2.cpp)
target_link_libraries(concatenate_pcl2 ${catkin_LIBRARIES})
add_dependencies(concatenate_pcl2 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_gencfg})

install(TARGETS concatenate_pcl2
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)