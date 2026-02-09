cmake_minimum_required(VERSION 3.0.2)
project(pil_ayiklama_sim)

## C++ Standard
add_compile_options(-std=c++14)

## Find catkin and packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
  xacro
  robot_state_publisher
  cv_bridge
  image_transport
  message_filters
)

## Find Gazebo
find_package(gazebo REQUIRED)

## Catkin package
catkin_package(
  LIBRARIES random_box_spawner conveyor_mover grinder_remover
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs gazebo_ros
  DEPENDS GAZEBO
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

###########
## Build ##
###########

## Gazebo plugins
add_library(random_box_spawner src/random_box_spawner.cpp)
target_link_libraries(random_box_spawner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(random_box_spawner ${catkin_EXPORTED_TARGETS})

add_library(conveyor_mover src/conveyor_mover.cpp)
target_link_libraries(conveyor_mover ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(conveyor_mover ${catkin_EXPORTED_TARGETS})

add_library(grinder_remover src/grinder_remover.cpp)
target_link_libraries(grinder_remover ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(grinder_remover ${catkin_EXPORTED_TARGETS})

## Python scripts
catkin_install_python(PROGRAMS
  scripts/color_detector.py
  scripts/sorter_controller.py
  scripts/conveyor_driver.py
  scripts/system_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

install(TARGETS random_box_spawner conveyor_mover grinder_remover
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY urdf worlds launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
