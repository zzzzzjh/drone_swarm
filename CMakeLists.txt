cmake_minimum_required(VERSION 3.0.2)
project(drone_command)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  tf2_ros
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)




## Generate messages in the 'msg' folder
 add_message_files(
  DIRECTORY msg
  FILES
  ControlCommand.msg
  TrajectoryPoint.msg
  Trajectory.msg
  AttitudeReference.msg
  DroneState.msg
  Topic_for_log.msg
  ControlOutput.msg
  PositionReference.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/utils
  include/swarm
  include/single
  include/position_controller
  ${catkin_INCLUDE_DIRS}

)

add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)

###### Main File ##########
##px4_pos_controller.cpp
add_executable(px4_pos_controller src/single/px4_pos_controller.cpp)
add_dependencies(px4_pos_controller drone_command_gencpp) 
target_link_libraries(px4_pos_controller ${catkin_LIBRARIES})

##px4_pos_estimator.cpp
add_executable(px4_pos_estimator src/single/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator drone_command_gencpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)

##px4_sender.cpp
add_executable(px4_sender src/single/px4_sender.cpp)
add_dependencies(px4_sender drone_command_gencpp)
target_link_libraries(px4_sender ${catkin_LIBRARIES})

##gazebo_ground_truth.cpp
add_executable(gazebo_ground_truth src/single/gazebo_ground_truth.cpp)
add_dependencies(gazebo_ground_truth drone_command_gencpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES})
##
##get_uwb.cpp
add_executable(get_uwb src/single/get_uwb.cpp)
add_dependencies(get_uwb drone_command_gencpp)
target_link_libraries(get_uwb ${catkin_LIBRARIES})

##move.cpp
add_executable(move src/Application/move.cpp)
add_dependencies(move drone_command_gencpp)
target_link_libraries(move ${catkin_LIBRARIES})

add_executable(square src/Application/square.cpp)
add_dependencies(square drone_command_gencpp)
target_link_libraries(square ${catkin_LIBRARIES})

add_executable(set_mode src/single/set_mode.cpp)
add_dependencies(set_mode drone_command_gencpp)
target_link_libraries(set_mode ${catkin_LIBRARIES})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drone_command.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
