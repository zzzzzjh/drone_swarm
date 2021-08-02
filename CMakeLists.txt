cmake_minimum_required(VERSION 3.0.2)
project(drone_swarm)

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
  drone_msg
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)




## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
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
  nav_msgs
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
<<<<<<< HEAD
=======
  include/single
  include/position_controller
>>>>>>> 7756ba2e33a1848426e4fbb36823146bbdf8c064
  ${catkin_INCLUDE_DIRS}

)

#add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
#add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)

###### Main File ##########



###### Swarm ##########

##swarm_controller.cpp
add_executable(swarm_controller src/swarm/swarm_controller.cpp)
add_dependencies(swarm_controller drone_swarm_gencpp)
target_link_libraries(swarm_controller ${catkin_LIBRARIES})

##swarm_estimator.cpp
add_executable(swarm_estimator src/swarm/swarm_estimator.cpp)
add_dependencies(swarm_estimator drone_swarm_gencpp)
target_link_libraries(swarm_estimator ${catkin_LIBRARIES})

##swarm_formation_control.cpp
add_executable(swarm_formation_control src/swarm/swarm_formation_control.cpp)
add_dependencies(swarm_formation_control drone_swarm_gencpp)
target_link_libraries(swarm_formation_control ${catkin_LIBRARIES})

##swarm_ground_station.cpp
add_executable(swarm_ground_station src/swarm/swarm_ground_station.cpp)
add_dependencies(swarm_ground_station drone_swarm_gencpp)
target_link_libraries(swarm_ground_station ${catkin_LIBRARIES})

##swarm_terminal_control.cpp
add_executable(swarm_terminal_control src/swarm/swarm_terminal_control.cpp)
add_dependencies(swarm_terminal_control drone_swarm_gencpp)
target_link_libraries(swarm_terminal_control ${catkin_LIBRARIES})

##swarm_gazebo_truth.cpp
add_executable(swarm_gazebo_truth src/swarm/swarm_gazebo_truth.cpp)
add_dependencies(swarm_gazebo_truth drone_swarm_gencpp)
target_link_libraries(swarm_gazebo_truth ${catkin_LIBRARIES})

##swarm_ground_station_msg.cpp
add_executable(swarm_ground_station_msg src/swarm/swarm_ground_station_msg.cpp)
add_dependencies(swarm_ground_station_msg drone_swarm_gencpp)
target_link_libraries(swarm_ground_station_msg ${catkin_LIBRARIES})
##swarm_trajectory.cpp
#add_executable(swarm_trajectory src/swarm/swarm_trajectory.cpp)
#add_dependencies(swarm_trajectory drone_swarm_gencpp)
#target_link_libraries(swarm_trajectory ${catkin_LIBRARIES})
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drone_swarm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
