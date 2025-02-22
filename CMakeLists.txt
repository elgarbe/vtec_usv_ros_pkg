cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy 
std_msgs 
genmsg
sensor_msgs
tf2
tf2_ros
tf2_geometry_msgs
#gazebo_ros
)

#Eigen libs
#find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  DEPENDS eigen
#  INCLUDE_DIRS include
#  LIBRARIES sensors
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#include
 ${catkin_INCLUDE_DIRS}
 include/cpp/include
 ${EIGEN_3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sensors.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sensors_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sensors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_subdirectory(include/cpp libvncxx)
add_executable(ins_2d src/ins_2d.cpp)
add_executable(asmc src/asmc.cpp)
add_executable(aitst src/aitst.cpp)
add_executable(tracking_los src/tracking_los.cpp)
add_executable(tracking_aitsmc src/tracking_aitsmc.cpp)
add_executable(tracking_asmc src/tracking_asmc.cpp)
add_executable(tracking_finite_time_asmc src/tracking_finite_time_asmc.cpp)
add_executable(tracking_aitst src/tracking_aitst.cpp)
add_executable(tracking_ast src/tracking_ast.cpp)
add_executable(tracking_sast2 src/tracking_sast2.cpp)
add_executable(tracking_ismc src/tracking_ismc.cpp)
add_executable(tracking_control_asmc src/tracking_control_asmc.cpp)
add_executable(tracking_control_antsmc src/tracking_control_antsmc.cpp)
add_executable(tracking_control_sast src/tracking_control_sast.cpp)
add_executable(tracking_control_sast2 src/tracking_control_sast2.cpp)
add_executable(tracking_control_sast3 src/tracking_control_sast3.cpp)
add_executable(tracking_control_sst src/tracking_control_sst.cpp)
add_executable(tracking_control_supertwisting src/tracking_control_supertwisting.cpp)
add_executable(tracking_control_ast src/tracking_control_ast.cpp)
add_executable(tracking_control_pid src/tracking_control_pid.cpp)
add_executable(tracking_control_fxteso_antsm src/tracking_control_fxteso_antsm.cpp)
add_executable(finite_time_tracking_control_asmc src/finite_time_tracking_control_asmc.cpp)
add_executable(asmc_speeds src/asmc_speeds.cpp)
add_executable(dynamic_model_simulate src/dynamic_model_simulate.cpp)
add_executable(dynamic_uncertain_model src/dynamic_uncertain_model.cpp)
add_executable(boat_tf2_broadcaster src/boat_tf2_broadcaster.cpp)
add_executable(velodyne2boat_tf2_broadcaster src/velodyne2boat_tf2_broadcaster.cpp)
add_executable(zed2boat_tf2_broadcaster src/zed2boat_tf2_broadcaster.cpp)
add_executable(waypoints2path src/waypoints2path.cpp)
add_executable(traj2path src/traj2path.cpp)
add_executable(pid src/pid.cpp)
add_executable(pid_fl src/pid_fl.cpp)
add_executable(adaptive_super_twisting src/adaptive_super_twisting.cpp)
add_executable(super_twisting src/super_twisting.cpp)
#add_executable(boat_gazebo_broadcaster src/boat_gazebo_broadcaster.cpp)
add_executable(finite_time_asmc src/finite_time_asmc.cpp)
add_executable(obs_finite_time_asmc src/obs_finite_time_asmc.cpp)
add_executable(simpl_adaptive_super_twisting src/simpl_adaptive_super_twisting.cpp)
target_link_libraries(ins_2d ${catkin_LIBRARIES})
target_link_libraries(ins_2d PRIVATE libvncxx)
target_link_libraries(ins_2d LINK_PUBLIC pthread)
target_link_libraries(asmc ${catkin_LIBRARIES})
target_link_libraries(aitst ${catkin_LIBRARIES})
target_link_libraries(tracking_los ${catkin_LIBRARIES})
target_link_libraries(tracking_aitsmc ${catkin_LIBRARIES})
target_link_libraries(tracking_asmc ${catkin_LIBRARIES})
target_link_libraries(tracking_finite_time_asmc ${catkin_LIBRARIES})
target_link_libraries(tracking_aitst ${catkin_LIBRARIES})
target_link_libraries(tracking_ast ${catkin_LIBRARIES})
target_link_libraries(tracking_sast2 ${catkin_LIBRARIES})
target_link_libraries(tracking_control_asmc ${catkin_LIBRARIES})
target_link_libraries(tracking_control_antsmc ${catkin_LIBRARIES})
target_link_libraries(tracking_control_sast ${catkin_LIBRARIES})
target_link_libraries(tracking_control_sast2 ${catkin_LIBRARIES})
target_link_libraries(tracking_control_sast3 ${catkin_LIBRARIES})
target_link_libraries(tracking_control_sst ${catkin_LIBRARIES})
target_link_libraries(tracking_control_supertwisting ${catkin_LIBRARIES})
target_link_libraries(tracking_control_ast ${catkin_LIBRARIES})
target_link_libraries(tracking_control_pid ${catkin_LIBRARIES})
target_link_libraries(tracking_control_fxteso_antsm ${catkin_LIBRARIES})
target_link_libraries(finite_time_tracking_control_asmc ${catkin_LIBRARIES})
target_link_libraries(tracking_ismc ${catkin_LIBRARIES})
target_link_libraries(asmc_speeds ${catkin_LIBRARIES})
target_link_libraries(dynamic_model_simulate ${catkin_LIBRARIES})
target_link_libraries(dynamic_uncertain_model ${catkin_LIBRARIES})
target_link_libraries(boat_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(velodyne2boat_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(zed2boat_tf2_broadcaster ${catkin_LIBRARIES})
target_link_libraries(waypoints2path ${catkin_LIBRARIES})
target_link_libraries(traj2path ${catkin_LIBRARIES})
target_link_libraries(pid ${catkin_LIBRARIES})
target_link_libraries(pid_fl ${catkin_LIBRARIES})
target_link_libraries(adaptive_super_twisting ${catkin_LIBRARIES})
target_link_libraries(super_twisting ${catkin_LIBRARIES})
#target_link_libraries(boat_gazebo_broadcaster ${catkin_LIBRARIES})
target_link_libraries(finite_time_asmc ${catkin_LIBRARIES})
target_link_libraries(obs_finite_time_asmc ${catkin_LIBRARIES})
target_link_libraries(simpl_adaptive_super_twisting ${catkin_LIBRARIES})