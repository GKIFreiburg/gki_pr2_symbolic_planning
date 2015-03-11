cmake_minimum_required(VERSION 2.8.3)
project(planner_modules_pr2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  continual_planning_msgs
  moveit_msgs
  nav_msgs
  pluginlib
  tf
  tfd_modules
  tidyup_msgs
  tidyup_utils
)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
#   continual_planning_msgs#   moveit_msgs#   nav_msgs#   tidyup_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_modules_pr2
#  CATKIN_DEPENDS continual_planning_msgs moveit_msgs nav_msgs planner_benchmarks pluginlib tf tfd_modules tidyup_msgs tidyup_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(planner_modules_pr2
#   src/${PROJECT_NAME}/planner_modules_pr2.cpp
# )

add_library(planner_modules_pr2
    src/module_param_cache.cpp
    src/caching_evaluation.cpp
    src/navstack_module.cpp
    src/navstack_planning_scene_module.cpp
    src/navstack_module_full_body.cpp
    src/tidyup_planning_scene_updater.cpp)

add_library(putdown_modules
    src/module_param_cache.cpp
    src/caching_evaluation.cpp
    src/putdown_modules.cpp
    src/subsumption_caching.cpp
    src/tidyup_planning_scene_updater.cpp)

#add_library(pickup_modules
#    src/module_param_cache.cpp
#    src/caching_evaluation.cpp
#    src/pickup_modules.cpp
#    src/subsumption_caching.cpp
#    src/tidyup_planning_scene_updater.cpp)

#rosbuild_find_ros_package(continual_planning_executive)
#add_executable(precache_navigation
#    src/precache_navigation.cpp
#    ${continual_planning_executive_PACKAGE_PATH}/src/symbolicState.cpp)
#target_link_libraries(precache_navigation planner_modules_pr2)

target_link_libraries(planner_modules_pr2 tidyup_utils)
target_link_libraries(putdown_modules tidyup_utils)

## Declare a cpp executable
# add_executable(planner_modules_pr2_node src/planner_modules_pr2_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(planner_modules_pr2_node planner_modules_pr2_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(planner_modules_pr2_node
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
# install(TARGETS planner_modules_pr2 planner_modules_pr2_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_planner_modules_pr2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)