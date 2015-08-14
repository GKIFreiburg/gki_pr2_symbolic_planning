cmake_minimum_required(VERSION 2.8.3)
project(navigation_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
tf
pluginlib
actionlib
continual_planning_executive
move_base_msgs
tidyup_utils
planner_modules_pr2
tfd_modules
inverse_capability_map
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES navigation_actions
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
add_library(navigation_actions
    src/stateCreatorROSNavigation.cpp
    src/actionExecutorROSNavigation.cpp
    src/goalCreatorROSNavigation.cpp
    ### Grounding Stuff
    src/actionExecutorROSNavigationGrounding.cpp
)

target_link_libraries(navigation_actions
  ${catkin_LIBRARIES}
)
