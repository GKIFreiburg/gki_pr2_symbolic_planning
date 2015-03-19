cmake_minimum_required(VERSION 2.8.3)
project(tidyup_state_creators)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  std_srvs
  continual_planning_executive
  tidyup_utils
  visualization_msgs
  tidyup_msgs
  geometry_msgs
  #state_transformer
)

find_package(GTest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tidyup_state_creators
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)
 
add_library(tidyup_state_creators
    src/stateCreatorArmsAtSide.cpp
    src/stateCreatorObjectReachable.cpp
    src/stateCreatorRobotPose.cpp
    src/stateCreatorFromPlanningScene.cpp
    src/goalCreatorResetWorld.cpp
    src/goalCreatorTidyupInitialize.cpp
    src/goalCreatorLoadTablesIntoPlanningScene.cpp
    src/goalCreatorFromPlanningScene.cpp
    src/goalCreatorMoveObjectsToTable.cpp
    src/robotPoseVisualization.cpp
    src/StateAccessException.cpp)
    
target_link_libraries(tidyup_state_creators
  ${catkin_LIBRARIES}
)

# Link runTests with what we want to test and the GTest and pthread library
#catkin_add_gtest(simple_test test/simple_test.cpp)
#target_link_libraries(simple_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)

if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}-test test/stateCreatorFromPlanningSceneTest.cpp src/stateCreatorFromPlanningScene.cpp)
	target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
endif()
