cmake_minimum_required(VERSION 2.8.3)
project(object_manipulation_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	tidyup_msgs
	moveit_msgs
	moveit_core
	moveit_ros_planning_interface
	visualization_msgs
	std_srvs
	tidyup_utils
	pluginlib
	actionlib
	continual_planning_executive
	tfd_modules
	#tidyup # TODO move arms-to-side to tidyup utils
	#state_transformer
	#pr2_controller_msgs
)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES object_manipulation_actions
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
) 

add_library(object_manipulation_actions
#    src/actionExecutorArmToSideInterface.cpp
    src/actionExecutorArmToSideActionServer.cpp
    src/actionExecutorArmToInspectObject.cpp
    src/actionExecutorArmToCarry.cpp
    src/actionExecutorPickupObject.cpp
    src/actionExecutorPutdownObject.cpp
    src/actionExecutorDetectObjects.cpp
    src/actionExecutorDetectExpectedObjects.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_manipulation_actions tidyup_msgs_generate_messages_cpp)

target_link_libraries(object_manipulation_actions
	${catkin_LIBRARIES}
)
