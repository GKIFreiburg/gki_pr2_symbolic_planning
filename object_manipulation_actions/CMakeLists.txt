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
	ork_to_planning_scene
	ork_to_planning_scene_msgs
	symbolic_planning_utils
	object_surface_placements
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
	include
	${catkin_INCLUDE_DIRS}
) 

add_library(object_manipulation_actions
    src/actionExecutorArmToSide.cpp
    src/actionExecutorArmToFront.cpp
    src/actionExecutorPickupObject.cpp
    src/actionExecutorPutdownObject.cpp
    src/actionExecutorInspectLocation.cpp
    src/actionExecutorInspectObject.cpp
    src/actionExecutorLiftTorso.cpp
    src/actionExecutorInspectTable.cpp
    src/actionExecutorPickupObject.cpp
    src/actionExecutorPutdownObject.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_manipulation_actions
	tidyup_msgs_generate_messages_cpp
	ork_to_planning_scene_msgs_generate_messages_cpp
)

target_link_libraries(object_manipulation_actions
	${catkin_LIBRARIES}
)
