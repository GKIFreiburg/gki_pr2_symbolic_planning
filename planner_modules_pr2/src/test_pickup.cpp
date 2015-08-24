/*
 * test_pickup.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */
// this header needs to be included first!!
#include "planner_modules_pr2/manipulation_planning.h"


#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shapes.h>
#include <tf_conversions/tf_eigen.h>
#include "planner_modules_pr2/manipulation_exceptions.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manipualion_pickup_test");

	planning_scene_monitor::PlanningSceneMonitorPtr monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	ROS_INFO_STREAM("requesting planning scene");
	monitor->requestPlanningSceneState("/get_planning_scene");
	ROS_INFO_STREAM("creating diff");
	planning_scene::PlanningScenePtr scene = monitor->getPlanningScene()->diff();
	ROS_INFO_STREAM("decouple scene");
	scene->decoupleParent();

	ros::NodeHandle nh;
	ros::Publisher initial_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("pickup_initial_scene", 1, true);
	ros::Publisher final_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("pickup_final_scene", 1, true);

	std::string object_name = "coke_1";
	ROS_INFO_STREAM("adding object: "<<object_name);
	collision_detection::WorldPtr world = scene->getWorldNonConst();
	Eigen::Affine3d pose_eigen;
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = 4.45;
	pose_msg.position.y = 5.26;
	pose_msg.position.z = 1.0;
	pose_msg.orientation.w = 1;
	tf::poseMsgToEigen(pose_msg, pose_eigen);
	shapes::ShapePtr shape (new shapes::Cylinder(0.025, 0.1));
	world->addToObject(object_name, shape, pose_eigen);
	ROS_INFO_STREAM("TODO: setting arm joints");
	robot_state::RobotState& state = scene->getCurrentStateNonConst();
	state.setToRandomPositions();

	ROS_INFO_STREAM("TODO: setting spine joint");
	state.setVariablePosition("torso_lift_joint", 0.25);

	ROS_INFO_STREAM("moving robot to table");
	state.setVariablePosition("world_joint/x", 4.7);
	state.setVariablePosition("world_joint/y", 5.85);
	state.setVariablePosition("world_joint/theta", -M_PI_2);

	ROS_INFO_STREAM("scene initialized.");
	moveit_msgs::PlanningScene msg;
	scene->getPlanningSceneMsg(msg);
	initial_scene_publisher.publish(msg);

	try
	{
		planner_modules_pr2::ManipulationPlanningPtr p = planner_modules_pr2::ManipulationPlanning::instance();
		ROS_INFO_STREAM("trying pickup");
		p->pickup(scene, object_name, "right", "table1");
		moveit_msgs::PlanningScene msg;
		scene->getPlanningSceneMsg(msg);
		final_scene_publisher.publish(msg);
	}
	catch (planner_modules_pr2::ManipulationException& ex)
	{
		ROS_ERROR("Pickup failed: %s", ex.what());
	}
	catch (std::exception& ex)
	{
		ROS_ERROR("Error: %s", ex.what());
	}

	ros::spin();
	return 0;
}


