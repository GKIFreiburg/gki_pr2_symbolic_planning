/*
 * test_pickup.cpp
 *
 *  Created on: Jul 29, 2015
 *      Author: andreas
 */
// this header needs to be included first!!
#include "planner_modules_pr2/manipulation_planning.h"


#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <ros/ros.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "planner_modules_pr2/manipulation_exceptions.h"

using namespace planner_modules_pr2;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pickup_test");

	ros::NodeHandle nh;
	ros::Publisher error_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("/modules_test/failed_scene", 1, true);

	TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();

	std::string object_name = "coke_1";
	geometry_msgs::Pose object_pose;
	object_pose.position.x = 4.45;
	object_pose.position.y = 5.26;
	object_pose.position.z = 1.0;
	object_pose.orientation.w = 1;

	//psu->addObject(scene, object_name, object_pose);

	vector<geometry_msgs::Pose2D> robot_poses;
	{
		robot_poses.push_back(geometry_msgs::Pose2D());
		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
		robot_pose.x = 4.7;
		robot_pose.y = 5.85;
		robot_pose.theta = -M_PI_2;
	}
	{
		robot_poses.push_back(geometry_msgs::Pose2D());
		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
		robot_pose.x = 1.3;
		robot_pose.y = 6.35;
		robot_pose.theta = -M_PI;
	}
	{
		robot_poses.push_back(geometry_msgs::Pose2D());
		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
		robot_pose.x = 4.7;
		robot_pose.y = 7.85;
		robot_pose.theta = M_PI_2;
	}
	vector<double> torso_heights;
	for (double z = 0; z < 0.305; z += 0.1)
	{
		torso_heights.push_back(z);
	}
	vector<string> table_names;
	table_names.push_back("table1");
	table_names.push_back("table2");

	// TODO: write module tests
	vector<geometry_msgs::Pose> object_poses;
	ros::spinOnce();


	ros::spin();
	return 0;
}


