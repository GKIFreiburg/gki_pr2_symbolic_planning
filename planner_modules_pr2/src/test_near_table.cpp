#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <ros/ros.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "planner_modules_pr2/robot_near_modules.h"

using namespace planner_modules_pr2;

ros::Publisher error_scene_publisher;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_near_test");

	ros::NodeHandle nh;
	error_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("/modules_test/failed_scene", 1, true);

//	std::string object_name = "coke_1";
//	geometry_msgs::Pose object_pose;
//	object_pose.position.x = 4.45;
//	object_pose.position.y = 5.26;
//	object_pose.position.z = 1.0;
//	object_pose.orientation.w = 1;
//	psu->addObject(scene, object_name, object_pose);

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

	ros::spinOnce();

	vector<bool> expected_results;
	expected_results.push_back(true); //pose1 table1
	expected_results.push_back(false); //pose2 table1
	expected_results.push_back(false); //pose3 table1

	expected_results.push_back(false); //pose1 table2
	expected_results.push_back(true); //pose2 table2
	expected_results.push_back(false); //pose3 table2

	int test_id = 0;
	int failed_count = 0;
	for(size_t t = 0; t < table_names.size(); t++)
	{
		for(size_t r = 0; r < robot_poses.size(); r++)
		{
			for(size_t z = 0; z < torso_heights.size(); z++)
			{
				TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
				planning_scene::PlanningScenePtr scene = psu->getEmptyScene();
				psu->updateArmToSidePosition(scene, "right_arm");
				psu->updateArmToSidePosition(scene, "left_arm");
				psu->updateRobotPose2D(scene, robot_poses[r], torso_heights[z]);
				bool result = robot_near_table::compute_value(scene, table_names[t]) < modules::INFINITE_COST;
				bool expected_result = expected_results[t*robot_poses.size() + r];
				if (expected_result != result)
				{
					moveit_msgs::PlanningScene msg;
					scene->getPlanningSceneMsg(msg);
					error_scene_publisher.publish(msg);
					failed_count++;
					ROS_ERROR_STREAM(__PRETTY_FUNCTION__<<": test "<<test_id<<" failed!");
					sleep(1);
				}
				test_id++;
			}
		}
	}
	if (failed_count > 0)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__<<": "<<failed_count<<" of "<<test_id<<" tests failed!");
	}
	else
	{
		ROS_INFO_STREAM(__PRETTY_FUNCTION__<<": all "<<test_id<<" tests successfully completed.");
	}

	//ros::spinOnce();
	return 0;
}


