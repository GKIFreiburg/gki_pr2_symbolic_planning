#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <ros/ros.h>
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "planner_modules_pr2/navigation_modules.h"

using namespace planner_modules_pr2;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_near_test");

	ros::NodeHandle nh;
	ros::Publisher error_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("/modules_test/failed_scene", 1, true);

	vector<geometry_msgs::Pose2D> robot_poses;
	{
		robot_poses.push_back(geometry_msgs::Pose2D());
		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
		robot_pose.x = 4.2;
		robot_pose.y = 6.85;
		robot_pose.theta = -M_PI_2;
	}
	{
		robot_poses.push_back(geometry_msgs::Pose2D());
		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
		robot_pose.x = 1.3;
		robot_pose.y = 6.35;
		robot_pose.theta = -M_PI;
	}
//	{
//		robot_poses.push_back(geometry_msgs::Pose2D());
//		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
//		robot_pose.x = 4.2;
//		robot_pose.y = 7.85;
//		robot_pose.theta = M_PI_2;
//	}
//	{
//		robot_poses.push_back(geometry_msgs::Pose2D());
//		geometry_msgs::Pose2D& robot_pose = robot_poses.back();
//		robot_pose.x = 3.7;
//		robot_pose.y = 7.85;
//		robot_pose.theta = M_PI_2;
//	}

	double torso_height = 0.2;

	::navigation_init(0, NULL);

	int test_count = 0;
	int success_count = 0;

	forEach(const geometry_msgs::Pose2D& start_pose, robot_poses)
	{
		forEach(const geometry_msgs::Pose2D& goal_pose, robot_poses)
		{
			if (hypot(start_pose.x-goal_pose.x, start_pose.y-goal_pose.y) < 0.0001)
				continue;
			TidyupPlanningSceneUpdaterPtr psu = TidyupPlanningSceneUpdater::instance();
			planning_scene::PlanningScenePtr scene = psu->getEmptyScene();
			psu->updateArmToSidePosition(scene, "right_arm");
			psu->updateArmToSidePosition(scene, "left_arm");
			psu->updateRobotPose2D(scene, start_pose, torso_height);
			//psu->visualize(scene);
			nav_msgs::GetPlan srv;
			srv.request.goal.pose.position.x = goal_pose.x;
			srv.request.goal.pose.position.y = goal_pose.y;
			srv.request.goal.pose.position.z = 0;
			srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose.theta);
			srv.request.goal.header.frame_id = scene->getPlanningFrame();
			test_count++;
			bool result = navigation::compute_value(scene, srv) < modules::INFINITE_COST;
			if (result != true)
			{
				ROS_ERROR_STREAM("navigation test failed...");
				ROS_ERROR_STREAM("start: \n"<<start_pose);
				ROS_ERROR_STREAM("goal: \n"<<goal_pose);
			}
			else
			{
				success_count++;
			}
		}
	}
	if (success_count==test_count)
	{
		ROS_INFO_STREAM(success_count<<" navigation tests successful.");
	}
	else
	{
		ROS_ERROR_STREAM("only "<<success_count<<" of "<<test_count<<" navigation tests successful!");
	}

	ros::spin();
	return 0;
}


