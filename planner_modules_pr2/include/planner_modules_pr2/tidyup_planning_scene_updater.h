/*
 * tidyup_planning_scene_updater.h
 *
 *  Created on: 10 Aug 2012
 *      Author: andreas
 */

#ifndef TIDYUP_PLANNING_SCENE_UPDATER_H_
#define TIDYUP_PLANNING_SCENE_UPDATER_H_

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <map>
#include <set>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace modules;

typedef std::map<std::string, std::pair<std::string, geometry_msgs::Pose> > GraspedObjectMap;
typedef std::map<std::string, geometry_msgs::Pose> MovableObjectsMap;
typedef std::map<std::string, std::string> ObjectsOnTablesMap;

class TidyupPlanningSceneUpdater
{
public:
	static TidyupPlanningSceneUpdater* instance();
	virtual ~TidyupPlanningSceneUpdater();

	bool readObjects(
			predicateCallbackType predicateCallback,
			numericalFluentCallbackType numericalFluentCallback,
			MovableObjectsMap& movableObjects,
			GraspedObjectMap& graspedObjects,
			ObjectsOnTablesMap& objectsOnStatic);

	bool readRobotPose2D(
			geometry_msgs::Pose2D& robot_pose,
			double& torso_position,
			modules::numericalFluentCallbackType numericalFluentCallback);

	bool readPose(
			geometry_msgs::Pose& pose,
			const std::string& poseName,
			numericalFluentCallbackType numericalFluentCallback);

	bool readPoint(
			geometry_msgs::Point& point,
			const string& poseName,
			numericalFluentCallbackType numericalFluentCallback);

	planning_scene::PlanningScenePtr getEmptyScene();

	void updateRobotPose2D(planning_scene::PlanningScenePtr scene,
			const geometry_msgs::Pose& robot_pose,
			const double torso_position);
	void updateRobotPose2D(
			planning_scene::PlanningScenePtr scene,
			const geometry_msgs::Pose2D& robot_pose,
			const double torso_position);

	void updateObjects(
			planning_scene::PlanningScenePtr scene,
			const MovableObjectsMap& movableObjects,
			const GraspedObjectMap& graspedObjects);

	void visualize(planning_scene::PlanningScenePtr scene);

private:
	TidyupPlanningSceneUpdater();

	void attachObject(
			const string& arm_prefix,
			const string& object,
			const std::vector<shapes::ShapeConstPtr>& shapes,
			const geometry_msgs::Pose& grasp,
			robot_state::RobotState& robot_state);

	void setArmJointsToSidePosition(const std::string& arm, planning_scene::PlanningScenePtr scene);

	std::string logName;
	geometry_msgs::Pose defaultAttachPose;
	planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;

	static TidyupPlanningSceneUpdater* instance_;

	ros::Publisher scene_publisher;

};

#endif /* TIDYUP_PLANNING_SCENE_UPDATER_H_ */
