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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace modules;

typedef std::map<std::string, std::pair<std::string, geometry_msgs::Pose> > GraspedObjectMap;

class TidyupPlanningSceneUpdater
{
public:
	virtual ~TidyupPlanningSceneUpdater();

	static bool readState(const string& robotLocation, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, geometry_msgs::Pose& robotPose, std::map<std::string, geometry_msgs::Pose>& movableObjects, GraspedObjectMap& graspedObjects,
			std::map<std::string, std::string>& objectsOnStatic);

	static bool update(const geometry_msgs::Pose& robotPose, const std::map<std::string, geometry_msgs::Pose>& movableObjects, const GraspedObjectMap& graspedObjects);

	static bool fillPoseFromState(geometry_msgs::Pose& pose, const std::string& poseName, numericalFluentCallbackType numericalFluentCallback);

	static bool fillPointFromState(geometry_msgs::Point& point, const string& poseName, numericalFluentCallbackType numericalFluentCallback);

private:
	TidyupPlanningSceneUpdater();
	bool readState_(const string& robotLocation, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, geometry_msgs::Pose& robotPose, std::map<std::string, geometry_msgs::Pose>& movableObjects, GraspedObjectMap& graspedObjects,
			std::map<std::string, std::string>& objectsOnStatic);
	bool update_(const geometry_msgs::Pose& robotPose, const std::map<std::string, geometry_msgs::Pose>& movableObjects, const GraspedObjectMap& graspedObjects);
	bool fillPoseFromState_(geometry_msgs::Pose& pose, const std::string& poseName, numericalFluentCallbackType numericalFluentCallback);
	bool fillPointFromState_(geometry_msgs::Point& point, const string& poseName, numericalFluentCallbackType numericalFluentCallback);

	std::string logName;
//    geometry_msgs::Pose defaultAttachPose;
	planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;

	static TidyupPlanningSceneUpdater* instance;

};

#endif /* TIDYUP_PLANNING_SCENE_UPDATER_H_ */
