/*
 * tidyup_planning_scene_updater.cpp
 *
 *  Created on: 10 Aug 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
//#include "tidyup_utils/planning_scene_interface.h"
//#include "tidyup_utils/arm_state.h"
#include "tidyup_utils/stringutil.h"
//#include "tidyup_utils/geometryPoses.h"
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

using std::vector;
using std::map;
using std::set;
using std::string;

TidyupPlanningSceneUpdater* TidyupPlanningSceneUpdater::instance = NULL;

TidyupPlanningSceneUpdater::TidyupPlanningSceneUpdater() :
		logName("[psu]")
{
//    defaultAttachPose.position.x = 0.032;
//    defaultAttachPose.position.y = 0.015;
//    defaultAttachPose.position.z = 0.0;
//    defaultAttachPose.orientation.x = 0.707;
//    defaultAttachPose.orientation.y = -0.106;
//    defaultAttachPose.orientation.z = -0.690;
//    defaultAttachPose.orientation.w = 0.105;

	scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

	ROS_INFO("%s initialized.\n", logName.c_str());
}

TidyupPlanningSceneUpdater::~TidyupPlanningSceneUpdater()
{
}

bool TidyupPlanningSceneUpdater::readState(const string& robotLocation, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, geometry_msgs::Pose& robotPose, map<string, geometry_msgs::Pose>& movableObjects, GraspedObjectMap& graspedObjects,
		map<string, string>& objectsOnStatic)
{
	if (instance == NULL)
		instance = new TidyupPlanningSceneUpdater();
	return instance->readState_(robotLocation, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic);
}

bool TidyupPlanningSceneUpdater::readState_(const string& robotLocation, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, geometry_msgs::Pose& robotPose, map<string, geometry_msgs::Pose>& movableObjects, GraspedObjectMap& graspedObjects,
		map<string, string>& objectsOnStatic)
{
	// get poses of all movable objects
	scene_monitor->requestPlanningSceneState("/get_planning_scene");
	planning_scene::PlanningScenePtr scene = scene_monitor->getPlanningScene();
	collision_detection::WorldConstPtr world = scene->getWorld();
	geometry_msgs::Pose pose;
	for (collision_detection::World::const_iterator it = world->begin(); it != world->end(); it++)
	{
		const string& objectName = it->first;
		if (StringUtil::startsWith(objectName, "table"))
			continue;
		if (StringUtil::startsWith(objectName, "sponge"))
			continue;
		if (fillPoseFromState_(pose, objectName, numericalFluentCallback))
		{
			movableObjects.insert(make_pair(objectName, pose));
		}
	}
	vector<const moveit::core::AttachedBody*> attachedObjects;
	scene->getCurrentState().getAttachedBodies(attachedObjects);
	for (vector<const moveit::core::AttachedBody*>::iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
	{
		const string& objectName = (*it)->getName();
		if (StringUtil::startsWith(objectName, "table"))
			continue;
		if (StringUtil::startsWith(objectName, "sponge"))
			continue;
		if (fillPoseFromState_(pose, objectName, numericalFluentCallback))
		{
			movableObjects.insert(make_pair(objectName, pose));
		}
	}

	// find grasped objects and objects on tables
	PredicateList* predicates = NULL;
	if (!predicateCallback(predicates))
	{
		ROS_ERROR("%s predicateCallback failed.", logName.c_str());
		return false;
	}
	ROS_ASSERT(predicates != NULL);
	for (PredicateList::iterator it = predicates->begin(); it != predicates->end(); it++)
	{
		Predicate p = *it;
		if (!p.value)
			continue;
		if (p.name == "on")
		{
			ROS_ASSERT(p.parameters.size() == 2);
			// (on movable static)
			objectsOnStatic.insert(make_pair(p.parameters.front().value, p.parameters.back().value));
		}
		else if (p.name == "grasped")
		{
			ROS_ASSERT(p.parameters.size() == 2);
			// (grasped object arm)
			string objectName = p.parameters.front().value;
			ROS_ASSERT(fillPoseFromState_(pose, objectName, numericalFluentCallback));
			graspedObjects.insert(make_pair(objectName, make_pair(p.parameters.back().value, pose)));
		}
	}

	// Robot pose
	if (!fillPoseFromState_(robotPose, robotLocation, numericalFluentCallback))
	{
		ROS_ERROR("%s get robot location failed.", logName.c_str());
	}
	return true;
}

bool TidyupPlanningSceneUpdater::update(const geometry_msgs::Pose& robotPose, const map<string, geometry_msgs::Pose>& movableObjects, const GraspedObjectMap& graspedObjects)
{
	if (instance == NULL)
		instance = new TidyupPlanningSceneUpdater();
	return instance->update_(robotPose, movableObjects, graspedObjects);
}

bool TidyupPlanningSceneUpdater::update_(const geometry_msgs::Pose& robotPose,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const GraspedObjectMap& graspedObjects)
{
	planning_scene::PlanningScenePtr scene = scene_monitor->getPlanningScene();
	collision_detection::WorldPtr world = scene->getWorldNonConst();

	// set robot state in planning scene
	ROS_INFO("%s update robot state in planning scene", logName.c_str());
	tf::Pose robotPoseTf;
	Eigen::Affine3d robotPoseEigen;
	tf::poseMsgToTF(robotPose, robotPoseTf);
	tf::transformTFToEigen(robotPoseTf, robotPoseEigen);
	scene->getTransformsNonConst().setTransform(robotPoseEigen, "/base_footprint");
	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	scene->setCurrentState(robot_state);
	// TODO: set default arm state
//    ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(state.joint_state);
//    ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(state.joint_state);

// update pose of movalbe object in the planning scene
	for (map<string, geometry_msgs::Pose>::const_iterator movabelObjectIt = movableObjects.begin(); movabelObjectIt != movableObjects.end(); movabelObjectIt++)
	{
		string object_name = movabelObjectIt->first;
		ROS_INFO("%s updating object %s", logName.c_str(), object_name.c_str());
		const moveit::core::AttachedBody* attachedObject = scene->getCurrentStateNonConst().getAttachedBody(object_name);
		collision_detection::World::ObjectConstPtr object = world->getObject(object_name);
		tf::Pose object_pose_tf;
		Eigen::Affine3d object_pose_eigen;
		tf::poseMsgToTF(movabelObjectIt->second, object_pose_tf);
		tf::transformTFToEigen(object_pose_tf, object_pose_eigen);
		if (attachedObject != NULL)
		{
			// if this object is attached somewhere we need to detach it
			scene->getCurrentStateNonConst().clearAttachedBody(movabelObjectIt->first);
			world->addToObject(object_name, attachedObject->getShapes().front(), object_pose_eigen);
		}
		else if (object != NULL)
		{
			// object is not attached, update pose
			world->moveShapeInObject(object_name, object->shapes_.front(), object_pose_eigen);
		}
		else
		{
			ROS_ERROR("%s object %s does not exist in planning scene.", logName.c_str(), object_name.c_str());
			return false;
		}
	}

    // attach object to the correct arm
//    const moveit_msgs::CollisionObject* object = psi->getCollisionObject(request.putdown_object);
    for (GraspedObjectMap::const_iterator graspedIt = graspedObjects.begin(); graspedIt != graspedObjects.end(); graspedIt++)
    {
        const string& objectName = graspedIt->first;
        const string& arm = graspedIt->second.first;
        ROS_INFO("%s attaching object %s to arm %s", logName.c_str(), objectName.c_str(), arm.c_str());
//        ArmState::get("/arm_configurations/side_tuck/position/", arm).replaceJointPositions(state.joint_state);
//        psi->attachObjectToGripper(objectName, arm);
//        psi->updateObject(objectName, graspedIt->second.second);
    }
//    psi->setRobotState(state);
    return true;
}

bool TidyupPlanningSceneUpdater::fillPointFromState(geometry_msgs::Point& point,
        const string& poseName,
        numericalFluentCallbackType numericalFluentCallback)
{
    if (instance == NULL) instance = new TidyupPlanningSceneUpdater();
    return instance->fillPointFromState_(point, poseName, numericalFluentCallback);
}

bool TidyupPlanningSceneUpdater::fillPointFromState_(geometry_msgs::Point& point,
        const string& poseName,
        numericalFluentCallbackType numericalFluentCallback)
{
    // create the numerical fluent request
    ParameterList startParams;
    startParams.push_back(Parameter("", "", poseName));
    NumericalFluentList nfRequest;
    nfRequest.reserve(3);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));

    // get the fluents
    NumericalFluentList* nfRequestP = &nfRequest;
    if (!numericalFluentCallback(nfRequestP))
    {
        ROS_ERROR("fillPoseFromState failed for object: %s", poseName.c_str());
        return false;
    }

    // fill pose stamped
    point.x = nfRequest[0].value;
    point.y = nfRequest[1].value;
    point.z = nfRequest[2].value;
    return true;
}

bool TidyupPlanningSceneUpdater::fillPoseFromState(geometry_msgs::Pose& pose,
        const string& poseName,
        numericalFluentCallbackType numericalFluentCallback)
{
    if (instance == NULL) instance = new TidyupPlanningSceneUpdater();
    return instance->fillPoseFromState_(pose, poseName, numericalFluentCallback);
}

bool TidyupPlanningSceneUpdater::fillPoseFromState_(geometry_msgs::Pose& pose,
        const string& poseName,
        numericalFluentCallbackType numericalFluentCallback)
{
    // create the numerical fluent request
    ParameterList startParams;
    startParams.push_back(Parameter("", "", poseName));
    NumericalFluentList nfRequest;
    nfRequest.reserve(7);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));

    // get the fluents
    NumericalFluentList* nfRequestP = &nfRequest;
    if (!numericalFluentCallback(nfRequestP))
    {
        ROS_ERROR("fillPoseFromState failed for object: %s", poseName.c_str());
        return false;
    }

    // fill pose stamped
    pose.position.x = nfRequest[0].value;
    pose.position.y = nfRequest[1].value;
    pose.position.z = nfRequest[2].value;
    pose.orientation.x = nfRequest[3].value;
    pose.orientation.y = nfRequest[4].value;
    pose.orientation.z = nfRequest[5].value;
    pose.orientation.w = nfRequest[6].value;
    return true;
}

