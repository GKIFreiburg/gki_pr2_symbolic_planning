/*
 * tidyup_planning_scene_updater.cpp
 *
 *  Created on: 10 Aug 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include "tidyup_utils/stringutil.h"
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose2D.h>

using std::vector;
using std::map;
using std::set;
using std::string;

namespace planner_modules_pr2
{

TidyupPlanningSceneUpdaterPtr TidyupPlanningSceneUpdater::instance_;

TidyupPlanningSceneUpdaterPtr TidyupPlanningSceneUpdater::instance()
{
	if (instance_ == NULL)
		instance_.reset(new TidyupPlanningSceneUpdater());
	return instance_;
}

TidyupPlanningSceneUpdater::TidyupPlanningSceneUpdater() :
		logName("[psu]")
{
	defaultAttachPose.position.x = 0.032;
	defaultAttachPose.position.y = 0.015;
	defaultAttachPose.position.z = 0.0;
	defaultAttachPose.orientation.x = 0.707;
	defaultAttachPose.orientation.y = -0.106;
	defaultAttachPose.orientation.z = -0.690;
	defaultAttachPose.orientation.w = 0.105;

	scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	scene_monitor->requestPlanningSceneState("/get_planning_scene");

	ros::NodeHandle nh("~");
	scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("tfd_planning/planning_scene", 1, true);

	ROS_INFO_STREAM(logName<<": initialized.");
}

TidyupPlanningSceneUpdater::~TidyupPlanningSceneUpdater()
{
}

bool TidyupPlanningSceneUpdater::readObjects(
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback,
		MovableObjectsMap& movableObjects,
		GraspedObjectMap& graspedObjects,
		ObjectsOnTablesMap& objectsOnStatic)
{
	// get poses of all movable objects
	planning_scene::PlanningScenePtr scene = scene_monitor->getPlanningScene();
	collision_detection::WorldConstPtr world = scene->getWorld();
	geometry_msgs::Pose pose;
	for (collision_detection::World::const_iterator it = world->begin(); it != world->end(); it++)
	{
		const string& objectName = it->first;
		// first object in world is "<octomap>" which should be skipped
		if (objectName.find("octomap") != std::string::npos)
			continue;
		if (StringUtil::startsWith(objectName, "table"))
			continue;
		if (StringUtil::startsWith(objectName, "sponge"))
			continue;
		if (readPose(pose, objectName, numericalFluentCallback))
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
		if (readPose(pose, objectName, numericalFluentCallback))
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
			ROS_ASSERT(readPose(pose, objectName, numericalFluentCallback));
			graspedObjects.insert(make_pair(objectName, make_pair(p.parameters.back().value, pose)));
		}
	}
	return true;
}

const string& TidyupPlanningSceneUpdater::getPlanningFrame()
{
	return scene_monitor->getPlanningScene()->getPlanningFrame();
}

planning_scene::PlanningScenePtr TidyupPlanningSceneUpdater::getEmptyScene()
{
	return scene_monitor->getPlanningScene();
}

planning_scene::PlanningScenePtr TidyupPlanningSceneUpdater::getCurrentScene(
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback)
{
	planning_scene::PlanningScenePtr scene = getEmptyScene();
	geometry_msgs::Pose2D robot_pose;
	double torso_position;
	readRobotPose2D(robot_pose, torso_position, numericalFluentCallback);
	updateRobotPose2D(scene, robot_pose, torso_position);
	updateArmToSidePosition(scene, "right_arm");
	updateArmToSidePosition(scene, "left_arm");
	MovableObjectsMap objects;
	GraspedObjectMap grasped;
	ObjectsOnTablesMap onTables;
	readObjects(predicateCallback, numericalFluentCallback, objects, grasped, onTables);
	updateObjects(scene, objects, grasped);
	return scene;
}

void TidyupPlanningSceneUpdater::updateRobotPose2D(planning_scene::PlanningScenePtr scene,
		const geometry_msgs::Pose& robot_pose,
		const double torso_position)
{
	geometry_msgs::Pose2D pose;
	pose.x = robot_pose.position.x;
	pose.y = robot_pose.position.y;
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(robot_pose.orientation, orientation);
	pose.theta = tf::getYaw(orientation);
	updateRobotPose2D(scene, pose, torso_position);
}

void TidyupPlanningSceneUpdater::updateRobotPose2D(planning_scene::PlanningScenePtr scene,
		const geometry_msgs::Pose2D& robot_pose,
		const double torso_position)
{
	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	robot_state.setVariablePosition("world_joint/x", robot_pose.x);
	robot_state.setVariablePosition("world_joint/y", robot_pose.y);
	robot_state.setVariablePosition("world_joint/theta", robot_pose.theta);
	robot_state.setVariablePosition("torso_lift_joint", torso_position);
	robot_state.update();
}

void TidyupPlanningSceneUpdater::updateArmToSidePosition(planning_scene::PlanningScenePtr scene, const std::string& arm)
{
	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	const moveit::core::JointModelGroup* model_group = robot_state.getJointModelGroup(arm);
	std::map< std::string, double > values;
	// right_arm_to_side is a group defined in pr2.srdf (tidyup_pr2_moveit_config)
	if (!model_group->getVariableDefaultPositions(arm+"_to_side", values))
	{
		ROS_WARN("%s: Could not set positions for %s to side!", __func__, arm.c_str());
	}
	std::map< std::string, double >::iterator it;
	for (it = values.begin(); it != values.end(); it++)
	{
		// ROS_INFO_STREAM(it->first << " " << it->second);
		robot_state.setJointPositions(it->first, &it->second);
	}
}

void TidyupPlanningSceneUpdater::updateObjects(
		planning_scene::PlanningScenePtr scene,
		const MovableObjectsMap& movableObjects,
		const GraspedObjectMap& graspedObjects)
{
	collision_detection::WorldPtr world = scene->getWorldNonConst();

	// update pose of movalbe object in the planning scene
	for (map<string, geometry_msgs::Pose>::const_iterator movabelObjectIt = movableObjects.begin(); movabelObjectIt != movableObjects.end(); movabelObjectIt++)
	{
		string object_name = movabelObjectIt->first;
		ROS_INFO("%s updating object %s", logName.c_str(), object_name.c_str());
		bool is_attached = scene->getCurrentStateNonConst().hasAttachedBody(object_name);
		bool exist_in_world = world->hasObject(object_name);
		tf::Pose object_pose_tf;
		Eigen::Affine3d object_pose_eigen;
		tf::poseMsgToTF(movabelObjectIt->second, object_pose_tf);
		tf::transformTFToEigen(object_pose_tf, object_pose_eigen);
		if (is_attached)
		{
			// if this object is attached somewhere we need to detach it
			const moveit::core::AttachedBody* attachedObject = scene->getCurrentStateNonConst().getAttachedBody(object_name);
			scene->getCurrentStateNonConst().clearAttachedBody(movabelObjectIt->first);
			world->addToObject(object_name, attachedObject->getShapes().front(), object_pose_eigen);
		}
		else if (exist_in_world)
		{
			// object is not attached, update pose
			collision_detection::World::ObjectConstPtr object = world->getObject(object_name);
			world->moveShapeInObject(object_name, object->shapes_.front(), object_pose_eigen);
		}
		else
		{
			ROS_ERROR("%s object %s does not exist in planning scene.", logName.c_str(), object_name.c_str());
		}
	}

	// attach object to the correct arm
	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
	for (GraspedObjectMap::const_iterator graspedIt = graspedObjects.begin(); graspedIt != graspedObjects.end(); graspedIt++)
	{
		const string& object_name = graspedIt->first;
		const string& arm = graspedIt->second.first;
		string arm_prefix = arm.substr(0, arm.find_first_of("_"));
		ROS_INFO("%s attaching object %s to arm %s", logName.c_str(), object_name.c_str(), arm.c_str());
		bool is_attached = scene->getCurrentStateNonConst().hasAttachedBody(object_name);
		bool exist_in_world = world->hasObject(object_name);
		if (exist_in_world)
		{
			// attach
			collision_detection::World::ObjectConstPtr object = world->getObject(object_name);
			attachObject(arm_prefix, object_name, object->shapes_, defaultAttachPose, robot_state);
			world->removeObject(object_name);
		}
		else if (is_attached)
		{
			// if incorrect arm update arm
			const moveit::core::AttachedBody* attachedObject = robot_state.getAttachedBody(object_name);
			string attach_link_name = arm_prefix[0]+"_wrist_roll_link";
			if (attachedObject->getAttachedLinkName() != attach_link_name)
			{
				std::vector<shapes::ShapeConstPtr> shapes = attachedObject->getShapes();
				robot_state.clearAttachedBody(object_name);
				attachObject(arm_prefix, object_name, shapes, defaultAttachPose, robot_state);
			}
		}
	}
}

bool TidyupPlanningSceneUpdater::readRobotPose2D(
		geometry_msgs::Pose2D& robot_pose,
		double& torso_position,
		modules::numericalFluentCallbackType numericalFluentCallback)
{
	ParameterList poseParams;
	NumericalFluentList nfRequest;
	nfRequest.reserve(4);
	nfRequest.push_back(NumericalFluent("robot-x", poseParams));
	nfRequest.push_back(NumericalFluent("robot-y", poseParams));
	nfRequest.push_back(NumericalFluent("robot-theta", poseParams));
	nfRequest.push_back(NumericalFluent("robot-torso-position", poseParams));

	NumericalFluentList* nfRequestP = &nfRequest;
	if ( !numericalFluentCallback(nfRequestP))
	{
		ROS_ERROR("numericalFluentCallback failed.");
		return false;
	}

	robot_pose.x = nfRequest[0].value;
	robot_pose.y = nfRequest[1].value;
	robot_pose.theta = nfRequest[2].value;
	torso_position = nfRequest[3].value;
	return true;
}

bool TidyupPlanningSceneUpdater::readPoint(
		geometry_msgs::Point& point,
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

bool TidyupPlanningSceneUpdater::readPose(
		geometry_msgs::Pose& pose,
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

void TidyupPlanningSceneUpdater::addObject(
		planning_scene::PlanningScenePtr scene,
		const string& object_name,
		const geometry_msgs::Pose& object_pose)
{
	ROS_INFO_STREAM("adding object: "<<object_name);
	collision_detection::WorldPtr world = scene->getWorldNonConst();
	Eigen::Affine3d pose_eigen;
	tf::poseMsgToEigen(object_pose, pose_eigen);
	shapes::ShapePtr shape (new shapes::Cylinder(0.025, 0.1));
	world->addToObject(object_name, shape, pose_eigen);
}

void TidyupPlanningSceneUpdater::visualize(planning_scene::PlanningScenePtr scene)
{
	moveit_msgs::PlanningScene msg;
	scene->getPlanningSceneMsg(msg);
	scene_publisher.publish(msg);
}

void TidyupPlanningSceneUpdater::attachObject(
		const string& arm_prefix,
		const string& object,
		const std::vector<shapes::ShapeConstPtr>&
		shapes, const geometry_msgs::Pose& grasp,
		robot_state::RobotState& robot_state)
{
	tf::Pose attach_pose_tf;
	tf::poseMsgToTF(grasp, attach_pose_tf);
	Eigen::Affine3d attach_pose_eigen;
	tf::poseTFToEigen(attach_pose_tf, attach_pose_eigen);
	EigenSTL::vector_Affine3d attach_trans;
	attach_trans.push_back(attach_pose_eigen);
	const std::vector<std::string>& names = robot_state.getRobotModel()->getEndEffector(arm_prefix+"_eef")->getLinkModelNamesWithCollisionGeometry();
	std::set<std::string> touch_links;
	touch_links.insert(names.begin(), names.end());
	std::string link_name = arm_prefix[0]+"_wrist_roll_link";
	robot_state.attachBody(object, shapes, attach_trans, touch_links, link_name);
}

}
