#include "tidyup_state_creators/stateCreatorFromPlanningScene.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorFromPlanningScene,
		continual_planning_executive::StateCreator)

		namespace tidyup_state_creators
		{
	StateCreatorFromPlanningScene::StateCreatorFromPlanningScene()
	{
		ros::NodeHandle nh;
		srvPlanningScene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(
				move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	}

	StateCreatorFromPlanningScene::~StateCreatorFromPlanningScene()
	{
	}

	void StateCreatorFromPlanningScene::initialize(
			const std::deque<std::string>& arguments)
	{
	}

	bool StateCreatorFromPlanningScene::fillState(SymbolicState& state)
	{
		initializePlanningScene();
		// Tables have been added to symbolic state in goalCreatorLoadTablesIntoPlanningScene
		initializeTables(state);

		// Store all collision object ids from the symbolic state into this set
		// If a collision object is not listed in the planning scene anymore, means that it can be deleted and therefore
		// is removed from the set.
		// At the end, all remaining objects in the set are going to be removed from the symbolic state
		std::set<std::string> objects_to_remove;

		// store all movable objects from symbolic state in the set
		const multimap<string, string> objects = state.getTypedObjects();
		std::pair<multimap<string, string>::const_iterator,
		multimap<string, string>::const_iterator> iterators = objects.equal_range(
				"movable_object");
		for (multimap<string, string>::const_iterator it = iterators.first;
				it != iterators.second; it++)
		{
			objects_to_remove.insert(it->second);
		}

		ROS_DEBUG_STREAM(
				"StateCreatorFromPlanningScene::" << __func__ << ": number of collision objects in planning scene: " << planningScene_.world.collision_objects.size());

		forEach(const moveit_msgs::CollisionObject& object, planningScene_.world.collision_objects){
			ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": processing object: " << object.id);

			if (StringUtil::startsWith(object.id, "table"))
			{
				// tables are already in symbolic state - loaded in goalCreatorLoadTablesIntoPlanningScene
				continue;
			}
			if (StringUtil::startsWith(object.id, "door"))
			{
				continue;
			}
			if (StringUtil::startsWith(object.id, "sponge"))
			{
				continue;
			}
			// add/update object in symbolic state
			addObjectToSymbolicState(state, object, "movable_object");
			// remove object from set, because it should not be deleted
			objects_to_remove.erase(object.id);

			// add predicate object-on
			std::string closest_table;
			closest_table = findMatchingTable(planningScene_.world.collision_objects, object);
			if (!closest_table.empty())// found a matching table
			{
				ROS_INFO_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": putting " << object.id << " on " << closest_table);
				Predicate pOn;
				pOn.name = "object-on";
				pOn.parameters.push_back(object.id);
				pOn.parameters.push_back(closest_table);
				state.setBooleanPredicate(pOn.name, pOn.parameters, true);
			}
			else
				ROS_WARN("StateCreatorFromPlanningScene::%s: NO matching Table found for object: %s",
						__func__, object.id.c_str());

			// set predicate object-grasped to false, since object-on table means it is not grasped anymore
			bool value = false;
			Predicate p_grasped;
			p_grasped.name = "object-grasped";
			p_grasped.parameters.push_back(object.id);
			p_grasped.parameters.push_back("left_arm");
			if (state.hasBooleanPredicate(p_grasped, &value))
			{
				ROS_WARN("StateCreatorFromPlanningScene::%s: setting predicate %s %s %s to false", __func__,
						p_grasped.name.c_str(), p_grasped.parameters[0].c_str(), p_grasped.parameters[1].c_str());
				state.setBooleanPredicate(p_grasped.name, p_grasped.parameters, false);
			}
			p_grasped.parameters[1] = "right_arm";
			if (state.hasBooleanPredicate(p_grasped, &value))
			{
				ROS_WARN("StateCreatorFromPlanningScene::%s: setting predicate %s %s %s to false", __func__,
						p_grasped.name.c_str(), p_grasped.parameters[0].c_str(), p_grasped.parameters[1].c_str());
				state.setBooleanPredicate(p_grasped.name, p_grasped.parameters, false);
			}

		}
		// attached objects
		forEach(const moveit_msgs::AttachedCollisionObject& attachedObject, planningScene_.robot_state.attached_collision_objects){
			const moveit_msgs::CollisionObject& object = attachedObject.object;
			// add/update object in symbolic state
			addObjectToSymbolicState(state, object, "movable_object");
			// remove object from set, because it should not be deleted
			objects_to_remove.erase(object.id);

			// set predicate object-grasped
			vector<string> params;
			params.push_back(object.id);
			params.push_back("arm_name");
			if (StringUtil::startsWith(attachedObject.link_name, "l_"))
			{
				ROS_DEBUG_STREAM("processing attached object " << object.id << " on left_arm.");
				params[1] = "left_arm";
				state.setBooleanPredicate("object-grasped", params, true);
			}
			else if (StringUtil::startsWith(attachedObject.link_name, "r_"))
			{
				ROS_DEBUG_STREAM("processing attached object " << object.id << " on right_arm.");
				params[1] = "right_arm";
				state.setBooleanPredicate("object-grasped", params, true);
			}
			else
			{
				ROS_ERROR_STREAM("processing attached object " << object.id << " on unknown link.");
			}

			// set predicate object-on to false, since object is no more on the table when grasped
			std::string closest_table;
			closest_table = findMatchingTable(planningScene_.world.collision_objects, object);
			if (!closest_table.empty())
			{
				Predicate pOn;
				pOn.name = "object-on";
				pOn.parameters.push_back(object.id);
				pOn.parameters.push_back(closest_table);
				state.setBooleanPredicate(pOn.name, pOn.parameters, false);
			}
		}

		// remove all objects not present in planning scene from symbolic state and also the predicates are removed
		for (std::set<std::string>::iterator it = objects_to_remove.begin();
				it != objects_to_remove.end(); it++)
		{
			ROS_INFO(
					"StateCreatorFromPlanningScene::%s: removing object %s from symbolic state",
					__func__, (*it).c_str());
			state.removeObject(*it);
		}

		return true;
	}

	void StateCreatorFromPlanningScene::initializePlanningScene()
	{
		ROS_INFO("StateCreatorFromPlanningScene::%s: Waiting for %s service.",
				__func__, move_group::GET_PLANNING_SCENE_SERVICE_NAME.c_str());
		srvPlanningScene_.waitForExistence();

		moveit_msgs::GetPlanningScene::Request request;
		moveit_msgs::GetPlanningScene::Response response;
		//request.components.WORLD_OBJECT_NAMES; IMPORTANT: This declaration does not work!
		request.components.components =
				moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY
				| moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

		if (!srvPlanningScene_.call(request, response))
		{
			ROS_ERROR(
					"StateCreatorFromPlanningScene::%s: Failed to get initial planning scene.",
					__func__);
		}

		ROS_DEBUG(
				"StateCreatorFromPlanningScene::%s: Number of collision objects: %lu",
				__func__, response.scene.world.collision_objects.size());

		setPlanningScene(response.scene);
	}

	void StateCreatorFromPlanningScene::setPlanningScene(
			const moveit_msgs::PlanningScene& scene)
	{
		planningScene_ = scene;
	}

	void StateCreatorFromPlanningScene::initializeTables(
			const SymbolicState& currentState)
	{
		ROS_DEBUG_STREAM("processing tables");
		pair<SymbolicState::TypedObjectConstIterator,
		SymbolicState::TypedObjectConstIterator> tablesRange =
				currentState.getTypedObjects().equal_range("table");
		for (SymbolicState::TypedObjectConstIterator tablesIterator =
				tablesRange.first; tablesIterator != tablesRange.second; tablesIterator++)
		{
			ROS_DEBUG_STREAM("processing "<<tablesIterator->second);
			tables_.insert(tablesIterator->second);

			// Note: tableLocations_ not needed at the moment therefore commented out
			//            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> locationsRange =
			//                    currentState.getTypedObjects().equal_range("manipulation_location");
			//            for (SymbolicState::TypedObjectConstIterator locationsIterator = locationsRange.first;
			//                    locationsIterator != locationsRange.second; locationsIterator++)
			//            {
			//                // (location-near-table ?l - manipulation-location ?t - table)
			//                Predicate pAt;
			//                pAt.name = "location-near-table";
			//                pAt.parameters.push_back(locationsIterator->second);
			//                pAt.parameters.push_back(tablesIterator->second);
			//                bool value = false;
			//                currentState.hasBooleanPredicate(pAt, &value);
			//                if (value)
			//                {
			//                    ROS_DEBUG_STREAM("adding location "<<locationsIterator->second<<" to "<<tablesIterator->second);
			//                    tableLocations_.insert(make_pair(tablesIterator->second, locationsIterator->second));
			//                }
			//            }
		}
	}

	bool StateCreatorFromPlanningScene::extractPoseStampedFromCollisionObject(
			const moveit_msgs::CollisionObject& co,
			geometry_msgs::PoseStamped& pose) const
	{
		if (co.mesh_poses.empty() && co.primitive_poses.empty())
		{
			ROS_WARN(
					"stateCreatorFromPlanningScene::%s: CollisionObject %s had no mesh_poses nor primitive_poses",
					__func__, co.id.c_str());
			return false;
		}

		std::vector<geometry_msgs::Pose> poses;
		if (!co.mesh_poses.empty() && !co.primitive_poses.empty())
		{
			ROS_WARN(
					"%s: CollisionObject %s had mesh_poses and primitive_poses -> using primitive_poses",
					__func__, co.id.c_str());
			poses = co.primitive_poses;
		}
		else if (!co.primitive_poses.empty())
		{
			poses = co.primitive_poses;
		}
		else
		{
			ROS_ASSERT(!co.mesh_poses.empty());
			poses = co.mesh_poses;
		}

		if (poses.size() > 1)
		{
			ROS_WARN("%s: CollisionObject %s had %zu poses -> using first.", __func__,
					co.id.c_str(), poses.size());
		}
		pose.pose = poses.front();
		pose.header = co.header;
		return true;
	}

	void StateCreatorFromPlanningScene::addObjectToSymbolicState(
			SymbolicState& state, const moveit_msgs::CollisionObject& co,
			const std::string& objectType)
	{
		// Verify that objectType is spelled correctly
		if (!doesObjectTypeExist(objectType))
			return;
		state.addObject(co.id, objectType);
		state.setNumericalFluent("timestamp", co.id, co.header.stamp.toSec());
		state.addObject(co.header.frame_id, "frameid");
		ROS_DEBUG_STREAM(
				"StateCreatorFromPlanningScene::" << __func__ << " object: " << co.id << " has frame: " << co.header.frame_id);
		state.setObjectFluent("frame-id", co.id, co.header.frame_id);

		geometry_msgs::PoseStamped poseStamped;
		if (!extractPoseStampedFromCollisionObject(co, poseStamped))
		{
			ROS_ERROR(
					"StateCreatorFromPlanningScene::%s: object:%s does not have a pose!",
					__func__, co.id.c_str());
			return;
		}
		geometry_msgs::Pose pose = poseStamped.pose;
		state.setNumericalFluent("x", co.id, pose.position.x);
		state.setNumericalFluent("y", co.id, pose.position.y);
		state.setNumericalFluent("z", co.id, pose.position.z);
		state.setNumericalFluent("qx", co.id, pose.orientation.x);
		state.setNumericalFluent("qy", co.id, pose.orientation.y);
		state.setNumericalFluent("qz", co.id, pose.orientation.z);
		state.setNumericalFluent("qw", co.id, pose.orientation.w);
	}

	bool StateCreatorFromPlanningScene::doesObjectTypeExist(
			const string& objectType)
	{
		std::string types[] =
		{ "pose", "frameid", "location", "manipulation_location", "table",
				"movable_object", "arm", "arm_state" };
		std::set<std::string> objectTypes(types,
				types + sizeof(types) / sizeof(types[0]));
		ROS_ASSERT(objectTypes.size() == 8);

		if (objectTypes.find(objectType) != objectTypes.end())
		{
			return true;
		}
		else
		{
			ROS_ERROR(
					"StateCreatorFromPlanningScene::%s: Object Type %s does not exist "
					"- maybe typo", __func__, objectType.c_str());
			return false;
		}
	}

	std::string StateCreatorFromPlanningScene::findMatchingTable(
			const std::vector<moveit_msgs::CollisionObject>& allCos,
			const moveit_msgs::CollisionObject& co)
	{
		string closest_table = "";
		double closest_distance = 2.0;
		forEach(const moveit_msgs::CollisionObject& table, allCos){
			// if collisionObject table is really a table (was added in initializedTables())
			if (tables_.find(table.id) != tables_.end())
			{
				ROS_DEBUG("StateCreatorFromPlanningScene::%s: CO Name: %s was found in tables_ and object name is: %s",
						__func__, table.id.c_str(), co.id.c_str());
				geometry_msgs::PoseStamped table_pose_stamped;
				if (!extractPoseStampedFromCollisionObject(table, table_pose_stamped))
				{
					ROS_ERROR("StateCreatorFromPlanningScene::%s: table:%s does not have a pose!", __func__, table.id.c_str());
					continue;
				}

				// get the point of co
				geometry_msgs::PoseStamped co_pose_stamped;
				if (!extractPoseStampedFromCollisionObject(co, co_pose_stamped))
				{
					ROS_ERROR("StateCreatorFromPlanningScene::%s: object:%s does not have a pose!", __func__, co.id.c_str());
					return closest_table;
				}

				// using only positions of table and collision object
				geometry_msgs::Point table_origin;
				geometry_msgs::Point co_origin;

				// verify that poses are in same frame
				if (table_pose_stamped.header.frame_id != co.header.frame_id)
				{
					ROS_WARN("StateCreatorFromPlanningScene::%s: table pose in frame: %s and object pose in frame: %s",
							__func__, table_pose_stamped.header.frame_id.c_str(), co.header.frame_id.c_str());
					ROS_INFO("StateCreatorFromPlanningScene::%s: converting object pose to frame %s", __func__,
							table_pose_stamped.header.frame_id.c_str());

					// transform object pose in table frame
					geometry_msgs::PoseStamped co_pose_transformed;
					try
					{
						tf_.waitForTransform(table_pose_stamped.header.frame_id, co_pose_stamped.header.frame_id,
								co_pose_stamped.header.stamp,
								ros::Duration(0.5));
						tf_.transformPose(table_pose_stamped.header.frame_id, co_pose_stamped, co_pose_transformed);
					}
					catch (tf::TransformException& ex)
					{
						ROS_ERROR("StateCreatorFromPlanningScene::%s: %s", __func__, ex.what());
					}
					table_origin = table_pose_stamped.pose.position;
					co_origin = co_pose_transformed.pose.position;
				}
				else
				{
					table_origin = table_pose_stamped.pose.position;
					co_origin = co_pose_stamped.pose.position;
				}

				// Verify if object (co) is beneath table
				// Since object poses are  projected on table surface therefore need a certain threshold
				double threshold = 0.02;
				if (table_origin.z - co_origin.z > threshold)
				{
					ROS_WARN("StateCreatorFromPlanningScene::%s: collision object %s (h = %lf) is beneath table %s (h = %lf)"
							"with threshold %lf", __func__, co.id.c_str(), co_origin.z, table.id.c_str(), table_origin.z, threshold);
					continue;
				}
				// simplified: find table with smallest distance to object
				double distance = hypot(co_origin.x - table_origin.x, co_origin.y - table_origin.y);
				ROS_DEBUG("StateCreatorFromPlanningScene::%s: Distance between %s and %s is %lf", __func__, table.id.c_str(), co.id.c_str(), distance);
				if (distance < closest_distance)
				{
					closest_distance = distance;
					closest_table = table.id;
				}
			}
		}

		return closest_table;
	}
		}
;

