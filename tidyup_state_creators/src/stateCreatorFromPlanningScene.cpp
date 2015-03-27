#include "tidyup_state_creators/stateCreatorFromPlanningScene.h"
#include "tidyup_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

PLUGINLIB_EXPORT_CLASS(tidyup_state_creators::StateCreatorFromPlanningScene, continual_planning_executive::StateCreator)

namespace tidyup_state_creators
{
    StateCreatorFromPlanningScene::StateCreatorFromPlanningScene()
    {
    	ros::NodeHandle nh;
    	srvPlanningScene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    }

    StateCreatorFromPlanningScene::~StateCreatorFromPlanningScene()
    {
    }

    void StateCreatorFromPlanningScene::initialize(const std::deque<std::string>& arguments)
    {
    }

    bool StateCreatorFromPlanningScene::fillState(SymbolicState& state)
    {
    	initializePlanningScene();
    	// Tables have been added to symbolic state in goalCreatorLoadTablesIntoPlanningScene
    	initializeTables(state);


    	// delete all movable objects from symbolic state, and re-add the important objects later (from PS)
		const multimap<string, string> objects = state.getTypedObjects();
		std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> iterators =
		        objects.equal_range("movable_object");
		for (multimap<string, string>::const_iterator it = iterators.first; it != iterators.second; it++)
		{
			// also predicates with the objects are removed
			state.removeObject(it->second);
		}

    	ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": number of collision objects in planning scene: "
    			<< planningScene_.world.collision_objects.size());

    	forEach(const moveit_msgs::CollisionObject& object, planningScene_.world.collision_objects)
		{
    		ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": processing object: " << object.id);

    		if (StringUtil::startsWith(object.id, "table"))
    		{
    			// tables are already in symbolic state - load in goalCreatorLoadTablesIntoPlanningScene
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
    		addObjectToSymbolicState(state, object, "movable_object");
    		// add object-on predicate to object
    		findMatchingTable(state, planningScene_.world.collision_objects, object);
		}
    	// attached objects
    	forEach(const moveit_msgs::AttachedCollisionObject& attachedObject, planningScene_.robot_state.attached_collision_objects)
    	{
    		const moveit_msgs::CollisionObject& object = attachedObject.object;
    		addObjectToSymbolicState(state, object, "movable_object");

            // grasped predicate
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
        request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
            moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

        if (!srvPlanningScene_.call(request, response))
        {
            ROS_ERROR("StateCreatorFromPlanningScene::%s: Failed to get initial planning scene.", __func__);
        }

        ROS_DEBUG("StateCreatorFromPlanningScene::%s: Number of collision objects: %lu",
        		__func__, response.scene.world.collision_objects.size());

        setPlanningScene(response.scene);
    }

    void StateCreatorFromPlanningScene::setPlanningScene(const moveit_msgs::PlanningScene& scene)
    {
    	planningScene_ = scene;
    }

    void StateCreatorFromPlanningScene::initializeTables(const SymbolicState& currentState)
    {
        ROS_DEBUG_STREAM("processing tables");
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> tablesRange =
                currentState.getTypedObjects().equal_range("table");
        for (SymbolicState::TypedObjectConstIterator tablesIterator = tablesRange.first;
                tablesIterator != tablesRange.second; tablesIterator++)
        {
            ROS_DEBUG_STREAM("processing "<<tablesIterator->second);
            tables_.insert(tablesIterator->second);
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

    bool StateCreatorFromPlanningScene::extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject& co,
    		geometry_msgs::PoseStamped& pose) const
    {
    	if (co.mesh_poses.empty() && co.primitive_poses.empty())
    	{
    		ROS_WARN("stateCreatorFromPlanningScene::%s: CollisionObject %s had no mesh_poses nor primitive_poses", __func__, co.id.c_str());
    		return false;
    	}

        std::vector<geometry_msgs::Pose> poses;
        if(!co.mesh_poses.empty() && !co.primitive_poses.empty()) {
            ROS_WARN("%s: CollisionObject %s had mesh_poses and primitive_poses -> using primitive_poses",
                    __func__, co.id.c_str());
            poses = co.primitive_poses;
        } else if(!co.primitive_poses.empty()) {
            poses = co.primitive_poses;
        } else {
            ROS_ASSERT(!co.mesh_poses.empty());
            poses = co.mesh_poses;
        }

        if(poses.size() > 1) {
            ROS_WARN("%s: CollisionObject %s had %zu poses -> using first.", __func__, co.id.c_str(), poses.size());
        }
        pose.pose = poses.front();
        pose.header = co.header;
    	return true;
    }

    void StateCreatorFromPlanningScene::addObjectToSymbolicState(SymbolicState& state, const moveit_msgs::CollisionObject& co,
    		const std::string& objectType)
    {
    	// Verify that objectType is spelled correctly
    	if (!doesObjectTypeExist(objectType))
    		return;
    	state.addObject(co.id, objectType);
        state.setNumericalFluent("timestamp", co.id, co.header.stamp.toSec());
        state.addObject(co.header.frame_id, "frameid");
        ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << " object: " << co.id
        		<< " has frame: " << co.header.frame_id);
        state.setObjectFluent("frame-id", co.id, co.header.frame_id);

        geometry_msgs::PoseStamped poseStamped;
        if (!extractPoseStampedFromCollisionObject(co, poseStamped))
        {
        	ROS_ERROR("StateCreatorFromPlanningScene::%s: object:%s does not have a pose!", __func__, co.id.c_str());
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

    bool StateCreatorFromPlanningScene::doesObjectTypeExist(const string& objectType)
    {
    	std::string types[] = { "pose", "frameid", "location", "manipulation_location",
    							"table", "movable_object", "arm", "arm_state" };
    	std::set<std::string> objectTypes(types, types + sizeof(types) / sizeof(types[0]));
    	ROS_ASSERT(objectTypes.size() == 8);

    	if (objectTypes.find(objectType) != objectTypes.end())
    	{
    		return true;
    	}
    	else
    	{
    		ROS_ERROR("StateCreatorFromPlanningScene::%s: Object Type %s does not exist "
    				"- maybe typo", __func__, objectType.c_str());
    		return false;
    	}
    }

    void StateCreatorFromPlanningScene::findMatchingTable(SymbolicState& currentState,
    		const std::vector<moveit_msgs::CollisionObject>& allCos,
    		const moveit_msgs::CollisionObject& co)
    {
        string closest_table = "table";
        double closest_distance = 2.0;
        forEach(const moveit_msgs::CollisionObject& table, allCos)
        {
        	// if collisionObject table is really a table (was added in initializedTables())
            if (tables_.find(table.id) != tables_.end())
            {
                geometry_msgs::PoseStamped tablePoseStamped;
                if (!extractPoseStampedFromCollisionObject(table, tablePoseStamped))
                {
                	ROS_ERROR("StateCreatorFromPlanningScene::%s: table:%s does not have a pose!", __func__, table.id.c_str());
                	return;
                }
                const geometry_msgs::Point& origin = tablePoseStamped.pose.position;

                // get the point of co
                geometry_msgs::PoseStamped coPoseStamped;
                if (!extractPoseStampedFromCollisionObject(co, coPoseStamped))
                {
                	ROS_ERROR("StateCreatorFromPlanningScene::%s: object:%s does not have a pose!", __func__, co.id.c_str());
                	return;
                }
                const geometry_msgs::Point& coPoint = coPoseStamped.pose.position;

                // co is beneath table
                if (origin.z > coPoint.z)
                    continue;
                // simplified: find table with smallest distance to object
                double distance = hypot(coPoint.x - origin.x, coPoint.y - origin.y);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_table = table.id;
                }
            }
        }
        if (closest_table != "table") // found a matching table
        {
            ROS_DEBUG_STREAM("putting " << co.id << " on " << closest_table);
            Predicate pOn;
            pOn.name = "object-on";
            pOn.parameters.push_back(co.id);
            pOn.parameters.push_back(closest_table);
            currentState.setBooleanPredicate(pOn.name, pOn.parameters, true);
        }
        else
        	ROS_WARN("StateCreatorFromPlanningScene::%s: NO matching Table found for object: %s",
        			__func__, co.id.c_str());
    }

//    std::pair<double, double> StateCreatorFromPlanningScene::distanceBetweenTwoPoses(const geometry_msgs::PoseStamped & posePS,
//            const geometry_msgs::PoseStamped & poseState)
//    {
//        // OR poses might be in a sensor frame -> transform to PS frame first
//        geometry_msgs::PoseStamped poseOR_transformed;
//        try {
//            tf_.waitForTransform(posePS.header.frame_id, poseState.header.frame_id, poseState.header.stamp,
//                    ros::Duration(0.5));
//            tf_.transformPose(posePS.header.frame_id, poseState, poseOR_transformed);
//        } catch (tf::TransformException &ex) {
//            ROS_ERROR("%s", ex.what());
//        }
//
//		ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": frame ObjPlanningScene: "
//				<< posePS.header.frame_id << ": frame ObjSymbolicState: "
//				<< poseState.header.frame_id);
//        tf::Pose tfPS;
//        tf::Pose tfState;
//        tf::poseMsgToTF(posePS.pose, tfPS);
//        tf::poseMsgToTF(poseState.pose, tfState);
//        tf::Pose delta = tfPS.inverseTimes(tfState);
//        return std::make_pair(hypot(delta.getOrigin().x(), delta.getOrigin().y()),
//                fabs(delta.getOrigin().z()));   // usually we're interested in the 2d distance independently
//    }

};

