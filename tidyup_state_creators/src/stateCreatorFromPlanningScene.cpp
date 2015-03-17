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

        ROS_DEBUG("StateCreatorFromPlanningScene::%s: Waiting for %s service.",
        		__func__, move_group::GET_PLANNING_SCENE_SERVICE_NAME.c_str());
        srvPlanningScene_.waitForExistence();

    	ros::NodeHandle nhOrk("/ork_to_planning_scene");
    	nhOrk.param("object_match_distance", object_match_distance_, 0.15);
    	nhOrk.param("object_z_match_distance", object_z_match_distance_, 0.15);
    }

    StateCreatorFromPlanningScene::~StateCreatorFromPlanningScene()
    {
    }

    void StateCreatorFromPlanningScene::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool StateCreatorFromPlanningScene::fillState(SymbolicState & state)
    {
    	initializePlanningScene();

    	ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": number of collision objects in planning scene: "
    			<< planning_scene_.world.collision_objects.size());

    	forEach(const moveit_msgs::CollisionObject& object, planning_scene_.world.collision_objects)
		{
    		ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": processing object: " << object.id);
    		if (StringUtil::startsWith(object.id, "table"))
    		{
                geometry_msgs::PoseStamped poseTable;
                if (!extractPoseStampedFromSymbolicState(state, object.id, poseTable))
                {
                	ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ <<
                			": Symbolic state does not have object: " << object.id);
                	// add object to state
                	addObjectToState(state, object, "table");
                	continue;
                }

                geometry_msgs::PoseStamped poseTablePS;
                if (!extractPoseStampedFromCollisionObject(object, poseTablePS))
				{
                	ROS_ERROR_STREAM("StateCreatorFromPlanningScene::" << __func__ <<
                			"PoseStamped could not be extracted from object:%s" << object.id);
                	continue;
				}

                // compute distance between the two poses
    			std::pair<double, double> dist;
    			dist = distanceBetweenTwoPoses(poseTablePS, poseTable);
    			ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": Distance: " <<
    					dist.first << " Height difference: " << dist.second);
    			bool result;
    			result = isMatch(dist, object_match_distance_, object_z_match_distance_);
    			if (result)
    			{
    				ROS_WARN("StateCreatorFromPlanningScene::%s: object: %s already in symbolic state!",
    						__func__, object.id.c_str());
    				// table is already in symbolic state
					continue;
    			}
    			else
    			{
    				ROS_ERROR("StateCreatorFromPlanningScene::%s: object: %s was moved! - should not happen",
    						__func__, object.id.c_str());
    				return false;
    			}
    		}
		}

        return true;
    }

    void StateCreatorFromPlanningScene::initializePlanningScene()
    {
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
    	planning_scene_ = scene;
    }

    bool StateCreatorFromPlanningScene::checkIfTableInState(
    		const SymbolicState& state, const std::string& tableName)
    {
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> tablesRange =
                state.getTypedObjects().equal_range("table");
        for (SymbolicState::TypedObjectConstIterator tablesIterator = tablesRange.first;
                tablesIterator != tablesRange.second; tablesIterator++)
        {
        	ROS_DEBUG_STREAM("processing "<<tablesIterator->second);
        	if (tableName.compare(tablesIterator->second) == 0) // equal
        	{
        		return true;
        	}

        }
    	return false;
    }


    bool StateCreatorFromPlanningScene::extractPoseStampedFromSymbolicState(const SymbolicState & state, const string & object,
            geometry_msgs::PoseStamped & pose) const
    {
        // first get xyz, qxyzw from state
        Predicate p;
        p.parameters.push_back(object);

        double posX = 0;
        p.name = "x";
        if(!state.hasNumericalFluent(p, &posX)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no x-location in state.", __func__, object.c_str());
            return false;
        }
        double posY = 0;
        p.name = "y";
        if(!state.hasNumericalFluent(p, &posY)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no y-location in state.", __func__, object.c_str());
            return false;
        }
        double posZ = 0;
        p.name = "z";
        if(!state.hasNumericalFluent(p, &posZ)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no z-location in state.", __func__, object.c_str());
            return false;
        }

        double qx;
        p.name = "qx";
        if(!state.hasNumericalFluent(p, &qx)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no qx in state.", __func__, object.c_str());
            return false;
        }
        double qy;
        p.name = "qy";
        if(!state.hasNumericalFluent(p, &qy)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no qy in state.", __func__, object.c_str());
            return false;
        }
        double qz;
        p.name = "qz";
        if(!state.hasNumericalFluent(p, &qz)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no qz in state.", __func__, object.c_str());
            return false;
        }
        double qw;
        p.name = "qw";
        if(!state.hasNumericalFluent(p, &qw)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no qw in state.", __func__, object.c_str());
            return false;
        }

        double timestamp;
        p.name = "timestamp";
        if(!state.hasNumericalFluent(p, &timestamp)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no timestamp in state.", __func__, object.c_str());
            return false;
        }

        string frameid;
        p.name = "frame-id";
        if(!state.hasObjectFluent(p, &frameid)) {
            ROS_DEBUG("StateCreatorFromPlanningScene::%s: object: %s - no frameid in state.", __func__, object.c_str());
            return false;
        }

        pose.header.frame_id = frameid;
        pose.header.stamp = ros::Time(timestamp);
        pose.pose.position.x = posX;
        pose.pose.position.y = posY;
        pose.pose.position.z = posZ;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        return true;
    }

    bool StateCreatorFromPlanningScene::extractPoseStampedFromCollisionObject(const moveit_msgs::CollisionObject &co,
    		geometry_msgs::PoseStamped & pose) const
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

    void StateCreatorFromPlanningScene::addObjectToState(SymbolicState & state, const moveit_msgs::CollisionObject& co,
    		const std::string& objectType)
    {
        ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << " object: " << co.id
        		<< " has frame: " << co.header.frame_id);
    	state.addObject(co.id, objectType);
        state.setNumericalFluent("timestamp", co.id, co.header.stamp.toSec());
        state.addObject(co.header.frame_id, "frameid");
        state.setObjectFluent("frame-id", co.id, co.header.frame_id);

        geometry_msgs::Pose pose;
        if (co.primitive_poses.size() != 0)
        	pose = co.primitive_poses[0];
        else if (co.mesh_poses.size() != 0)
        	pose = co.mesh_poses[0];
        else {
        	ROS_ERROR("GoalCreatorFromPlanningScene::%s: object:%s does not have a pose!", __func__, co.id.c_str());
        	return;
        }
        state.setNumericalFluent("x", co.id, pose.position.x);
        state.setNumericalFluent("y", co.id, pose.position.y);
        state.setNumericalFluent("z", co.id, pose.position.z);
        state.setNumericalFluent("qx", co.id, pose.orientation.x);
        state.setNumericalFluent("qy", co.id, pose.orientation.y);
        state.setNumericalFluent("qz", co.id, pose.orientation.z);
        state.setNumericalFluent("qw", co.id, pose.orientation.w);
    }

    std::pair<double, double> StateCreatorFromPlanningScene::distanceBetweenTwoPoses(const geometry_msgs::PoseStamped & posePS,
            const geometry_msgs::PoseStamped & poseState)
    {
		ROS_DEBUG_STREAM("StateCreatorFromPlanningScene::" << __func__ << ": frame ObjPlanningScene: "
				<< posePS.header.frame_id << ": frame ObjSymbolicState: "
				<< poseState.header.frame_id);
        tf::Pose tfPS;
        tf::Pose tfState;
        tf::poseMsgToTF(posePS.pose, tfPS);
        tf::poseMsgToTF(poseState.pose, tfState);
        tf::Pose delta = tfPS.inverseTimes(tfState);
        return std::make_pair(hypot(delta.getOrigin().x(), delta.getOrigin().y()),
                fabs(delta.getOrigin().z()));   // usually we're interested in the 2d distance independently
    }

};

