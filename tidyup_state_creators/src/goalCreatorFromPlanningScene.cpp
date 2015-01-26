#include "tidyup_utils/stringutil.h"
#include <tidyup_state_creators/goalCreatorFromPlanningScene.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>

PLUGINLIB_DECLARE_CLASS(tidyup_state_creators, goal_creator_from_planning_scene,
        tidyup_state_creators::GoalCreatorFromPlanningScene, continual_planning_executive::GoalCreator)

namespace tidyup_state_creators
{
    GoalCreatorFromPlanningScene::GoalCreatorFromPlanningScene()
    {
    }

    GoalCreatorFromPlanningScene::~GoalCreatorFromPlanningScene()
    {
    }

    void GoalCreatorFromPlanningScene::setInitialScene(const arm_navigation_msgs::PlanningScene& scene)
    {
        initial_scene = scene;
    }

    void GoalCreatorFromPlanningScene::initializeSceneFromEnvironmentServer()
    {
        arm_navigation_msgs::GetPlanningScene srv;
        if (! ros::service::call("/environment_server/get_planning_scene", srv))
        {
            ROS_ERROR("Failed to get initial planning scene from environment server.");
        }
        setInitialScene(srv.response.planning_scene);
    }

    void GoalCreatorFromPlanningScene::initializeTables(const SymbolicState & currentState)
    {
        ROS_DEBUG_STREAM("processing tables");
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> tablesRange =
                currentState.getTypedObjects().equal_range("static_object");
        for (SymbolicState::TypedObjectConstIterator tablesIterator = tablesRange.first;
                tablesIterator != tablesRange.second; tablesIterator++)
        {
            ROS_DEBUG_STREAM("processing "<<tablesIterator->second);
            tables.insert(tablesIterator->second);
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> locationsRange =
                    currentState.getTypedObjects().equal_range("manipulation_location");
            for (SymbolicState::TypedObjectConstIterator locationsIterator = locationsRange.first;
                    locationsIterator != locationsRange.second; locationsIterator++)
            {
                // (static-object-at-location ?s - static_object ?g - manipulation_location)
                Predicate pAt;
                pAt.name = "static-object-at-location";
                pAt.parameters.push_back(tablesIterator->second);
                pAt.parameters.push_back(locationsIterator->second);
                bool value = false;
                currentState.hasBooleanPredicate(pAt, &value);
                if (value)
                {
                    ROS_DEBUG_STREAM("adding location "<<locationsIterator->second<<" to "<<tablesIterator->second);
                    tableLocations.insert(make_pair(tablesIterator->second, locationsIterator->second));
                }
            }
        }
    }

    void GoalCreatorFromPlanningScene::findMatchingTable(SymbolicState & currentState, const string& object, const geometry_msgs::Pose& pose)
    {
        string closest_table = "table";
        double closest_distance = 2.0;
        forEach(const arm_navigation_msgs::CollisionObject& obj, initial_scene.collision_objects)
        {
            if (tables.find(obj.id) != tables.end())
            {
                const geometry_msgs::Point& origin = obj.poses[0].position;
                // beneath table
                if (origin.z > pose.position.z)
                    continue;
                // simplified: find table with smallest distance to object
                double distance = hypot(pose.position.x-origin.x, pose.position.y-origin.y);
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_table = obj.id;
                }
            }
        }
        if (closest_table != "table")
        {
            ROS_DEBUG_STREAM("putting "<<object<<" on "<<closest_table);
            Predicate pOn;
            pOn.name = "on";
            pOn.parameters.push_back(object);
            pOn.parameters.push_back(closest_table);
            currentState.setBooleanPredicate(pOn.name, pOn.parameters, true);

            // (graspable-from ?o - movable_object ?g - manipulation_location ?a - arm)
//            Predicate pRGraspable;
//            pRGraspable.name = "graspable-from";
//            pRGraspable.parameters.push_back(object);
//            pRGraspable.parameters.push_back("location");
//            Predicate pLGraspable = pRGraspable;
//            pRGraspable.parameters.push_back("right_arm");
//            pLGraspable.parameters.push_back("left_arm");
//            pair<TableLocationsIterator, TableLocationsIterator> locationsRange = tableLocations.equal_range(closest_table);
//            for(TableLocationsIterator locationsIterator = locationsRange.first; locationsIterator != locationsRange.second; locationsIterator++)
//            {
//                const string& location = locationsIterator->second;
//                if (StringUtil::startsWith(location, closest_table))
//                {
//                    pRGraspable.parameters[1] = location;
//                    currentState.setBooleanPredicate(pRGraspable.name, pRGraspable.parameters, true);
//                    pLGraspable.parameters[1] = location;
//                    currentState.setBooleanPredicate(pLGraspable.name, pLGraspable.parameters, true);
//                }
//            }
        }
    }

    void GoalCreatorFromPlanningScene::initialize(const std::deque<std::string> & arguments)
    {
    }

    bool GoalCreatorFromPlanningScene::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ROS_DEBUG_STREAM("fill state from environment server");
        initializeSceneFromEnvironmentServer();
        initializeTables(currentState);
        ROS_DEBUG_STREAM("processing planning scene objects");
        forEach(const arm_navigation_msgs::CollisionObject& object, initial_scene.collision_objects)
        {
            ROS_DEBUG_STREAM("processing object: "<<object.id);
            if (StringUtil::startsWith(object.id, "table"))
            {
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
            currentState.addObject(object.id, "movable_object");
            const geometry_msgs::Pose& pose = object.poses[0];
            currentState.setNumericalFluent("timestamp", object.id, object.header.stamp.toSec());
            currentState.addObject(object.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", object.id, object.header.frame_id);
            currentState.setNumericalFluent("x", object.id, pose.position.x);
            currentState.setNumericalFluent("y", object.id, pose.position.y);
            currentState.setNumericalFluent("z", object.id, pose.position.z);
            currentState.setNumericalFluent("qx", object.id, pose.orientation.x);
            currentState.setNumericalFluent("qy", object.id, pose.orientation.y);
            currentState.setNumericalFluent("qz", object.id, pose.orientation.z);
            currentState.setNumericalFluent("qw", object.id, pose.orientation.w);
            findMatchingTable(currentState, object.id, pose);
        }
        forEach(const arm_navigation_msgs::AttachedCollisionObject& attached, initial_scene.attached_collision_objects)
        {
            const arm_navigation_msgs::CollisionObject& object = attached.object;
            currentState.addObject(object.id, "movable_object");
            const geometry_msgs::Pose& pose = object.poses[0];
            currentState.setNumericalFluent("timestamp", object.id, object.header.stamp.toSec());
            currentState.addObject(object.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", object.id, object.header.frame_id);
            currentState.setNumericalFluent("x", object.id, pose.position.x);
            currentState.setNumericalFluent("y", object.id, pose.position.y);
            currentState.setNumericalFluent("z", object.id, pose.position.z);
            currentState.setNumericalFluent("qx", object.id, pose.orientation.x);
            currentState.setNumericalFluent("qy", object.id, pose.orientation.y);
            currentState.setNumericalFluent("qz", object.id, pose.orientation.z);
            currentState.setNumericalFluent("qw", object.id, pose.orientation.w);

            // grasped predicate
            vector<string> params;
            params.push_back(object.id);
            params.push_back("arm_name");
            if (StringUtil::startsWith(attached.link_name, "l_"))
            {
                ROS_DEBUG_STREAM("processing attached object "<<object.id << " on left_arm.");
                params[1] = "left_arm";
                currentState.setBooleanPredicate("grasped", params, true);
            }
            else if (StringUtil::startsWith(attached.link_name, "r_"))
            {
                ROS_DEBUG_STREAM("processing attached object "<<object.id << " on right_arm.");
                params[1] = "right_arm";
                currentState.setBooleanPredicate("grasped", params, true);
            }
            else
            {
                ROS_DEBUG_STREAM("processing attached object "<<object.id << " on unknown link.");
            }
        }
        return true;
    }

};

