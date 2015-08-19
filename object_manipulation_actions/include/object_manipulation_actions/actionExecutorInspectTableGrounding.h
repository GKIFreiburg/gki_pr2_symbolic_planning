#ifndef ACTION_EXECUTOR_INSPECT_TABLE_GROUNDING_H
#define ACTION_EXECUTOR_INSPECT_TABLE_GROUNDING_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/SingleJointPositionAction.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <symbolic_planning_utils/planning_scene_interface.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

namespace object_manipulation_actions
{
    class ActionExecutorInspectTableGrounding : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorInspectTableGrounding();
		~ActionExecutorInspectTableGrounding();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();


        private:
        // ROS Interface
        tf::TransformListener tf_;

        // MoveIt
        moveit::planning_interface::MoveGroup* head_group_;

        std::string action_topic_;
        std::string action_name_;
        std::string predicate_table_inspected_;
        std::string predicate_sensor_data_stale_;
        std::string joint_name_head_yaw_;

        std::set<std::string> expected_objects_;

        ros::Duration actionTimeOut_;
        actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> actionLiftTorso_;
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> actionPointHead_;
        actionlib::SimpleActionClient<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction> actionOrkToPs_;
        boost::shared_ptr<symbolic_planning_utils::PlanningSceneInterface> psi_;

        // Parameters
        // TurnHead-Parameters
        int degrees_;

        // Action for pointing head
        bool executePointHead(const geometry_msgs::PoseStamped tablePose);
        // Action for object detection
        bool executeUpdatePlanningSceneFromORK(bool verify_planning_scene_update,
        		const std::vector<std::string>& expected_objects,
        		bool add_tables,
        		std::string table_prefix,
        		bool merge_tables);

        // Turn head by the given degrees
        bool executeTurnHead(const int degrees);

        // Give table collision object the same in PS as in the symbolic state
        // Basically cut off _number from table collision object
        void renameTableCollisionObject(const std::string& tableName);

    };

};

#endif // ACTION_EXECUTOR_INSPECT_TABLE_GROUNDING_H

