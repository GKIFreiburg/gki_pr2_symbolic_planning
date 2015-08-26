#ifndef ACTION_EXECUTOR_R_O_S_NAVIGATION_GROUNDING_H
#define ACTION_EXECUTOR_R_O_S_NAVIGATION_GROUNDING_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <vector>
#include <utility>

namespace navigation_actions
{

    class ActionExecutorROSNavigationGrounding : public continual_planning_executive::ActionExecutorInterface
    {
        public:
            /**
             * Parameters:
             * action_plan_name action_server_name 
             *      [start (<predicate> true|false)+] [goal (<predicate> true|false)+]
             * Ex: drive /move_base start visited true goal exploring true 
             */
            virtual void initialize(const std::deque<std::string> & arguments);

    		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

    		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

    		virtual void cancelAction();

        protected:
            std::string action_name_;
            std::string action_topic_move_base_;
            std::string predicate_table_inspected_recently_;
            std::string sampled_torso_height_;

            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* action_move_base_;

            // also possible to use action client for lifting torso
            // topic: "torso_controller/position_joint_action"
            // actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction>* action_lift_torso_;

            // using move group instead of normal action server because move group also checks for collisions
            moveit::planning_interface::MoveGroup* torso_group_;

            bool executeMoveBase(const geometry_msgs::PoseStamped& target_pose);

            bool executeLiftTorso(const geometry_msgs::PoseStamped& target_pose);

    };

};

#endif // ACTION_EXECUTOR_R_O_S_NAVIGATION_GROUNDING_H

