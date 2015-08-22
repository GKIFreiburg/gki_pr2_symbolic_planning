#ifndef ACTION_EXECUTOR_LIFT_TORSO_GROUNDING_H
#define ACTION_EXECUTOR_LIFT_TORSO_GROUNDING_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_listener.h>

namespace object_manipulation_actions
{

    class ActionExecutorLiftTorsoGrounding : public continual_planning_executive::ActionExecutorInterface
    {
        public:

    	ActionExecutorLiftTorsoGrounding();
		~ActionExecutorLiftTorsoGrounding();

		virtual void initialize(const std::deque<std::string> & arguments);

		virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;

		virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);

		virtual void cancelAction();

        private:
        tf::TransformListener tf_;
        // ROS Interface

        // MoveIt
		moveit::planning_interface::MoveGroup* torso_group_;

		std::string action_name_;
		std::string sampled_torso_height_;

        // LiftTorso-Parameters
        double vdist_threshold_;


        // Action for lifting torso
        // Frame /head_pan_link should be vdist_head_to_table_ above table
        moveit::planning_interface::MoveItErrorCode executeLiftTorso(const geometry_msgs::PoseStamped tablePose);

    };

};

#endif

