#ifndef ACTION_EXECUTOR_INSPECT_LOCATION_H
#define ACTION_EXECUTOR_INSPECT_LOCATION_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>

namespace object_manipulation_actions
{
    class ActionExecutorInspectLocation : public ActionExecutorActionlib<
    ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction,
    ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal,
    ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult>
    {
        public:
        /// Initialize the executor by creating the ActionClient.
        /**
         * \param [in] arguments should be name of action (as in the plan),
         *    and the name of the action server to connect to,
         *    e.g. "driveBase move_base"
         */
        virtual void initialize(const std::deque<std::string> & arguments);

        /// Determine if the action can be executed by this implementation.
        /**
         * Default implementation only compares _actionName with the name of the DurativeAction.
         */
        virtual bool canExecute(const DurativeAction & a, const SymbolicState & current) const;

        /// Fill the goal to execute this action.
        /**
         * \param [out] goal the ActionGoal to be filled.
         * \param [in] a the DurativeAction that is to be executed.
         * \param [in] current the current state where a should be executd.
         */
        virtual bool fillGoal(ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal & goal,
        		const DurativeAction & a, const SymbolicState & current);

        /// Update the state after an action was executed.
        /**
         * \param [in] actionReturnState the state of the executed action
         * \param [in] result the result returned by the action
         * \param [in, out] current the current planner state to be updated
         */
        virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
        		const ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result,
                const DurativeAction & a, SymbolicState & current);

        private:
        std::string action_topic_;
        std::string action_name_;
        std::string predicate_name_;

        bool add_tables_;
        bool verify_planning_scene_update_;

        std::set<std::string> expected_objects_;

    };

};

#endif

