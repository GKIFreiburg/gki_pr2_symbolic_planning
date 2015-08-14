#ifndef STATE_CREATOR_ROBOT_POSE_GROUNDING_H
#define STATE_CREATOR_ROBOT_POSE_GROUNDING_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace tidyup_state_creators
{

    /// This state creator sets the current robot pose to the state.
    /**
     * The current pose is estimated via tf as the transform from /map to /base_link.
     * A stamped pose in the state is represented by the fluents: robot-x robot-y robot-theata
     *
     * The state creator serves two purposes:
     * 1.) The fluents for the robot pose are set to the real values
     * 2.) Set the robot-near-table predicate if appropriate
     *
     */
    class StateCreatorRobotPoseGrounding : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPoseGrounding();
            ~StateCreatorRobotPoseGrounding();

            /// Initialize the state creator parameters.
            /**
             * robot-x robot-y robot-theta robot-near-table
             **/
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            tf::TransformListener _tf;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            string robot_x_;
            string robot_y_;
            string robot_theta_;
            string prediate_robot_near_table_;
    };

};

#endif // STATE_CREATOR_ROBOT_POSE_GROUNDING_H

