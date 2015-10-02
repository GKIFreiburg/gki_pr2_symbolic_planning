#ifndef STATE_CREATOR_ROBOT_POSE_H
#define STATE_CREATOR_ROBOT_POSE_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <inverse_capability_map/InverseCapabilityOcTree.h>

namespace tidyup_state_creators
{

// TODO: check if everything is working, if so, remove transformTorsoInTableFrame function


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
    class StateCreatorRobotPose : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPose();
            ~StateCreatorRobotPose();

            /// Initialize the state creator parameters.
            /**
             * robot-x robot-y robot-theta robot-near-table
             **/
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            tf::TransformListener tf_;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            string robot_x_;
            string robot_y_;
            string robot_theta_;
            string robot_torso_position_;
            string prediate_robot_near_table_;

            moveit::planning_interface::MoveGroup* torso_group_;

            std::map<std::string, InverseCapabilityOcTree*> inv_cap_maps_;

            // Fetch torso pose and convert into table frame
            tf::Pose transformTorsoInTableFrame(const geometry_msgs::PoseStamped& table);
    };

};

#endif // STATE_CREATOR_ROBOT_POSE_H

