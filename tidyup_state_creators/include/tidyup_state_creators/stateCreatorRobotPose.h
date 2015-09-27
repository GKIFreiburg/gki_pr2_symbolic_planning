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

    /// This state creator adds the current robot pose to the state.
    /**
     * The current pose is estimated via tf as the transform from /map to /base_link.
     * A stamped pose in the state is represented by the fluents: x y z qx qy qz qw as well as timestamp and frame-id
     *
     * The state creator serves two purposes:
     * 1. The fluents for the robot_pose are set to the real values
     * 2. An at-style predicate of the form (at location) is set.
     *
     * 1. If _robotPoseObject is not empty the x,y,z,etc. fluents will be filled accordingly.
     * 2.a If the _atPredicate is not empty, the at predicate will be set to the current robot pose.
     * 2.b If a _locationType is given it is checked if the current robot pose is one of the
     *      poses in the objects of _locationType and thus _atPredicate is possibly set to
     *      a location instead of the current pose.
     *      If a robot is "at" a location is determined by the _goalToleranceXY and _goalToleranceYaw.
     */
    class StateCreatorRobotPose : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPose();
            ~StateCreatorRobotPose();

            /// Initialize the state creator parameters.
            /**
             * robot_pose robot_pose_type at_predicate locations_type
             *
             * If any parameter is given as "-" (dash), it is assumed empty.
             */
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
            string robot_at_;

            moveit::planning_interface::MoveGroup* torso_group_;

            std::map<std::string, InverseCapabilityOcTree*> inv_cap_maps_;
    };

};

#endif

