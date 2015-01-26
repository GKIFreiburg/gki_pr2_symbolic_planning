#ifndef GOAL_CREATOR_FROM_PLANNING_SCENE_H
#define GOAL_CREATOR_FROM_PLANNING_SCENE_H

#include "continual_planning_executive/goalCreator.h"
#include <arm_navigation_msgs/PlanningScene.h>
#include <set>

namespace tidyup_state_creators
{

class GoalCreatorFromPlanningScene: public continual_planning_executive::GoalCreator
{
private:
    arm_navigation_msgs::PlanningScene initial_scene;
    std::set<string> tables;
    std::multimap<string, string> tableLocations;
    typedef multimap<string,string>::const_iterator TableLocationsIterator;

    void initializeTables(const SymbolicState & currentState);
    void findMatchingTable(SymbolicState & currentState, const string& object, const geometry_msgs::Pose& pose);
public:

    GoalCreatorFromPlanningScene();
    ~GoalCreatorFromPlanningScene();

    void setInitialScene(const arm_navigation_msgs::PlanningScene& scene);
    void initializeSceneFromEnvironmentServer();

    virtual void initialize(const std::deque<std::string> & arguments);
    virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
};

}
;

#endif

