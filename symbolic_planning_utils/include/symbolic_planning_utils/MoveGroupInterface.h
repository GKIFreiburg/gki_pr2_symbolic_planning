/*
 * MoveGroupInterface.h
 *
 *  Created on: Mar 19, 2015
 *      Author: andreas
 */

#ifndef MOVEGROUPINTERFACE_H_
#define MOVEGROUPINTERFACE_H_

class MoveGroupInterface
{
private:
    static MoveGroupInterface* instance;
    moveit::planning_interface::MoveGroup* right_arm_group_;
    moveit::planning_interface::MoveGroup* left_arm_group_;
    MoveGroupInterface();
public:
    static MoveGroupInterface* getInstance();
    virtual ~MoveGroupInterface();
};

#endif /* MOVEGROUPINTERFACE_H_ */
