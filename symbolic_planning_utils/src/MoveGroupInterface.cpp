/*
 * MoveGroupInterface.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: andreas
 */

#include "MoveGroupInterface.h"

MoveGroupInterface* MoveGroupInterface::instance = NULL;

MoveGroupInterface::MoveGroupInterface()
{
//    right_arm_group_ = new ...
}

MoveGroupInterface::~MoveGroupInterface()
{
}

MoveGroupInterface* MoveGroupInterface::getInstance()
{
    if (MoveGroupInterface::instance == NULL)
    {
        MoveGroupInterface::instance = new MoveGroupInterface();
    }
    return MoveGroupInterface::instance;
}

