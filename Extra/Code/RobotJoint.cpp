#include "RobotJoint.h"

RobotJoint::RobotJoint(MoveType j)
{
	this->curCoord = 0;
	this->jointType = j;
}