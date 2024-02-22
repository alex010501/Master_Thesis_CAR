#include "RobotLink.h"

RobotLink::RobotLink(Offset endLink, Axes AxisLink, Axes AxisLinkPrev)
{
	this->ExitPoint = endLink;
	this->ExitJointOrientation = AxisLink;
	this->PreviousJointOrientation = AxisLinkPrev;
	this->mass = 0;
	Offset temp; //remake with dynamics adding
	temp.x = 0;
	temp.y = 0;
	temp.z = 0;
	this->massPoint = temp;
	this->inertia = 0;
}

RobotLink::RobotLink(const RobotLink &t)
{
	this->ExitPoint = t.ExitPoint;
	this->ExitJointOrientation = t.ExitJointOrientation;
	this->PreviousJointOrientation = t.PreviousJointOrientation;
	this->mass = t.mass;
	this->massPoint = t.massPoint;
	this->inertia = t.inertia;
}