#pragma once

#include <Eigen/Dense>
#include "RobotAdditions.h"

class RobotLink //Class of links of Robot include math and visualization
{
private: //Private members

public:
    //Dynamic parameters
	double mass;
	Offset massPoint;
	double inertia;

    //Kinematic parameters
	Axes PreviousJointOrientation;
	Axes ExitJointOrientation;
	Offset ExitPoint;

	//Class constructor
	RobotLink() = default;
	RobotLink(const RobotLink &t);
	RobotLink(Offset endLink, Axes AxisLink, Axes AxisLinkPrev);
};