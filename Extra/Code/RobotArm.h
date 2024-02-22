#pragma once
#include <vector>
#include <Eigen/Dense>
#include <MathAdditions.h>
#include "RobotLink.h"
#include "RobotJoint.h"
#include "RobotAdditions.h"

class RobotArm
{
private:
	bool isInitialized = false;

	RobotLink LinkZero;
	std::vector<RobotLink> links;
	std::vector<RobotJoint> joints;
	std::vector<DirectPoint> DHPoints;
	std::vector<DHParams> LinkJointParams;

	void CalcDHPoints();
	void PointsToParams();
	Offset CalcLinkFullOffset(int index);

public:
	Offset originPosition;

	RobotArm() = default;
	RobotArm(Offset originPoint, Offset	endLinkZero, Axes AxisLinkZero);

	void AddLink(Offset endLink, Axes AxisLink, MoveType jointType);

	void initialize();

	Eigen::VectorXd getJointAngles();

	void setJointAngles(Eigen::VectorXd q);

	DirectPoint ForwardKinematics(Eigen::VectorXd q);
	
	static DirectPoint ForwardKinematics_static(RobotArm* arm, Eigen::VectorXd q);

	Eigen::VectorXd solveIK_POS(DirectPoint needPoint, std::string method);

	Eigen::VectorXd solveIK_VEL(Eigen::VectorXd needVelocity);
};