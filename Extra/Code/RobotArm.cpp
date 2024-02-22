#include <iostream>
#include "RobotArm.h"

RobotArm::RobotArm(Offset originPoint, Offset endLinkZero, Axes AxisLinkZero)
{
	this->originPosition = originPoint;
	RobotLink NewLink(endLinkZero, AxisLinkZero, NoneAxis);
	this->LinkZero = NewLink;	
}

/**
 * Adds a new link to the RobotArm.
 *
 * @param endLink the offset of the end of the new link
 * @param AxisLink the axes of the new link
 * @param jointType the type of joint for the new link
 *
 * @throws std::invalid_argument if the offset is zero
 */
void RobotArm::AddLink(Offset endLink, Axes AxisLink, MoveType jointType)
{
	if (!((endLink.x)||(endLink.y)||(endLink.z)))
	{
		throw std::invalid_argument("Offset must not be zero");
	}

	// Add new joint
	RobotJoint NewJoint(jointType);
	this->joints.push_back(NewJoint);

	// Add new link
	Axes prevAxis;
	if (this->links.size() > 0)
		prevAxis = this->links.back().ExitJointOrientation;
	else
		prevAxis = this->LinkZero.ExitJointOrientation;
	RobotLink NewLink(endLink, AxisLink, prevAxis);
	this->links.push_back(NewLink);
	this->isInitialized = false;
}

/**
 * Calculates the full offset of a robot arm link based on the given index
 *
 * @param index The index of the link
 *
 * @return The calculated offset of the link
 */
Offset RobotArm::CalcLinkFullOffset(int index)
{
	Offset offset;
	if (index == 0)
	{
		offset.x = this->LinkZero.ExitPoint.x;
		offset.y = this->LinkZero.ExitPoint.y;
		offset.z = this->LinkZero.ExitPoint.z;
		return offset;
	}
	else
	{
		offset.x = this->links[index].ExitPoint.x + this->CalcLinkFullOffset(index - 1).x;
		offset.y = this->links[index].ExitPoint.y + this->CalcLinkFullOffset(index - 1).y;
		offset.z = this->links[index].ExitPoint.z + this->CalcLinkFullOffset(index - 1).z;
		return offset;
	}
}

/**
 * Calculate the DH points for the robot arm using links info
 */
void RobotArm::CalcDHPoints()
{
	DirectPoint NewPoint;
	switch (this->LinkZero.ExitJointOrientation)
	{
	case X_Axis:
		NewPoint << 0, 0, 1, 0,
					1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 0, 1;
		break;
	case Y_Axis:
		NewPoint << 0, 1, 0, 0,
					0, 0, 1, 0,
					1, 0, 0, 0,
					0, 0, 0, 1;
		break;
	case Z_Axis:
		NewPoint << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
		break;
	default:
		break;
	}
	this->DHPoints.push_back(NewPoint);

	int count = this->links.size();
	for (int i = 0; i < count; i++)
	{
		switch (this->links[i].ExitJointOrientation)
		{
		case X_Axis:
			NewPoint << 0, 0, 1, 0,
						1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 0, 1;
			break;
		case Y_Axis:
			NewPoint << 0, 1, 0, 0,
						0, 0, 1, 0,
						1, 0, 0, 0,
						0, 0, 0, 1;
			break;
		case Z_Axis:
			NewPoint << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;
			break;		
		default:
			break;
		}		
		this->DHPoints.push_back(NewPoint);	
	}

	for(int i = count; i > 0; i--)
	{
		RobotLink link = this->links[i];
		RobotLink prevLink = this->links[i - 1];
		double x = link.ExitPoint.x;
		double y = link.ExitPoint.y;
		double z = link.ExitPoint.z;
		Eigen::Vector2d v;
		DirectPoint NewPoint;
		if (link.ExitJointOrientation == prevLink.ExitJointOrientation) // case of parallel axis
		{
			switch (link.ExitJointOrientation)
			{
			case X_Axis:
				v << y, z;
				v.normalize();
				NewPoint <<    0,     0, 1, this->CalcLinkFullOffset(i).x,
					   		v[0], -v[1], 0,	this->CalcLinkFullOffset(i).y,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->DHPoints[i] = NewPoint;
				break;
			case Y_Axis:
				v << x, z;
				v.normalize();				
				NewPoint << v[0], -v[1], 0, this->CalcLinkFullOffset(i).x,
					   		   0,     0, 1,	this->CalcLinkFullOffset(i).y,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->DHPoints[i] = NewPoint;
				break;
			case Z_Axis:
				v << x, y;
				v.normalize();
				NewPoint << v[0], -v[1], 0, this->CalcLinkFullOffset(i).x,
					   		v[1],  v[0], 0,	this->CalcLinkFullOffset(i).y,
					   		   0,     0, 1,	this->CalcLinkFullOffset(i).z,
					   		   0,     0, 0,	1;
				this->DHPoints[i] = NewPoint;
				break;
			default:
				break;
			}
		}
		// cases of nonparallel axis
		else if ((link.ExitJointOrientation == X_Axis) && (prevLink.ExitJointOrientation == Y_Axis))
		{
			DirectPoint NewPoint;
			NewPoint <<  0, 0, 1, this->CalcLinkFullOffset(i-1).x,
					   	 0, 1, 0, this->CalcLinkFullOffset(i).y,
					   	-1, 0, 0, this->CalcLinkFullOffset(i).z,
					   	 0, 0, 0, 1;		
			this->DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Y_Axis) && (prevLink.ExitJointOrientation == X_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 0, 1, 0, this->CalcLinkFullOffset(i).x,
					   	0, 0, 1, this->CalcLinkFullOffset(i-1).y,
					   	1, 0, 0, this->CalcLinkFullOffset(i).z,
					   	0, 0, 0, 1;
			this->DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Y_Axis) && (prevLink.ExitJointOrientation == Z_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << -1, 0, 0, this->CalcLinkFullOffset(i).x,
						 0, 0, 1, this->CalcLinkFullOffset(i-1).y,
						 0, 1, 1, this->CalcLinkFullOffset(i).z,
						 0, 0, 0, 1;
			this->DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Z_Axis) && (prevLink.ExitJointOrientation == Y_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 1, 0, 0, this->CalcLinkFullOffset(i).x,
						0, 1, 0, this->CalcLinkFullOffset(i).y,
						0, 0, 1, this->CalcLinkFullOffset(i-1).z,
						0, 0, 0, 1;
			this->DHPoints[i] = NewPoint;
		}
		else if ((link.ExitJointOrientation == Z_Axis) && (prevLink.ExitJointOrientation == X_Axis))
		{
			DirectPoint NewPoint;
			NewPoint <<  0, 1, 0, this->CalcLinkFullOffset(i).x,
						-1, 0, 0, this->CalcLinkFullOffset(i).y,
						 0, 0, 1, this->CalcLinkFullOffset(i-1).z,
						 0, 0, 0, 1;
			this->DHPoints[i] = NewPoint;

		}
		else if ((link.ExitJointOrientation == X_Axis) && (prevLink.ExitJointOrientation == Z_Axis))
		{
			DirectPoint NewPoint;
			NewPoint << 0, 0, 1, this->CalcLinkFullOffset(i-1).x,
					   	1, 0, 0, this->CalcLinkFullOffset(i).y,
					   	0, 1, 0, this->CalcLinkFullOffset(i).z,
					   	0, 0, 0, 1;
			this->DHPoints[i] = NewPoint;
		}	
	}
}

/**
 * Converts a set of DH points to DH parameters.
 *
 * @throws std::invalid_argument if any axis of the DH points are invalid.
 */
void RobotArm::PointsToParams()
{
	int count = this->DHPoints.size();
	for (int i = 1; i < count; i++)
		try {
			DHParams params = RobotAdditions::CalcDHParams(this->DHPoints[i-1], this->DHPoints[i]);
			this->LinkJointParams.push_back(params);
		}
		catch (const std::invalid_argument& e) {
			throw(e);			
		}
}

/**
 * Calculates the forward kinematics of the robot arm
 *
 * @return The transformation matrix representing the end-effector position
 */
DirectPoint RobotArm::ForwardKinematics(Eigen::VectorXd q)
{
	if (!this->isInitialized)
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		int count = this->LinkJointParams.size();
		for (int i = 0; i < count; i++)
		{
			DHParams params = this->LinkJointParams[i];
			T = T * RobotAdditions::CalcTransposeMatrix(params, q[i], this->joints[i].jointType);
			// T = T * CalcTransposeMatrix(params, this->joints[i].curCoord, this->joints[i].jointType);
		}
		return T;
	}
	else
	{
		std::cout << "RobotArm is not initialized" << std::endl;
		std::cout << "Do you want to initialize it now? (y/n)" << std::endl;
		char answer;
		std::cin >> answer;
		if (answer == 'y')
		{
			this->initialize();
			return this->ForwardKinematics(q);
		}
		else
			return DirectPoint::Identity();
	}
}

/**
 * Initializes the RobotArm.
 *
 * @throws std::exception if one of the DH points is invalid
 */
void RobotArm::initialize()
{
	if (!this->isInitialized)
		try	{
			this->CalcDHPoints();
			this->PointsToParams();
			this->isInitialized = true;
		}
		catch(const std::exception& e)
		{
			std::cerr << "One of the DH points is invalid: "<< e.what() << '\n';
			throw(e);
		}
	else
		std::cout << "RobotArm is already initialized" << std::endl;
}

/**
 * Retrieves the joint angles of the robot arm.
 *
 * @return An Eigen::VectorXd object representing the joint angles
 */
Eigen::VectorXd RobotArm::getJointAngles()
{
	Eigen::VectorXd q(this->joints.size());
	q = Eigen::VectorXd::Zero(6);
	for (int i = 0; i < this->joints.size(); i++)
		q[i] = this->joints[i].curCoord;
	return q;
}

/**
 * Sets the joint angles of the robot arm.
 *
 * @param q An Eigen::VectorXd representing the joint angles.
 */
void RobotArm::setJointAngles(Eigen::VectorXd q)
{
	for (int i = 0; i < this->joints.size(); i++)
		this->joints[i].curCoord = q[i];
}

Eigen::VectorXd RobotArm::solveIK_POS(DirectPoint needPoint, std::string method = "1 - BFGS")
{
	Eigen::VectorXd q = this->getJointAngles();

	std::function<DirectPoint(Eigen::VectorXd)> forwFunct = std::bind(&RobotArm::ForwardKinematics, this, std::placeholders::_1);
    auto forwFunc = forwFunct.target<DirectPoint(*)(Eigen::VectorXd)>();

    // Call the optimization functions
    if (forwFunc)
    {
        switch (method.c_str()[0])
        {
        case '1':
            return MathAdditions::BFGS<DirectPoint>(needPoint, q.size(), *forwFunc,
													RobotAdditions::costRobotFunction,
													RobotAdditions::gradientCostRobotFunction,
													q);
            break;
        default:
            return Eigen::VectorXd::Zero(6);
            break;
        }
    }
    else return Eigen::VectorXd::Zero(6);
}

Eigen::VectorXd RobotArm::solveIK_VEL(Eigen::VectorXd needVelocity)
{
	Eigen::VectorXd q = this->getJointAngles();

	std::function<DirectPoint(Eigen::VectorXd)> forwFunct = std::bind(&RobotArm::ForwardKinematics, this, std::placeholders::_1);
    auto forwFunc = forwFunct.target<DirectPoint(*)(Eigen::VectorXd)>();

    if (forwFunc)
    {
		Eigen::MatrixXd J = RobotAdditions::calcRobotJacobian(*forwFunc, q);
		return J.inverse() * needVelocity;
	}
	else return Eigen::VectorXd::Zero(q.size());
}