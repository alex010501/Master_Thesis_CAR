#include <MathAdditions.h>
#include <iostream>

/**
 * Converts degrees to radians.
 *
 * @param deg the value in degrees to be converted
 *
 * @return the value in radians
 */
double MathAdditions::DegToRad(double deg)
{
    return deg * PI / 180.0;
}

/**
 * Converts radians to degrees.
 *
 * @param rad the value in radians to be converted
 *
 * @return the value in degrees
 */
double MathAdditions::RadToDeg(double rad)
{
    return rad * 180.0 / PI;
}

/**
 * Calculates the projection of vector `a` onto vector `b`.
 *
 * @param a The first vector.
 * @param b The second vector.
 *
 * @return The projection of vector `a` onto vector `b`.
 * 
 * @throws std::invalid_argument if the second vector is null.
 */
double MathAdditions::projVector(Eigen::Vector3d a, Eigen::Vector3d b)
{
    if (b.norm() == 0)
    {
        throw std::invalid_argument("Vector b must not be null");
    }
    return a.dot(b) / b.norm();
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @param Axis The axis around which the angle is calculated.
 *
 * @return The angle between the vectors in radians with direction of rotation.
 *
 * @throws std::invalid_argument if either of the vectors is null or one of the vectors is parallel to the axis.
 */
double MathAdditions::getAngleAroundAxis(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d Axis)
{
    if (Axis.norm() == 0)
        throw std::invalid_argument("Axis must not be null");
    
    Axis.normalize();
    a = a - MathAdditions::projVector(a, Axis) * Axis;
    b = b - MathAdditions::projVector(b, Axis) * Axis;
    Eigen::Vector3d c = a.cross(b);
    if (a.norm() == 0 || b.norm() == 0)
        throw std::invalid_argument("Vectors must not be parallel to axis");
    else if (a.norm()*b.norm() == a.dot(b))
        return 0;
    else if (a.norm()*b.norm() == -a.dot(b))
        return PI;
    else if (MathAdditions::projVector(a, b) == 0)
        if (MathAdditions::projVector(c, Axis) > 0)
            return PI/2;
        else
            return -PI/2;
    else
        if (MathAdditions::projVector(c, Axis) > 0)
            return acos(a.dot(b) / (a.norm() * b.norm()));
        else
            return -acos(a.dot(b) / (a.norm() * b.norm()));
}

/**
 * Calculates the angle between two vectors around a given axis.
 *
 * @param a The first vector.
 * @param b The second vector.
 *
 * @return The angle between the vectors in radians.
 *
 * @throws std::invalid_argument if either of the vectors is null.
 */
double MathAdditions::getAngle(Eigen::Vector3d a, Eigen::Vector3d b)
{
    if (a.norm() == 0 || b.norm() == 0)
        throw std::invalid_argument("Vectors must not be null"); 
    else if (a.norm()*b.norm() == a.dot(b))
        return 0;
    else if (a.norm()*b.norm() == -a.dot(b))
        return PI;
    else if (MathAdditions::projVector(a, b) == 0)
        return PI/2;
    else
        return acos(a.dot(b) / (a.norm() * b.norm()));
}

/**
 * Generates a 3x3 rotation matrix around the x-axis
 *
 * @param angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rx(double angle)
{
    Eigen::Matrix3d Rx;
    Rx << 1,          0,           0,
          0, cos(angle), -sin(angle),
          0, sin(angle),  cos(angle);
    return Rx;
}

/**
 * Generates a 3x3 rotation matrix around the y-axis
 *
 * @param angle The angle of rotation in radians
 *
 * @return The rotation matrix.
 */
Eigen::Matrix3d MathAdditions::Ry(double angle)
{
    Eigen::Matrix3d Ry;
    Ry << cos(angle), 0, sin(angle),
                   0, 1,          0,
         -sin(angle), 0, cos(angle);
    return Ry;
}

/**
 * Generates a 3x3 rotation matrix around the z-axis
 *
 * @param angle The angle of rotation in radians
 *
 * @return The rotation matrix
 */
Eigen::Matrix3d MathAdditions::Rz(double angle)
{
    Eigen::Matrix3d Rz;
    Rz << cos(angle), -sin(angle), 0,
          sin(angle),  cos(angle), 0,
                   0,           0, 1;
    return Rz;
}

/**
 * Calculates the Jacobian matrix for a given forward function and error function.
 *
 * @tparam T the type of the output of the forward function and the input of the error function
 * 
 * @param forwFunc a pointer to the forward function
 * @param errFunc a pointer to the error function
 * @param x_init the initial vector
 * @param num_DOF the number of degrees of freedom
 * @param eps the epsilon value (default: 1e-6)
 *
 * @return the Jacobian matrix
 *
 */
template <typename T>
Eigen::MatrixXd MathAdditions::calcJacobian(forwardFunc<T> forwFunc, // Forward function
                                            errorFunc<T> errFunc, // Error function
                                            Eigen::VectorXd x_init, int num_DOF, double eps)
// Eigen::MatrixXd calcJacobian(forwardFunc<T> forwFunc, Eigen::VectorXd (*errFunc)(T a, T b), Eigen::VectorXd x_init, int num_DOF, double eps = 1e-6)
{
	int vectorSize = x_init.size();	

	Eigen::MatrixXd J(num_DOF, vectorSize);

	for (int i = 0; i < vectorSize; i++)
	{
		// Calculate forward kinematics for q + delta_q
        Eigen::VectorXd x_plus = x_init;
        x_plus(i) += eps;
        T T_plus = forwFunc(x_plus);

		// Calculate forward kinematics for q - delta_q
		Eigen::VectorXd x_minus = x_init;
        x_minus(i) -= eps;
        T T_minus = forwFunc(x_minus);

		// Calculate partial derivative
		Eigen::VectorXd derivative = errFunc(T_plus, T_minus) / (2 * eps);

        // Add to Jacobian matrix
		// J.block<num_DOF,1>(0,i) = derivative;
        J.col(i) = derivative;       
	}
	
	return J;
}

/**
 * This function implements the Broyden-Fletcher-Goldfarb-Shanno (BFGS) optimization algorithm
 * to find the minimum of a given cost function. It iteratively updates the Hessian approximation
 * to approximate the inverse of the true Hessian matrix, and calculates the search direction
 * using the updated Hessian approximation and the gradient of the cost function.
 *
 * @tparam T The type of the target value and forward function.
 * @param target The target value.
 * @param num_DOF The number of degrees of freedom.
 * @param forwFunc The forward function.
 * @param f The cost function.
 * @param df The gradient of the cost function.
 * @param x_init The initial guess. Default is a zero vector of size num_DOF.
 * @param eps The tolerance. Default is 1e-6.
 * @param alpha The step size. Default is 0.01.
 * @param max_iterations The maximum number of iterations. Default is 100.
 * @return The optimized vector x that minimizes the cost function.
 * 
 * @throws std::invalid_argument if the initial guess is not of size num_DOF.
 */
template <typename T>
Eigen::VectorXd MathAdditions::BFGS(T target, int num_DOF, // Target value, number of degrees of freedom
                                    forwardFunc<T> forwFunc, // Forward function
                                    double (*f)(forwardFunc<T>, Eigen::VectorXd q, T target), // Cost function
                                    Eigen::VectorXd (*df)(forwardFunc<T>, Eigen::VectorXd q, T target), // Gradient of cost function
                                    Eigen::VectorXd x_init, // Initial guess
                                    double eps, double alpha, int max_iterations) // Tolerance, step size, max iterations
{

    if (x_init.size() != num_DOF) {
        throw std::invalid_argument("x_init must be of size num_DOF");
    }

    // Initial guess
    Eigen::VectorXd x = x_init;

    // Initial Hessian approximation
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(x.size(), x.size());


    // BFGS iterations
    for (int i = 0; i < max_iterations; ++i) {
        // Calculate search direction
        Eigen::VectorXd p = -H * df(forwFunc, x, target);

        // Update x for comparison
        Eigen::VectorXd x_new = x + alpha * p;

        // Check for convergence
        if ((x_new - x).norm() < eps) {
            break;
        }

        // Update Hessian approximation
        Eigen::VectorXd s = x_new - x;
        Eigen::VectorXd y = df(forwFunc, x_new, target) - df(forwFunc, x, target);
        double rho = 1 / y.dot(s);
        H = (Eigen::MatrixXd::Identity(x.size(), x.size()) - rho * s * y.transpose()) * H
            * (Eigen::MatrixXd::Identity(x.size(), x.size()) - rho * y * s.transpose())
            + rho * s * s.transpose();

        // Update x
        x = x_new;
    }

    return x;
}