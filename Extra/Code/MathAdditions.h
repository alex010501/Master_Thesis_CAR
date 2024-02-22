#pragma once

#include <Eigen/Dense>

#define PI 3.141592653589793

namespace MathAdditions
{
    template <typename X>
    using forwardFunc = X (*)(Eigen::VectorXd x);

    template <typename X>
    using errorFunc = Eigen::VectorXd (*)(X a, X b);

    double DegToRad(double deg);

    double RadToDeg(double rad);

    double projVector(Eigen::Vector3d a, Eigen::Vector3d b);

    double getAngleAroundAxis(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d Axis);

    double getAngle(Eigen::Vector3d a, Eigen::Vector3d b);

    Eigen::Matrix3d Rx(double angle);

    Eigen::Matrix3d Ry(double angle);

    Eigen::Matrix3d Rz(double angle);

    template <typename T>
    Eigen::MatrixXd calcJacobian(forwardFunc<T> forwFunc, errorFunc<T> errFunc, Eigen::VectorXd x_init, int num_DOF, double eps = 1e-6);

    template <typename T>
    Eigen::VectorXd BFGS(T target, int num_DOF,
                         forwardFunc<T> forwFunc,
                         double (*f)(forwardFunc<T>, Eigen::VectorXd q, T target),
                         Eigen::VectorXd (*df)(forwardFunc<T>, Eigen::VectorXd q, T target),
                         Eigen::VectorXd x_init,
                         double eps = 1e-6, double alpha = 0.01, int max_iterations = 100);
}