#pragma once
#ifndef __CONTROL_MATH_H__
#define __CONTROL_MATH_H__

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.141592653589793238462643383279502884197
#define DEG2RAD PI / 180.0
#define RAD2DEG 180.0 / PI
#define DOFS 9

namespace Math
{
static Eigen::MatrixXd pseudo_inverse_QR(const Eigen::MatrixXd &A)
{
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(A);
    return cod.pseudoInverse();
}

static Eigen::VectorXd numerical_differentiation(Eigen::VectorXd x_pre, Eigen::VectorXd x, double dt)
{
    return (x - x_pre) / dt;
}

static Eigen::VectorXd numerical_integration(Eigen::VectorXd x_pre, Eigen::VectorXd x, double dt)
{
    return x_pre + x * dt;
}

static Eigen::Matrix3d calc_rotation_matrix_from_euler_angle_xyz(Eigen::Vector3d euler)
{
    double Roll = euler(0);
    double Pitch = euler(1);
    double Yaw = euler(2);

    Eigen::Matrix3d R;
    // R_.noalias() = R_roll * R_pitch * R_yaw;
    R = Eigen::AngleAxisd(Roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(Pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(Yaw, Eigen::Vector3d::UnitZ());
    return R;
}

static Eigen::Matrix3d get_skew_matrix(Eigen::Vector3d x)
{
    Eigen::Matrix3d skew;
    skew(0, 1) = -x(2);
    skew(0, 2) = x(1);
    skew(1, 0) = x(2);
    skew(1, 2) = -x(0);
    skew(2, 0) = -x(1);
    skew(2, 1) = x(0);
    return skew;
}

static Eigen::Vector3d calc_rotation_error(Eigen::Matrix3d R, Eigen::Matrix3d R_des)
{
    Eigen::Matrix3d skew_x, skew_y, skew_z;

    Eigen::Vector3d col_x = R.col(0);
    Eigen::Vector3d col_y = R.col(1);
    Eigen::Vector3d col_z = R.col(2);

    skew_x = get_skew_matrix(col_x);
    skew_y = get_skew_matrix(col_y);
    skew_z = get_skew_matrix(col_z);

    Eigen::Vector3d skew_des_x = R_des.col(0);
    Eigen::Vector3d skew_des_y = R_des.col(1);
    Eigen::Vector3d skew_des_z = R_des.col(2);

    Eigen::Vector3d skew_goal_x = skew_x * skew_des_x;
    Eigen::Vector3d skew_goal_y = skew_y * skew_des_y;
    Eigen::Vector3d skew_goal_z = skew_z * skew_des_z;

    Eigen::Vector3d R_err;
    R_err = (skew_goal_x + skew_goal_y + skew_goal_z) * (1.0 / 2.0);
    return R_err;
}

static Eigen::Matrix3d get_transformation_matrix_from_euler_xyz(Eigen::Vector3d euler)
{
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);

    Eigen::Matrix3d T;
    T.setZero();

    T << 1 ,    0      ,         sin(pitch)
       , 0 , cos(roll) , -cos(pitch) * sin(roll)
       , 0 , sin(roll) ,  cos(roll) * cos(pitch);
    return T;
}

static Eigen::MatrixXd get_analytic_from_geometric_Jacobian(Eigen::MatrixXd J, Eigen::Matrix3d I, Eigen::Matrix3d O, Eigen::Matrix3d T)
{
    int row = J.rows();
    int col = J.cols();
    Eigen::MatrixXd J_A;
    J_A.setZero(row, col);
    Eigen::MatrixXd T_RPY;
    T_RPY.setZero(6, 6);
    T_RPY.block<3, 3>(0, 0) = I;
    T_RPY.block<3, 3>(3, 0) = O;
    T_RPY.block<3, 3>(0, 3) = O;
    T_RPY.block<3, 3>(3, 3) = T;
    J_A = T_RPY * J;
    return J_A;
}

static Eigen::Matrix3d get_body_rotation_matrix(double Roll, double Pitch, double Yaw)
{
    Eigen::Matrix3d R_yaw;
    R_yaw.setZero();

    R_yaw(2, 2) = 1.0;
    R_yaw(0, 0) = cos(Yaw);
    R_yaw(0, 1) = -sin(Yaw);
    R_yaw(1, 0) = sin(Yaw);
    R_yaw(1, 1) = cos(Yaw);

    Eigen::Matrix3d R_pitch;
    R_pitch.setZero();

    R_pitch(1, 1) = 1.0;
    R_pitch(0, 0) = cos(Pitch);
    R_pitch(2, 2) = cos(Pitch);
    R_pitch(0, 2) = sin(Pitch);
    R_pitch(2, 0) = -sin(Pitch);

    Eigen::Matrix3d R_roll;
    R_roll.setZero();

    R_roll(0, 0) = 1.0;
    R_roll(1, 1) = cos(Roll);
    R_roll(2, 2) = cos(Roll);
    R_roll(1, 2) = -sin(Roll);
    R_roll(2, 1) = sin(Roll);

    Eigen::Matrix3d RGyro;
    RGyro.noalias() = R_yaw * R_pitch * R_roll;
    return RGyro;
}

}

#endif // __CONTROL_MATH_H__