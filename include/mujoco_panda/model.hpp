#pragma once
#ifndef __MODEL_HPP__
#define __MODEL_HPP__

// 3rd-party dependencies
#include <rbdl/rbdl.h>
#include <rbdl/Body.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "control_math.hpp"

class Model
{
public:
    explicit Model();
    virtual ~Model();
    void initialize();
    int get_model();
    int update_kinematics(Eigen::VectorXd &q, Eigen::VectorXd &qdot);
    int update_dynamics();
    int get_Jacobian();
    int get_state();
    double get_dofs();
    Eigen::Vector3d get_desired_position_from_joint_angle(Eigen::VectorXd q);
    Eigen::Vector3d get_desired_orientation_from_joint_angle(Eigen::VectorXd q);
    Eigen::MatrixXd get_desired_Jacobian_from_joint_angle(Eigen::VectorXd q);

public:
    RigidBodyDynamics::Model rbdl_model_;
    Eigen::VectorXd q_, qdot_; // joint angle and joint angular velocity of the model
    Eigen::Vector3d pos_, posdot_, ori_, oridot_; // position and orientation of the model
    Eigen::VectorXd bg_, b_, g_; // b = Coriolis and centrifugal force, g = gravitational force, bg = b + g 
    Eigen::MatrixXd A_, J_, J_des_; // inertial matrix and jacobian matrix of the model
    Eigen::Matrix3d R_; // rotation matrix of end-effector of the model
    Eigen::VectorXd min_joint_position_, max_joint_position_; // minimum and maximum joint position from model
    Eigen::VectorXd min_joint_torque_, max_joint_torque_; // minimum and maximum joint torque of model

private:
    void configurate_body();
    
    Eigen::VectorXd body_point_local_ee_; // translation of the end-effector calculated from the final joint coordinate
    Eigen::VectorXd zero_; // 3x1 zero column vector

    int dofs_;
    int ee_id_;
    bool bool_update_kinemtaics_, bool_update_dynamics_, bool_get_state_, bool_get_jacobian_;
};

#endif // __MODEL_HPP__