#pragma once
#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

// 3rd-party API & custom header dependencies
#include "control_math.h"
#include "model.h"
#include "trajectory.h"
#include "quadratic_programming.h"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "mujoco_panda/srv/commands.hpp"

// logging
#include <fstream>

class Controller : public rclcpp::Node
{
public:
    Controller() = delete;
    Controller(const char * node_name);
    ~Controller();

    void initialize();
    void update_model();
    void plan_trajectory();
    void calculate_command();
    void get_state(double t, double * q, double * qdot);
    void update();
    void set_command(double * tau, double * qpos);
    Eigen::Vector3d get_current();
    Eigen::Vector3d get_target();

private:
    // RBDL
    Model model_;
    Eigen::VectorXd q_, qdot_; // angle and angular velocities of joints of the panda
    Eigen::VectorXd tau_, qpos_; // torque of joints of the panda
    Eigen::VectorXd x_, xdot_; // poses and velocities of end-effector of the panda

    Eigen::VectorXd q_des_, qdot_des_; // desired joint space angle and joint angular velocity of panda
    Eigen::VectorXd q_goal_, qdot_goal_; // goal joint space angle and joint angular velocity of panda
    Eigen::VectorXd xddot_star_; // desired task space position and orientation of panda
    Eigen::VectorXd x_des_, xdot_des_; // desired task space position and orientation of panda
    Eigen::VectorXd x_goal_, xdot_goal_; // goal position and orientation which is obtained from the planner
    Eigen::VectorXd x_err_, xdot_err_; // error of the position and orientation

    Eigen::Vector3d pos_err_, posdot_err_;
    Eigen::Vector3d ori_err_, oridot_err_;
    Eigen::VectorXd xddot_ref_, qdot_ref_;
    Eigen::MatrixXd lambda_, null_space_projection_;

    Eigen::MatrixXd J_, J_T_, J_des_, J_T_des_; // jacobian, jacobian transpose matrices of the panda
    Eigen::VectorXd Jdot_qdot_; // Xdot = Jdot * qdot matrices of the panda
    Eigen::MatrixXd T_, J_A_, J_T_A_; // transformation matrix from geometric to analytic jacobian, analytic jacobian and jacobian transpose matices
    Eigen::Matrix3d R_, R_des_, Rdot_des_; // rotation matrix(current, desired, dot desired) of end-effector of the panda
    Eigen::MatrixXd I_dofs_, I_task_; // identity matrix DoFs x DoFs and DoFs of task x DoFs of task
    Eigen::Matrix3d I_3_, O_3_; // identity matrix 3x3 and zero matrix 3x3

    // HQP
    QuadraticProgram hqp_p1_, hqp_p2_;
    //first priority task - franka control, second priority task - joint damping
    Eigen::MatrixXd H1_, H2_, A1_, A2_;
	Eigen::VectorXd g1_, g2_, lbA1_, lbA2_, ubA1_, ubA2_, lb1_, lb2_, ub1_, ub2_;

    // Trajectory
    Trajectory trajectory_;

private:
    bool is_target_reached_; // true when the command and the state of robot is same
    bool is_command_received; // true when received the command. false after that
    double threshold_; // threshold value of target reached
    double time_, time_pre_; // current time which is obtained from the mujoco simulator
    double duration_; // time of trajectory interpolation
    double dt_;
    int dofs_; // degrees of the freedom
    double kpj_; //joint control P gain
	double kdj_; //joint control D gain	
	double kp_; //Operational space control P gain
	double kd_; //Operational space control D gain

private:
    void cmd_callback(const std::shared_ptr<mujoco_panda::srv::Commands::Request> request,
                    std::shared_ptr<mujoco_panda::srv::Commands_Response> response);

private:
    rclcpp::Service<mujoco_panda::srv::Commands>::SharedPtr cmd_service_;

// file I/O
private:
    std::fstream fs_;
    std::string filename_;
};

#endif // __CONTROLLER_HPP__