#pragma once
#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "control_math.h"

class Trajectory
{
public:
    explicit Trajectory();
    virtual ~Trajectory();
    void initialize(int dofs, double dt);
    void set_default(int dofs, double dt);
    void set_start(Eigen::VectorXd x_start, Eigen::VectorXd xdot_start, double start_time);
    void set_goal(Eigen::VectorXd x_goal, Eigen::VectorXd xdot_goal, double goal_time);
    void check_size(Eigen::VectorXd x);
    void update_time(double time);
    void reset_target();
    void get_cmd_count(int cmd_count);
    bool is_traj_finished();

    Eigen::VectorXd get_position_trajectory();
    Eigen::VectorXd get_orientation_trajectory();
private:
    int dofs_, size_;
    int cmd_count_;
    double dt_, time_;
    double start_time_, goal_time_;

    Eigen::VectorXd x_start_, x_goal_;
    Eigen::VectorXd xdot_start_, xdot_goal_;
    Eigen::VectorXd bool_cmd_;
};

#endif // __TRAJECTORY_H__