#include "mujoco_panda/trajectory.hpp"

Trajectory::Trajectory()
{
}

Trajectory::~Trajectory()
{
}

void Trajectory::set_default(int dofs, double dt)
{
    dofs_ = dofs;
    dt_ = dt;
}

void Trajectory::set_start(Eigen::VectorXd x_start, Eigen::VectorXd xdot_start, double start_time)
{
    x_start_ = x_start;
    xdot_start_ = xdot_start;
    start_time_ = start_time;
}

void Trajectory::set_goal(Eigen::VectorXd x_goal, Eigen::VectorXd xdot_goal, double goal_time)
{
    x_goal_ = x_goal;
    xdot_goal_ = xdot_goal;
    goal_time_ = goal_time;
}

void Trajectory::check_size(Eigen::VectorXd x)
{
    size_ = x.size();
    x_start_.resize(size_);
    x_goal_.resize(size_);
    xdot_start_.resize(size_);
    xdot_goal_.resize(size_);
}

void Trajectory::update_time(double time)
{
    time_ = time;
}

void Trajectory::reset_target()
{
    if (bool_cmd_(cmd_count_) == 0)
    {
        goal_time_ = 0.0;
        start_time_ = 0.0; 
    }
}

void Trajectory::get_cmd_count(int cmd_count)
{
    cmd_count_ = cmd_count;
    return;
}

bool Trajectory::is_traj_finished()
{
    // if (time_ == 0)
    // {
    //     return true;
    // }
    // else 
    if (time_ >= goal_time_ && bool_cmd_(cmd_count_) == 0)
    {
        bool_cmd_(cmd_count_) = 1;
        return true;
    }
    else
    {
        return false;
    }
}

Eigen::VectorXd Trajectory::get_position_trajectory()
{
    Eigen::VectorXd x(size_);
    if (time_ <= start_time_)
    {
        x = x_start_;
    }
    else if (time_ >= goal_time_)
    {
        x = x_goal_;
    }
    else
    {
        x = x_start_ + xdot_start_ * (time_ - start_time_)
			+ (3.0 * (x_goal_ - x_start_) / (std::pow((goal_time_ - start_time_), 2)) - 2.0 * xdot_start_ / (goal_time_ - start_time_) - xdot_goal_ / (goal_time_ - start_time_)) * std::pow((time_ - start_time_), 2)
			+ (-2.0 * (x_goal_ - x_start_) / (std::pow((goal_time_ - start_time_), 3)) + (xdot_start_ + xdot_goal_) / (std::pow((goal_time_ - start_time_), 2))) * std::pow((time_ - start_time_), 3);
    }
    return x;
}

Eigen::VectorXd Trajectory::get_orientation_trajectory()
{
    Eigen::VectorXd xdot(size_);
    if (time_ <= start_time_)
    {
        xdot = xdot_start_;
    }
    else if (time_ >= goal_time_)
    {
        xdot = xdot_goal_;
    }
    else
    {
        xdot = xdot_start_
            + 2.0 * (3.0 * (x_goal_ - x_start_) / (std::pow((goal_time_ - start_time_), 2)) - 2.0 * xdot_start_ / (goal_time_ - start_time_) - xdot_goal_ / (goal_time_ - start_time_)) * (time_ - start_time_)
			+ 3.0 * (-2.0 * (x_goal_ - x_start_) / (std::pow((goal_time_ - start_time_), 3)) + (xdot_start_ + xdot_goal_) / (std::pow((goal_time_ - start_time_), 2))) * std::pow((time_ - start_time_), 2);
    }
    return xdot;
}

void Trajectory::initialize(int dofs, double dt)
{
    dofs_ = dofs;
    size_ = 6;
    cmd_count_ = 0;
    dt_ = dt;
    time_ = 0.0;
    start_time_ = 0.0;
    goal_time_ = 0.0;

    x_start_.setZero(size_);
    x_goal_.setZero(size_);
    xdot_start_.setZero(size_);
    xdot_goal_.setZero(size_);
    bool_cmd_.setZero(100); // max size of command buffer
}