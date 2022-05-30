#include "mujoco_panda/controller.hpp"

Controller::Controller(const char * node_name)
 : Node(node_name)
{
    cmd_service_ = this->create_service<mujoco_panda::srv::Commands>(
        "cmd",
        std::bind(&Controller::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    initialize();
}

Controller::~Controller()
{
	fs_.close();
}

void Controller::initialize()
{
    hqp_p1_.initialize();
    hqp_p2_.initialize();
    
    time_ = 0.0;
    time_pre_ = 0.0;
    dt_ = 0.002;

    kpj_ = 400.0;
	kdj_ = 40.0;
	kp_ = 400.0;
	kd_ = 40.0;

    // RBDL
    model_.get_model();

    dofs_ = model_.get_dofs();
    model_.initialize();
    RCLCPP_INFO(this->get_logger(), "dof: %d", dofs_);
    trajectory_.initialize(dofs_, dt_);

    q_.setZero(dofs_);
    qdot_.setZero(dofs_);
    tau_.setZero(dofs_);
    qpos_.setZero(dofs_);

    x_.setZero(6);
    xdot_.setZero(6);

    q_des_.setZero(dofs_);
    qdot_des_.setZero(dofs_);
    q_goal_.setZero(dofs_);
    qdot_goal_.setZero(dofs_);
    xddot_star_.setZero(6);
    x_des_.setZero(6);
    xdot_des_.setZero(6);
    x_goal_.setZero(6);
    xdot_goal_.setZero(6);
    x_err_.setZero(6);
    xdot_err_.setZero(6);

    q_goal_(0) = 0.0 * DEG2RAD;
    q_goal_(1) = 0.0 * DEG2RAD;
    q_goal_(2) = 0.0 * DEG2RAD;
    q_goal_(3) = -90.0 * DEG2RAD;
    q_goal_(4) = 0.0 * DEG2RAD;
    q_goal_(5) = 90.0 * DEG2RAD;
    q_goal_(6) = 45.0 * DEG2RAD;
    q_goal_(7) = 0.0;
    q_goal_(8) = 0.0;

	x_goal_(0) = 0.088;
	x_goal_(1) = -9.40479e-17;
	x_goal_(2) = 0.833;
	x_goal_(3) = 180.0 * DEG2RAD;
	x_goal_(4) = 0.0 * DEG2RAD;
	x_goal_(5) = 0.0 * DEG2RAD;

    pos_err_.setZero();
    posdot_err_.setZero();
    ori_err_.setZero();
    oridot_err_.setZero();
    xddot_ref_.setZero(6);
    qdot_ref_.setZero(dofs_);
    lambda_.setZero(6, 6);
    null_space_projection_.setZero(6, 6);

    J_.setZero(6, dofs_);
    J_T_.setZero(dofs_, 6);
    J_des_.setZero(6, dofs_);
    J_T_des_.setZero(dofs_, 6);
    Jdot_qdot_.setZero(6);
    T_.setZero(6, 6);
    J_A_.setZero(6, dofs_);
    J_T_A_.setZero(dofs_, 6);

    R_.setZero();
    R_des_.setZero();
    Rdot_des_.setZero();
    I_dofs_.setIdentity(dofs_, dofs_);
    I_task_.setIdentity(6, 6);
    I_3_.setIdentity();
    O_3_.setZero();

    // HQP
    hqp_p1_.initialize_problem_size(DOFS + 6, 6); //variable size = (joint dof)+(task dof), constraint size =(task dof)
	H1_.setZero(hqp_p1_.num_var_, hqp_p1_.num_var_);
	g1_.setZero(hqp_p1_.num_var_);
	A1_.setZero(hqp_p1_.num_cons_, hqp_p1_.num_var_);
	lbA1_.setZero(hqp_p1_.num_cons_);
	ubA1_.setZero(hqp_p1_.num_cons_);
	lb1_.setZero(hqp_p1_.num_var_);
	ub1_.setZero(hqp_p1_.num_var_);
	hqp_p2_.initialize_problem_size(DOFS + 6 + 3, 6 + 6 + 3); //variable size = (joint dof)+(task dof), constraint size =(1st prioirty task dof)  + (2nd prioirty task dof)
	H2_.setZero(hqp_p2_.num_var_, hqp_p2_.num_var_);
	g2_.setZero(hqp_p2_.num_var_);
	A2_.setZero(hqp_p2_.num_cons_, hqp_p2_.num_var_);
	lbA2_.setZero(hqp_p2_.num_cons_);
	ubA2_.setZero(hqp_p2_.num_cons_);
	lb2_.setZero(hqp_p2_.num_var_);
	ub2_.setZero(hqp_p2_.num_var_);

    // Trajectory
    duration_ = 5.0;
	is_target_reached_ = false;
	is_command_received = false;
	threshold_ = 0.02;

	// file I/O
	filename_ = "/home/gyufoxy/rp_ws/octave/log.txt";
	fs_.open(filename_, std::ios_base::openmode::_S_trunc);

    return;
}

void Controller::update_model()
{
    model_.update_kinematics(q_, qdot_); // TODO: need to get current value of each joint position and velocity (q_, qdot_)
    model_.update_dynamics();
    model_.get_Jacobian();
    model_.get_state();
    J_ = model_.J_;
    J_T_ = J_.transpose();
	Jdot_qdot_ = J_ * qdot_;

    x_.head(3) = model_.pos_;
    x_.tail(3) = model_.ori_;
    R_ = model_.R_;

    xdot_.head(3) = model_.posdot_;
    xdot_.tail(3) = model_.oridot_;

    J_A_ = Math::get_analytic_from_geometric_Jacobian(J_, I_3_, O_3_, Math::get_transformation_matrix_from_euler_xyz(x_.tail(3)));

	// std::cout << "goal: " << x_goal_.head(3).transpose() << "\tcur: " << x_.head(3).transpose() << '\n';
	if ((std::abs(x_(0) - x_goal_(0)) < threshold_) && (std::abs(x_(1) - x_goal_(1)) < threshold_) && (std::abs(x_(2) - x_goal_(2)) < threshold_))
	{
		RCLCPP_INFO(this->get_logger(), "Target reached!");
		is_target_reached_ = true;
	}
    return;
}

void Controller::plan_trajectory()
{
    if (trajectory_.is_traj_finished() || (is_command_received == true))
    {
		
        trajectory_.check_size(x_);
        trajectory_.set_start(x_, xdot_, time_); // TODO: set time_
        trajectory_.set_goal(x_goal_, xdot_goal_, time_ + duration_); // TODO: set x_goal, xdot_goal
		is_command_received = false;
		is_target_reached_ = false;
		std::cout << "goal: " << x_goal_.transpose() << '\t' << time_ << '\n';
    }
    trajectory_.update_time(time_);
    x_des_ = trajectory_.get_position_trajectory();
    xdot_des_ = trajectory_.get_orientation_trajectory();
	fs_ << time_ << '\t' << x_des_(0) << '\t' << x_des_(1) << '\t' << x_des_(2) << '\t' << x_(0) << '\t' << x_(1) << '\t' << x_(2) << '\n';
	// std::cout << x_des_.transpose() << '\n';
}

void Controller::calculate_command()
{
    tau_.setZero();

    // TODO: get desired X, Xdot (x_des_, xdot_des_ (6x1))
    kp_ = 400.0;
	kd_ = 20.0;

	x_err_.head(3) = x_des_.head(3) - x_.head(3);
	R_des_ = Math::get_body_rotation_matrix(x_des_(3), x_des_(4), x_des_(5));	
	x_err_.tail(3) = Math::calc_rotation_error(R_, R_des_);

	xdot_err_.head(3) = xdot_des_.head(3) - xdot_.head(3);
	xdot_err_.tail(3) = -xdot_.tail(3); //only damping for orientation	

	xddot_star_.segment(0, 3) = kp_ * x_err_.head(3) + kd_ * xdot_err_.head(3);// position control
	xddot_star_.segment(3, 3) = kp_ * x_err_.tail(3) + kd_ * xdot_err_.tail(3);// orientation control

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Solve rHQP   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double threshold = 0.001;
	int max_iter = 1000;
	// first priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// set cost function x^T*H*x + g
	H1_.setZero();
	for (int i = 0; i < DOFS; i++) // torque
	{
		H1_(i, i) = 1.0;
	}
	for (int i = DOFS; i < DOFS + 6; i++) // slack
	{
		H1_(i, i) = 1000000.0;
	}
	g1_.setZero();
	hqp_p1_.update_min_problem(H1_, g1_);

	Eigen::MatrixXd J_Ainv(6, DOFS);
	J_Ainv = J_ * model_.A_.inverse();

	// set A*x <= b
	A1_.setZero();
	lbA1_.setZero();
	ubA1_.setZero();
	A1_.block<6, DOFS>(0, 0) = J_Ainv;
	A1_.block<6, 6>(0, DOFS) = -I_task_;

	for (int i = 0; i < 6; i++)
	{
		lbA1_(i) = -Jdot_qdot_(i) + xddot_star_(i) - threshold;
		ubA1_(i) = -Jdot_qdot_(i) + xddot_star_(i) + threshold;
	}
	hqp_p1_.update_subject_to_Ax(A1_, lbA1_, ubA1_);

	// set lb <= x <= ub
	lb1_.setZero();
	ub1_.setZero();
	// joint torque limit
	for (int i = 0; i < DOFS; i++)
	{
		// torque limit
		//lb1_(i) = Model._min_joint_torque(i) - Model._bg(i);
		//ub1_(i) = Model._max_joint_torque(i) - Model._bg(i);
		lb1_(i) = model_.min_joint_torque_(i);
		ub1_(i) = model_.max_joint_torque_(i);
	}
	lb1_(7) = 0.0 - threshold;
	ub1_(7) = 0.0 + threshold;
	lb1_(8) = 0.0 - threshold;
	ub1_(8) = 0.0 + threshold;
	// NOTE: verify an effect of decreasing the wrist joint inequality constraints
	// lb1_(6) = -1.0-Model._bg(6);
	// ub1_(6) = 1.0-Model._bg(6);
	// lb1_(13) = -1.0-Model._bg(13);
	// ub1_(13) = 1.0-Model._bg(13);
	// task limit
	for (int i = 0; i < 6; i++)
	{
		lb1_(i + DOFS) = -1000000.0;
		ub1_(i + DOFS) = 1000000.0;
	}
	hqp_p1_.update_subject_to_X(lb1_, ub1_);

	// Solve
	hqp_p1_.enable_equality_condition(0.0001);
	hqp_p1_.solve_qpOASES(max_iter);
	//_torque = hqp_p1_._Xopt.segment(0, 15) + Model._bg;

	// second priority task QP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// set cost function x^T*H*x + g
	H2_.setZero();
	for (int i = 0; i < DOFS; i++) // torque
	{
		H2_(i, i) = 1.0;
	}
	for (int i = DOFS; i < DOFS + 6 + 3; i++) // slack
	{
		H2_(i, i) = 1000000.0;
	}
	H2_(7, 7) = 0.0001;
	H2_(8, 8) = 0.0001;
	//H2_.block<15, 15>(0, 0) = Model._A.inverse();
	//H2_.block<15, 15>(15, 15) = Model._A;
	g2_.setZero();
	hqp_p2_.update_min_problem(H2_, g2_);

	// set A*x <= b
	A2_.setZero();
	lbA2_.setZero();
	ubA2_.setZero();
	A2_.block<DOFS, DOFS>(0, 0) = model_.A_.inverse();
	A2_.block<DOFS, DOFS>(0, DOFS) = -I_dofs_;
	A2_.block<6, DOFS>(DOFS, 0) = J_Ainv;

	Eigen::VectorXd joint_acc_des(DOFS);
	joint_acc_des.setZero();
	kdj_ = 20.0;
	kpj_ = 100.0;
	joint_acc_des = kpj_ * ((model_.max_joint_position_ + model_.min_joint_position_) / 2.0 - q_) - kdj_ * qdot_;
	// joint_acc_des = -_kdj * _qdot;
	joint_acc_des(7) = 0.0;
	joint_acc_des(8) = 0.0;
	// joint_acc_des(0) = 400.0 * (_q_home(0)- _q(0)) - _kdj * _qdot(0);

	for (int i = 0; i < DOFS; i++)
	{
		lbA2_(i) = joint_acc_des(i) - threshold;
		ubA2_(i) = joint_acc_des(i) + threshold;
	}
	for (int i = 0; i < 6; i++)
	{
		lbA2_(i + DOFS) = -Jdot_qdot_(i) + xddot_star_(i) + hqp_p1_.X_opt_(i + DOFS) - threshold;
		ubA2_(i + DOFS) = -Jdot_qdot_(i) + xddot_star_(i) + hqp_p1_.X_opt_(i + DOFS) + threshold;
	}
	hqp_p2_.update_subject_to_Ax(A2_, lbA2_, ubA2_);

	// set lb <= x <= ub
	lb2_.setZero();
	ub2_.setZero();

	// joint torque limit
	for (int i = 0; i < DOFS; i++)
	{
		//lb2_(i) = Model._min_joint_torque(i) - Model._bg(i);
		//ub2_(i) = Model._max_joint_torque(i) - Model._bg(i);
		lb2_(i) = model_.min_joint_torque_(i) * 2.0;
		ub2_(i) = model_.max_joint_torque_(i) * 2.0;
	}
	lb2_(7) = 0.0 - threshold;
	ub2_(7) = 0.0 + threshold;
	lb2_(8) = 0.0 - threshold;
	ub2_(8) = 0.0 + threshold;
	// NOTE: verify an effect of decreasing the wrist joint inequality constraints
	// lb2_(6) = -1.0-Model._bg(6);
	// ub2_(6) = 1.0-Model._bg(6);
	// lb2_(13) = -1.0-Model._bg(13);
	// ub2_(13) = 1.0-Model._bg(13);
	// task limit
	for (int i = 0; i < DOFS; i++)
	{
		lb2_(i + DOFS) = -1000000.0;
		ub2_(i + DOFS) = 1000000.0;
	}
	hqp_p2_.update_subject_to_X(lb2_, ub2_);

	// Solve
	// hqp_p2_.EnableEqualityCondition(0.0001);
	hqp_p2_.solve_qpOASES(max_iter);

	if (hqp_p1_.num_state_ == 0 || hqp_p2_.num_state_ == 0)
	{
		tau_ = hqp_p2_.X_opt_.segment(0, DOFS) + model_.bg_;
		// cout << "------------- torque ---------------------\n" << hqp_p2_._Xopt.segment(0, 15). transpose() + Model._bg.transpose() << endl;
	}
	else // when solving HQP failed
	{
		std::cout << "Fault: Cannot solve QP!!" << '\n';
	}
}

void Controller::get_state(double t, double * q, double * qdot)
{
    time_pre_ = time_;
    time_ = t;
    dt_ = time_ - time_pre_;
    for (int i = 0; i < dofs_; i++)
    {
        q_(i) = q[i];
        qdot_(i) = qdot[i];
    }
    return;
}

void Controller::update()
{
    update_model();
    plan_trajectory();
    if (is_target_reached_ == false) calculate_command();
}

void Controller::set_command(double * tau, double * qpos)
{
    if(is_target_reached_ == true) tau_ = model_.bg_;
    for (int i = 0; i < dofs_; i++) tau[i] = tau_(i);
    return;
}

Eigen::Vector3d Controller::get_current()
{
	return x_.head(3);
}


Eigen::Vector3d Controller::get_target()
{
	return x_goal_.head(3);
}

void Controller::cmd_callback(const std::shared_ptr<mujoco_panda::srv::Commands::Request> request,
                    std::shared_ptr<mujoco_panda::srv::Commands_Response> response)
{
    x_goal_(0) = request->x;
	x_goal_(1) = request->y;
	x_goal_(2) = request->z;
	x_goal_(3) = request->roll;
	x_goal_(4) = request->pitch;
	x_goal_(5) = request->yaw;
    RCLCPP_INFO(this->get_logger(), "Received command request: [%f, %f, %f, %f, %f, %f]",
                x_goal_(0),
                x_goal_(1),
                x_goal_(2),
                x_goal_(3),
                x_goal_(4),
                x_goal_(5));
	response->finished = true;
	is_command_received = response->finished;
    return;
}