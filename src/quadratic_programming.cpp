#include "mujoco_panda/quadratic_programming.hpp"

QuadraticProgram::QuadraticProgram()
{
}
QuadraticProgram::~QuadraticProgram()
{
}

void QuadraticProgram::initialize()
{
    num_var_ = 1;
    num_cons_ = 1;
    initialize_problem_size(num_var_, num_cons_);

}

void QuadraticProgram::initialize_problem_size(const int& num_var, const int& num_cons)
{
    SQP_prob_ = qpOASES::SQProblem(num_var, num_cons);
    bool_constraint_Ax_ = false;
    bool_constraint_x_ = false;
    num_var_ = num_var;
    num_cons_ = num_cons;
    H_.resize(num_var_, num_var_);
    H_.setZero();
    g_.resize(num_var_);
    g_.setZero();
    A_.resize(num_cons_, num_var_);
    A_.setZero();
    lbA_.resize(num_cons_);
    lbA_.setZero();
    ubA_.resize(num_cons_);
    ubA_.setZero();
    lb_.resize(num_var_);
    lb_.setZero();
    ub_.resize(num_var_);
    ub_.setZero();
    b_initialized_ = false;
    num_state_ = 100;
    X_opt_.resize(num_var_);
    X_opt_.setZero();
    comp_time_ = 100000.0;
    options_.printLevel = qpOASES::PL_NONE;
}


void QuadraticProgram::update_min_problem(const Eigen::MatrixQPd& H, const Eigen::VectorQPd& g)
{
    for (int i = 0; i < num_var_; i++)
    {
        for (int j = 0; j < num_var_; j++)
        {
            H_(i, j) = H(i, j);
        }
        g_(i) = g(i);
    }
}

void QuadraticProgram::update_subject_to_Ax(const Eigen::MatrixQPd& A, const Eigen::VectorQPd& lbA, const Eigen::VectorQPd& ubA)
{
    for (int i = 0; i < num_cons_; i++)
    {
        for (int j = 0; j < num_var_; j++)
        {
            A_(i, j) = A(i, j);
        }
        lbA_(i) = lbA(i);
        ubA_(i) = ubA(i);
    }

    bool_constraint_Ax_ = true;

    for (int i = 0; i < num_cons_; i++)
    {
        if (lbA_(i) > ubA_(i))
        {
            std::cout << "-- Error in Constraint Value in lbA <= Ax <= ubA --" << '\n';
            std::cout << "lbA[" << i << "] is bigger than ubA." << '\n';

            bool_constraint_Ax_ = false;
        }
    }

    if (num_cons_ == 0)
    {
        bool_constraint_Ax_ = false;
        std::cout << "-- Number of Constraint is zero. --" << '\n';
    }
}

void QuadraticProgram::update_subject_to_X(const Eigen::VectorQPd& lb, const Eigen::VectorQPd& ub)
{
    for (int i = 0; i < num_var_; i++)
    {
        lb_(i) = lb(i);
        ub_(i) = ub(i);
    }
    bool_constraint_x_ = true;

    for (int i = 0; i < num_var_; i++)
    {
        if (lb_(i) > ub_(i))
        {
            std::cout << "-- Error in Constraint Value in lb <= x <= ub --" << '\n';
            std::cout << "lb[" << i << "] is bigger than ub." << '\n';

            bool_constraint_x_ = false;
        }
    }
    if (num_var_ == 0)
    {
        bool_constraint_x_ = false;
        std::cout << "-- Number of Variable is zero. --" << '\n';
    }
}

void QuadraticProgram::delete_subject_to_Ax()
{
    bool_constraint_Ax_ = false;
}

void QuadraticProgram::delete_subject_to_X()
{
    bool_constraint_x_ = false;
}

void QuadraticProgram::Print_min_prob()
{
    std::cout << "------------------------------------------------------------------------------" << '\n';
    std::cout << "----------------------------------    H    -----------------------------------" << '\n';
    std::cout << "------------------------------------------------------------------------------" << '\n';
    std::cout << H_ << '\n';
    std::cout << "------------------------------------------------------------------------------" << '\n';
    std::cout << "----------------------------------    g    -----------------------------------" << '\n';
    std::cout << "------------------------------------------------------------------------------" << '\n';
    std::cout << g_.transpose() << '\n';
    std::cout << "------------------------------------------------------------------------------" << '\n';
}

void QuadraticProgram::Print_subject_to_Ax()
{
    if (bool_constraint_Ax_ == true)
    {
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "----------------------------------    A    -----------------------------------" << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << A_ << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "---------------------------------    lbA    ----------------------------------" << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << lbA_.transpose() << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "---------------------------------    ubA    ----------------------------------" << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << ubA_.transpose() << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
    }
    else
    {
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "                   s.t. lbA <= Ax <= ubA is not inserted.                     " << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
    }

    if (num_cons_ == 0)
    {
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "                             wrong problem type                               " << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
    }
}

void QuadraticProgram::Print_subject_to_x()
{
    if (bool_constraint_x_ == true)
    {
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "---------------------------------    lb    -----------------------------------" << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << lb_.transpose() << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "---------------------------------    ub    -----------------------------------" << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << ub_.transpose() << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
    }
    else
    {
        std::cout << "------------------------------------------------------------------------------" << '\n';
        std::cout << "                     s.t. lb <= x <= ub is not inserted.                      " << '\n';
        std::cout << "------------------------------------------------------------------------------" << '\n';
    }
}

void QuadraticProgram::enable_equality_condition(const double Tolerance)
{
    options_.enableEqualities = qpOASES::BT_TRUE;
    qpOASES::real_t Tolerance_equal = Tolerance;
    options_.boundRelaxation = Tolerance_equal;
}

void QuadraticProgram::disable_equality_condition()
{
    options_.enableEqualities = qpOASES::BT_FALSE;
}

void QuadraticProgram::set_hotstart_computation_time_limit(const qpOASES::real_t& compute_time) //sec, do not use it yet
{
    comp_time_ = compute_time;
}

void QuadraticProgram::enable_print_option_debug()
{
    options_.printLevel = qpOASES::PL_DEBUG_ITER;
}

void QuadraticProgram::disable_print_option_debug()
{
    options_.printLevel = qpOASES::PL_NONE;
}

void QuadraticProgram::solve_qpOASES(const int& num_max_iter)
{
    //translate eigen to real_t formulation
    // H in min eq 1/2x'Hx + x'g
    for (int i = 0; i < num_var_; i++)
    {
        for (int j = 0; j < num_var_; j++)
        {
            H_realt_[num_var_ * j + i] = H_(j, i);
        }
    }

    // g in min eq 1/2x'Hx + x'g
    for (int i = 0; i < num_var_; i++)
    {
        g_realt_[i] = g_(i);
    }

    // A in s.t. eq lbA<= Ax <=ubA
    if (bool_constraint_Ax_ == true)
    {
        for (int i = 0; i < num_var_; i++)
        {
            for (int j = 0; j < num_cons_; j++)
            {
                A_realt_[num_var_ * j + i] = A_(j, i);
            }
        }
    }

    // lbA in s.t. eq lbA<= Ax <=ubA
    if (bool_constraint_Ax_ == true)
    {
        for (int i = 0; i < num_cons_; i++)
        {
            lbA_realt_[i] = lbA_(i);
        }
    }

    // ubA in s.t. eq lbA<= Ax <=ubA
    if (bool_constraint_Ax_ == true)
    {
        for (int i = 0; i < num_cons_; i++)
        {
            ubA_realt_[i] = ubA_(i);
        }
    }

    //lb in s.t. eq lb <= x <= ub
    if (bool_constraint_x_ == true)
    {
        for (int i = 0; i < num_var_; i++)
        {
            lb_realt_[i] = lb_(i);
        }
    }

    //ub in s.t. eq lb <= x <= ub
    if (bool_constraint_x_ == true)
    {
        for (int i = 0; i < num_var_; i++)
        {
            ub_realt_[i] = ub_(i);
        }
    }

    SQP_prob_.setOptions(options_);

    qpOASES::int_t nWSR = num_max_iter;
    qpOASES::returnValue m_status;

    if (b_initialized_ == false)//init
    {
        if (bool_constraint_Ax_ == true && bool_constraint_x_ == true)
        {
            m_status = SQP_prob_.init(H_realt_, g_realt_, A_realt_, lb_realt_, ub_realt_, lbA_realt_, ubA_realt_, nWSR);            
        }
        else if (bool_constraint_Ax_ == true && bool_constraint_x_ == false)
        {
            m_status = SQP_prob_.init(H_realt_, g_realt_, A_realt_, nullptr, nullptr, lbA_realt_, ubA_realt_, nWSR);            
        }
        else if (bool_constraint_Ax_ == false && bool_constraint_x_ == true)
        {
            m_status = SQP_prob_.init(H_realt_, g_realt_, nullptr, lb_realt_, ub_realt_, nullptr, nullptr, nWSR);            
        }
        else
        {
            m_status = SQP_prob_.init(H_realt_, g_realt_, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);            
        }
        b_initialized_ = true;
    }
    else//hotstart
    {
        if (bool_constraint_Ax_ == true && bool_constraint_x_ == true)
        {
            m_status = SQP_prob_.hotstart(H_realt_, g_realt_, A_realt_, lb_realt_, ub_realt_, lbA_realt_, ubA_realt_, nWSR);//,&comp_time_);
        }
        else if (bool_constraint_Ax_ == true && bool_constraint_x_ == false)
        {
            m_status = SQP_prob_.hotstart(H_realt_, g_realt_, A_realt_, nullptr, nullptr, lbA_realt_, ubA_realt_, nWSR);//,&comp_time_);
        }
        else if (bool_constraint_Ax_ == false && bool_constraint_x_ == true)
        {
            m_status = SQP_prob_.hotstart(H_realt_, g_realt_, nullptr, lb_realt_, ub_realt_, nullptr, nullptr, nWSR);//,&comp_time_);
        }
        else
        {
            m_status = SQP_prob_.hotstart(H_realt_, g_realt_, nullptr, nullptr, nullptr, nullptr, nullptr, nWSR);//,&comp_time_);            
        }
    }

    SQP_prob_.getPrimalSolution(X_opt_realt_);    

    num_state_ = m_status;
    for (int i = 0; i < num_var_; i++)
    {
        X_opt_(i) = X_opt_realt_[i];
    }
    return;
}