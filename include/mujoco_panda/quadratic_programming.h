#pragma once
#ifndef __QUADRATICPROGRAM_H__
#define __QUADRATICPROGRAM_H__

#include <iostream>
#include <Eigen/Dense>
#include "qpOASES.hpp"
#include "control_math.h"

#define useDynamicMemoryAllocation

#ifdef useDynamicMemoryAllocation
	#define CustomQPArraySize Dynamic
#else
	#define CustomQPArraySize 80
#endif

typedef double  rScalar;
namespace Eigen
{
		typedef Matrix<rScalar, -1, -1, 0, CustomQPArraySize, CustomQPArraySize> MatrixQPd;
		typedef Matrix<rScalar, -1, -1, 0, CustomQPArraySize, 1> VectorQPd;
		// typedef MatrixXd MatrixQPd;
		// typedef VectorXd VectorQPd;
}

class QuadraticProgram
{
public:
	explicit QuadraticProgram();
	virtual ~QuadraticProgram();

public:
	void initialize_problem_size(const int& num_var, const int& num_cons);
	void update_min_problem(const Eigen::MatrixQPd& H, const Eigen::VectorQPd& g);
	void update_subject_to_Ax(const Eigen::MatrixQPd& A, const Eigen::VectorQPd& lbA, const Eigen::VectorQPd& ubA);
	void update_subject_to_X(const Eigen::VectorQPd& lb, const Eigen::VectorQPd& ub);
	void delete_subject_to_Ax();
	void delete_subject_to_X();
	void Print_min_prob();
	void Print_subject_to_Ax();
	void Print_subject_to_x();
	void enable_print_option_debug();
	void disable_print_option_debug();
	void enable_equality_condition(const double Tolerance);
	void disable_equality_condition();
	void set_hotstart_computation_time_limit(const qpOASES::real_t& compute_time); //it maybe unstable
	void solve_qpOASES(const int& num_max_iter);
	int num_state_;
	Eigen::VectorQPd X_opt_;
	bool b_initialized_;
	int num_var_;
	int num_cons_;

private:
	qpOASES::SQProblem SQP_prob_;
	qpOASES::Options options_;	
	Eigen::MatrixQPd H_;
	Eigen::VectorQPd g_;
	bool bool_constraint_Ax_;
	Eigen::MatrixQPd A_;
	Eigen::VectorQPd lbA_;
	Eigen::VectorQPd ubA_;
	bool bool_constraint_x_;
	Eigen::VectorQPd lb_;
	Eigen::VectorQPd ub_;
	qpOASES::real_t comp_time_;

	qpOASES::real_t H_realt_[10000] = { 0 };
	qpOASES::real_t g_realt_[100] = { 0 };
	qpOASES::real_t A_realt_[10000] = { 0 };
	qpOASES::real_t lbA_realt_[100] = { 0 };
	qpOASES::real_t ubA_realt_[100] = { 0 };
	qpOASES::real_t lb_realt_[100] = { 0 };
	qpOASES::real_t ub_realt_[100] = { 0 };
	qpOASES::real_t X_opt_realt_[100] = { 0 };

public:
	void initialize();
};

#endif // __QUADRATICPROGRAM_H__