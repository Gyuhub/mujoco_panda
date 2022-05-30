#include "mujoco_panda/model.h"

Model::Model()
: dofs_(0)
{
}

Model::~Model()
{
}

int Model::get_model()
{
    std::string home_path = std::string(std::getenv("HOME"));
    std::string model_file_path = home_path + "/rp_ws/src/mujoco_panda/models/franka_panda.urdf";
    bool bool_get_model = RigidBodyDynamics::Addons::URDFReadFromFile(model_file_path.c_str(), &rbdl_model_, false, true);
    if (bool_get_model)
    {
        dofs_ = rbdl_model_.dof_count;
        std::cout << "Successfully get the RBDL model! the DoFs of the model is " << dofs_ << '\n';
    }
    else
    {
        std::cout << "Failed to get the model... Please check the path of model file\n";
        return -1;
    }
    return 0;
}

int Model::update_kinematics(Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    q_ = q;
    qdot_ = qdot;

    if (dofs_ != 0)
    {
        RigidBodyDynamics::UpdateKinematicsCustom(rbdl_model_, &q_, &qdot_, NULL);
        bool_update_kinemtaics_ = true;
    }
    else
    {
        bool_update_kinemtaics_ = false;
        std::cout << "Failed to update the kinemacies! Please check the correct model!\n";
        return -1;
    }
    return 0;
}

int Model::update_dynamics()
{
    if (bool_update_kinemtaics_ != false)
    {
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(rbdl_model_, q_, A_, false);
        RigidBodyDynamics::InverseDynamics(rbdl_model_, q_, zero_, zero_, g_, NULL);
        RigidBodyDynamics::InverseDynamics(rbdl_model_, q_, qdot_, zero_, bg_, NULL);
        b_ = bg_ - g_;
        bool_update_dynamics_ = true;
    }
    else
    {
        bool_update_dynamics_ = false;
        std::cout << "Failed to update the dynamics! Please check the update of the kinematics first!\n";
        return -1;
    }
    return 0;
}

int Model::get_Jacobian()
{
    if (bool_update_kinemtaics_ != false)
    {
        J_.setZero();
        Eigen::MatrixXd J;
        J.setZero(6, dofs_);
        RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_, ee_id_, body_point_local_ee_, J, false);
        J_.block<3, DOFS>(0, 0) = J.block<3, DOFS>(3, 0);
        J_.block<3, DOFS>(3, 0) = J.block<3, DOFS>(0, 0);
        bool_get_jacobian_ = true;
    }
    else
    {
        bool_get_jacobian_ = false;
        std::cout << "Failed to get the Jacobian! Please check the update of kinematics first!\n";
        return -1;
    }
    return 0;
}

int Model::get_state()
{
    if (bool_update_dynamics_ != false)
    {
        pos_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl_model_, q_, ee_id_, body_point_local_ee_, false);
        R_ = RigidBodyDynamics::CalcBodyWorldOrientation(rbdl_model_, q_, ee_id_, false).transpose();
        ori_ = R_.eulerAngles(0, 1, 2); // XYZ euler angle rotation

        Eigen::VectorXd xdot(6);
        xdot = J_ * qdot_;
        posdot_ = xdot.head(3);
        oridot_ = xdot.tail(3);
        bool_get_state_ = true;
    }
    else
    {
        bool_get_state_ = false;
        std::cout << "Failed to get the state! Please check the update of dynamics first!\n";
        return -1;
    }
    return 0;    
}

double Model::get_dofs()
{
    return dofs_;
}

Eigen::Vector3d Model::get_desired_position_from_joint_angle(Eigen::VectorXd q)
{
    Eigen::Vector3d pos_des;
    pos_des.setZero();
    pos_des = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl_model_, q, ee_id_, body_point_local_ee_, false);
    return pos_des;
}

Eigen::Vector3d Model::get_desired_orientation_from_joint_angle(Eigen::VectorXd q)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d ori_des;
    R.setZero();
    ori_des.setZero();
    R = RigidBodyDynamics::CalcBodyWorldOrientation(rbdl_model_, q, ee_id_, false).transpose();
    ori_des = R.eulerAngles(0, 1, 2);
    return ori_des;
}

Eigen::MatrixXd Model::get_desired_Jacobian_from_joint_angle(Eigen::VectorXd q)
{
    Eigen::MatrixXd J_des;
    J_des.setZero(6, dofs_);
    RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q, ee_id_, body_point_local_ee_, J_des_, false);
    J_des.block<3, DOFS>(0, 0) = J_des_.block<3, DOFS>(3, 0);
    J_des.block<3, DOFS>(3, 0) = J_des_.block<3, DOFS>(0, 0);
    return J_des;
}

void Model::configurate_body()
{
    body_point_local_ee_(0) = 0.0;
    body_point_local_ee_(1) = 0.0;
    body_point_local_ee_(2) = 0.2; //0.107; // 0.0584;
}

void Model::initialize()
{
    ee_id_ = 7;
    dofs_ = DOFS;
    bool_update_kinemtaics_ = false;
    bool_update_dynamics_ = false;
    bool_get_state_ = false;
    bool_get_jacobian_ = false;

    // get_model();
    q_.setZero(dofs_);
    qdot_.setZero(dofs_);

    pos_.setZero();
    posdot_.setZero();
    ori_.setZero();
    oridot_.setZero();

    bg_.setZero(dofs_);
    b_.setZero(dofs_);
    g_.setZero(dofs_);

    body_point_local_ee_.setZero(3);

    zero_.setZero(dofs_);

    A_.setZero(dofs_, dofs_);
    J_.setZero(6, dofs_);
    J_des_.setZero(6, dofs_);

    R_.setZero();

    max_joint_position_.setZero(dofs_);
    min_joint_position_.setZero(dofs_);
    max_joint_torque_.setZero(dofs_);
    min_joint_torque_.setZero(dofs_);

    max_joint_position_(0) = 2.8973;
    max_joint_position_(1) = 1.7628;
    max_joint_position_(2) = 2.8973;
    max_joint_position_(3) = -0.0698;
    max_joint_position_(4) = 2.8973;
    max_joint_position_(5) = 3.7525;
    max_joint_position_(6) = 2.8973;
    max_joint_position_(7) = 0.04;
    max_joint_position_(8) = 0.04;

    min_joint_position_ = -max_joint_position_;
    min_joint_position_(3) = -3.0718;
    min_joint_position_(5) = -0.0175;
    min_joint_position_(7) = 0.0;
    min_joint_position_(8) = 0.0;

    max_joint_torque_.setConstant(15.0);
    min_joint_torque_ = -max_joint_torque_;
    configurate_body();
}