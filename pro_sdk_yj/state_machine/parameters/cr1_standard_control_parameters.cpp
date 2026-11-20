#include "control_parameters.h"

void ControlParameters::GenerateCR1StandardParameters(){//B样机标准版 关节顺序：腰*1+左臂*4+右臂*4+左腿*6+右腿*6+头*0
    dof_num_ = 21;
    waist_dof_num_ = 1;
    arm_dof_num_ = 4;
    leg_dof_num_ = 6;
    neck_dof_num_ = 0;

    pre_height_ = 0.55;
    stand_height_ = 0.75;
    stand_duration_ = 10.;
    liedown_duration_ = 7.;

    arm_joint_lower_ = VecXf::Zero(arm_dof_num_);
    arm_joint_upper_ = VecXf::Zero(arm_dof_num_);
    arm_kp_ = VecXf::Zero(arm_dof_num_);
    arm_kd_ = VecXf::Zero(arm_dof_num_);
    arm_link_len_ = VecXf::Zero(arm_dof_num_);
    waist_joint_lower_ = VecXf::Zero(waist_dof_num_);
    waist_joint_upper_ = VecXf::Zero(waist_dof_num_);
    waist_kp_ = VecXf::Zero(waist_dof_num_);
    waist_kd_ = VecXf::Zero(waist_dof_num_);
    joint_vel_limit_ = VecXf::Zero(dof_num_);
    torque_limit_ = VecXf::Zero(dof_num_);

    joint_vel_limit_ << 19.38,
                        19.38, 19.38, 19.38, 19.38,
                        19.38, 19.38, 19.38, 19.38,
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        20., 19.38, 19.38, 20., 19.38, 23.76;

    torque_limit_ <<    107.,
                        107., 107., 107., 107.,
                        107., 107., 107., 107.,
                        413., 107., 107., 413., 107., 31., 
                        413., 107., 107., 413., 107., 31.;
    
    waist_joint_lower_ << -3.6652;
    waist_joint_upper_ << 1.7453;
    waist_kp_ << 600.;
    waist_kd_ << 6.;

    arm_joint_lower_ << -3.3161, -0.43633, -2.9671, -0.767945;//注意左右shoulderx,shoulderz,wristz,wristx需镜像
    arm_joint_upper_ << 1.5708,  3.5799,   2.9671,  1.74533;

    arm_kp_ << 200., 200., 200., 200.;
    arm_kd_ << 5., 5., 5., 5.;
    arm_link_len_ << 0.10135, 0.1486 , 0.1164, 0.34;
    hand_link_len_ = 0.15;
    
    leg_joint_lower_ << -2.7925, -0.4363, -0.5236, -0.1745, -1.0472, -0.61087;//注意左右hipx,hipz,anklex需镜像
    leg_joint_upper_ << 1.5708,  2.618,   3.6652,  2.5307,  0.7854,  0.61087;

    leg_kp_ << 1800.0, 1800.0, 600.0, 1800.0, 1600.0, 150.0;
    leg_kd_ << 24.0, 6.0, 6.0, 24.0, 6.0, 3.0;
    leg_link_len_ << 0.0483, 0.1635 , 0.2565, 0.410, 0.04665, 0.0;//最后一段待测

}