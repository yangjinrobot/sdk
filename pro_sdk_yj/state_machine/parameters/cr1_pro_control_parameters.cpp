#include "control_parameters.h"

void ControlParameters::GenerateCR1ProParameters(){//B样机pro版 关节顺序：腰*3+左臂*7+右臂*7+左腿*6+右腿*6+头*2
    dof_num_ = 31;
    waist_dof_num_ = 3;
    arm_dof_num_ = 7;
    leg_dof_num_ = 6;
    neck_dof_num_ = 2;

    pre_height_ = 0.45;
    stand_height_ = 0.75;
    stand_duration_ = 8.;
    liedown_duration_ = 5.;
    
    arm_joint_lower_ = VecXf::Zero(arm_dof_num_);
    arm_joint_upper_ = VecXf::Zero(arm_dof_num_);
    arm_kp_ = VecXf::Zero(arm_dof_num_);
    arm_kd_ = VecXf::Zero(arm_dof_num_);
    arm_link_len_ = VecXf::Zero(6);
    waist_joint_lower_ = VecXf::Zero(waist_dof_num_);
    waist_joint_upper_ = VecXf::Zero(waist_dof_num_);
    waist_kp_ = VecXf::Zero(waist_dof_num_);
    waist_kd_ = VecXf::Zero(waist_dof_num_);
    joint_vel_limit_ = VecXf::Zero(dof_num_);
    torque_limit_ = VecXf::Zero(dof_num_);

    joint_vel_limit_ << 19.38, 19.38, 20.,
                        19.38, 19.38, 19.38, 19.38, 23.76, 23.76, 23.76,
                        19.38, 19.38, 19.38, 19.38, 23.76, 23.76, 23.76,
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        23.76, 23.76;

    torque_limit_ <<    107., 107., 413.,
                        107., 107., 107., 107., 31., 31., 31.,
                        107., 107., 107., 107., 31., 31., 31.,
                        413., 107., 107., 413., 107., 31., 
                        413., 107., 107., 413., 107., 31.,
                        31., 31.;
    
    waist_joint_lower_ << -1.7453, -0.7854, -0.5236;
    waist_joint_upper_ << 3.6652, 0.7854, 1.5708;
    waist_kp_ << 600., 2800., 2300.;
    waist_kd_ << 6., 15., 20.;

    arm_joint_lower_ << -3.3161, -0.43633, -2.9671, -0.767945, -2.9671, -1.5708, -1.5708;//注意左右shoulderx,shoulderz,wristz,wristx需镜像
    arm_joint_upper_ << 1.5708,  3.5799,   2.9671,  1.74533,   2.9671,  1.5708,  1.5708;

    arm_kp_ << 600., 600., 600., 600., 90., 90., 90.;
    arm_kd_ << 6., 6., 6., 6., 2., 2., 2.;
    arm_link_len_ << 0.10135, 0.1486 , 0.1164, 0.099, 0.052, 0.082;
    hand_link_len_ = 0.15;
    
    leg_joint_lower_ << -2.7925, -0.4363, -0.5236, -0.1745, -1.0472, -0.61087;//注意左右hipx,hipz,anklex需镜像
    leg_joint_upper_ << 1.5708,  2.618,   3.6652,  2.5307,  0.7854,  0.61087;

    leg_kp_ << 1800.0, 600.0, 600.0, 1800.0, 600.0, 90.0;
    leg_kd_ << 24.0, 6.0, 6.0, 24.0, 6.0, 2.0;
    leg_link_len_ << 0.0483, 0.1635 , 0.2565, 0.410, 0.04665, 0.0;//最后一段待测

    neck_joint_lower_ = VecXf::Zero(neck_dof_num_);
    neck_joint_upper_ = VecXf::Zero(neck_dof_num_);
    neck_kp_ = VecXf::Zero(neck_dof_num_);
    neck_kd_ = VecXf::Zero(neck_dof_num_);
    neck_joint_lower_ << -1.75, -0.35;
    neck_joint_upper_ << 3.67, 0.35;
    neck_kp_ << 0., 0.;
    neck_kd_ << 0., 0.;

}