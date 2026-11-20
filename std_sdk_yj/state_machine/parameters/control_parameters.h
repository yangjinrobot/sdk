/**
 * @file control_parameters.h
 * @brief basic control parameters
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef CONTROL_PARAMETERS_HPP_
#define CONTROL_PARAMETERS_HPP_

#include "common_types.h"
#include "custom_types.h"

using namespace types;

class ControlParameters
{
private:
    void GenerateCR1StandardParameters();
    void GenerateCR1ProParameters();

public:
    ControlParameters(RobotName robot_name){
        if(robot_name==RobotName::CR1STANDARD) GenerateCR1StandardParameters();
        else if(robot_name==RobotName::CR1PRO) GenerateCR1ProParameters();
        else{
            std::cerr << "Not Deafult Robot" << std::endl;
        }
    }
    ~ControlParameters(){}

    /**
     * @brief robot dof numbers
     */
    int dof_num_;

    /**
     * @brief waist dof numbers
     */
    int waist_dof_num_;

    /**
     * @brief arm dof numbers
     */
    int arm_dof_num_;

    /**
     * @brief leg dof numbers
     */
    int leg_dof_num_;

    /**
     * @brief neck dof numbers
     */
    int neck_dof_num_;

    /**
     * @brief robot link length
     */
    float body_len_x_, body_len_y_;
    float hip_len_, thigh_len_, shank_len_;

    /**
     * @brief stand height configure
     */
    float pre_height_, stand_height_; 

    /**
     * @brief one leg joint PD gain
     */
    Vec3f swing_leg_kp_, swing_leg_kd_;

    /**
     * @brief joint position limitation
     */
    Vec3f fl_joint_lower_, fl_joint_upper_;

    /**
     * @brief joint velocity limitation
     */
    VecXf joint_vel_limit_;

    /**
     * @brief joint torque limitation
     */
    VecXf torque_limit_;
    
    /**
     * @brief stand up duration
     */
    float stand_duration_ = 1.5;

    /**
     * @brief lie down duration
     */
    float liedown_duration_ = 2.0;

    /**
     * @brief wheel vel limit for wheel-legged robot
     */
    float wheel_vel_limit_ = 100;
    
    /**
     * @brief wheel link length to shank
     */
    float wheel_link_len_ = 0.040575;

    /**
     * @brief arm joint position limitation
     */
    VecXf arm_joint_lower_, arm_joint_upper_;

    /**
     * @brief arm joint kp
     */
    VecXf arm_kp_;

    /**
     * @brief arm joint kd
     */
    VecXf arm_kd_;

    /**
     * @brief arm link length
     */
    VecXf arm_link_len_;

    /**
     * @brief hand link length
     */
    float hand_link_len_;

    /**
     * @brief human leg joint position limitation
     */
    Vec6f leg_joint_lower_, leg_joint_upper_;

    /**
     * @brief human leg joint kp
     */
    Vec6f leg_kp_;

    /**
     * @brief human leg joint kd
     */
    Vec6f leg_kd_;

    /**
     * @brief human leg link length
     */
    Vec6f leg_link_len_;

    /**
     * @brief human waist joint position limitation
     */
    VecXf waist_joint_lower_, waist_joint_upper_;

    /**
     * @brief human waist joint kp
     */
    VecXf waist_kp_;

    /**
     * @brief human waist joint kd
     */
    VecXf waist_kd_;

    /**
     * @brief human neck joint position limitation
     */
    VecXf neck_joint_lower_, neck_joint_upper_;

    /**
     * @brief human neck joint kp
     */
    VecXf neck_kp_;

    /**
     * @brief human neck joint kd
     */
    VecXf neck_kd_;
    /**
     * @brief policy path
     */
    std::string common_policy_path_;
    Vec3f common_policy_p_gain_, common_policy_d_gain_;
    VecXf humanleg_policy_p_gain_, humanleg_policy_d_gain_;

    // std::string speed_policy_path_;
    // Vec3f speed_policy_p_gain_, speed_policy_d_gain_;

    // std::string tumbler_policy_path_;
    // Vec3f tumbler_policy_p_gain_, tumbler_policy_d_gain_;
};



#endif