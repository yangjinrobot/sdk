/**
 * @file policy_runner_base.hpp
 * @brief policy runner base class
 * @author deeprobotics
 * @version 1.0
 * @date 2024-06-06
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef HUMANOID_POLICY_RUNNER_BASE_HPP_
#define HUMANOID_POLICY_RUNNER_BASE_HPP_

#include "common_types.h"
#include "custom_types.h"
#include "basic_function.hpp"
#include "onnxruntime_cxx_api.h"

using namespace types;

namespace humanoid {

class PolicyRunnerBase {
public:
    PolicyRunnerBase(std::string policy_name, RobotName robot_name) : policy_name_(policy_name), robot_name_(robot_name)  {
        vel_delta_const_ << 0.3, 0.2, 0.3;
        cmd_vel_input_.setZero();
    }

    virtual ~PolicyRunnerBase() {}

    /**
     * @brief use to display policy detail
     */
    virtual void DisplayPolicyInfo() {

    }

    /**
     * @brief Get the robot action object by run your policy
     * @return RobotAction 
     */
    virtual RobotAction GetRobotAction(const RobotBasicState &, const UserCommand &) = 0;

    /**
     * @brief execute function when first entering policy runner
     */
    virtual void OnEnter() = 0;

    /**
     * @brief execute function when exit policy runner
     */
    virtual void OnExit() = 0;

    /**
     * @brief check if the current motion has finished playing
     * @return true if motion reached its last frame
     */
    virtual bool IsMotionFinished() const { return false; }

    /**
     * @brief Set the decimation
     * @param  d decimation
     */
    virtual void SetDecimation(int d) {
        decimation_ = d;
    }

    /**
     * @brief get the first frame joint positions mapped to robot space
     * @param motion_idx external motion index
     * @return joint positions in robot DOF order, or empty vector if not supported
     */
    virtual VecXf GetMotionInitPosInRobotSpace(int motion_idx) { return VecXf(); }

    /**
     * @brief check if the current motion is paused at a stage breakpoint
     */
    virtual bool IsStagePaused() const { return false; }

    // 跳过手臂平滑过渡（mimic→loco 切换时由外部设置）
    virtual void SetSkipArmTransition(bool skip) {}

    /**
     * @brief resume playback from the current stage breakpoint
     */
    virtual void ResumeStage() {}

    /**
     * @brief set frame breakpoints for a specific motion (by internal array index)
     * @param motion_array_idx internal array index of the motion
     * @param breakpoints ordered list of frame numbers where playback should pause
     * @param finish_frame frame number where motion is considered finished (0 = use all frames)
     */
    virtual void SetMotionBreakpoints(int motion_array_idx, const std::vector<int>& breakpoints, int finish_frame = 0) {}

    /**
     * @brief set a ping-pong loop range for a specific motion (by internal array index)
     *        playback will loop back-and-forth between [start_frame, end_frame] until BreakLoop() is invoked
     */
    virtual void SetMotionLoopRange(int motion_array_idx, int start_frame, int end_frame) {}

    /**
     * @brief whether the active motion is currently inside a ping-pong loop
     */
    virtual bool IsLooping() const { return false; }

    /**
     * @brief request to break out of the current ping-pong loop and resume normal forward playback
     */
    virtual void BreakLoop() {}

    const std::string policy_name_;
    const RobotName robot_name_;

    int decimation_;
    int run_cnt_;
    int run_cnt_record_;
    Vec3f vel_delta_const_, cmd_vel_input_;
};

}  // namespace humanoid

#endif