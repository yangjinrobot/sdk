/**
 * @file motion_data_publish.hpp
 * @brief 
 * @author huangtaotao
 * @version 1.0
 * @date 2024-12-23
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef MOTION_DATA_PUBLISH_HPP_
#define MOTION_DATA_PUBLISH_HPP_

#include "common_types.h"
#include <atomic>
#include "motion_state_feedback.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "drdds/core/drdds_core.h"
#include "safe_controller/safe_controller.hpp"
#include "robot_hwi.hpp"
#include "robot_model.hpp"

class MotionDataPublish
{
private:
    std::thread publish_thread_;
    MotionStateFeedback *msfb_;
    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;
    std::atomic<bool> start_thread_flag_{false};
    int run_cnt_;
    std::unique_ptr<ChannelStdMsgInt32>  hes_status_pub_ ;
    uint32_t last_joints_timeout_ = 0;
    uint32_t last_joints_pos_error_ = 0;
    uint32_t last_joints_data_nan_error_ = 0;
    uint32_t last_joints_vel_error_ = 0;
    uint32_t last_joints_update_error_ = 0;
    uint32_t last_joints_over_heat_ = 0;
    uint32_t last_drivers_over_heat_ = 0;
    uint32_t last_driver_status_error_ = 0;
    bool last_motion_posture_error_ = false;
    bool last_body_posture_error_ = false;
    // bool last_hipy_pos_outlimit_ = 0;
    bool last_imu_update_timeout_flag_ = false;
    bool last_imu_data_nan_flag_ = false;
    bool last_lift_flag_ = false;
    DrLogger logger_motion_status;

   
protected:
    std::unique_ptr<ChannelMotionStatus>motion_status_pub;
    std::unique_ptr<ChannelMotionInfo>motion_info_pub;
    std::unique_ptr<ChannelStdMsgInt32>action_info_pub;
    std::unique_ptr<ChannelStdMsgInt32>pose_info_pub;

public:
    MotionDataPublish(/* args */):logger_motion_status("Motion_pub"){
        
    motion_info_pub = std::make_unique<ChannelMotionInfo>("/MOTION_INFO", 0);
    action_info_pub = std::make_unique<ChannelStdMsgInt32>("/ACTION_INFO", 0);
    pose_info_pub = std::make_unique<ChannelStdMsgInt32>("/POSE_INFO", 0);

    std::string model = GetRobotModel();
    std::map<std::uint8_t, XYZ> motorMap = RobotConfig().getMapForModel(model);
    hes_status_pub_ = std::make_unique<ChannelStdMsgInt32>("/HES_STATUS", 0);
    motion_status_pub = std::make_unique<ChannelMotionStatus>("/MOTION_STATUS", 0);

}
    ~MotionDataPublish(){ Stop();
    }

    void Stop(){
        start_thread_flag_.store(false, std::memory_order_release);
        if (publish_thread_.joinable()) publish_thread_.join();
    }

    std::string GetTime(void){
        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        //转为字符串
        std::stringstream ss;
        // 可以分别以不同的形式进行显示
        //	ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
        ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
        //	ss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");

        std::string str_time = ss.str();
        return str_time;
    }
    void SetMotionStateFeedbackSource(MotionStateFeedback* msfb){
        msfb_ = msfb;
    }

    void SetRobotInterfaceSource(std::shared_ptr<interface::RobotInterface> ri_ptr){
        ri_ptr_ = ri_ptr;
    }

    void SetUserCommandInfaceSource(std::shared_ptr<interface::UserCommandInterface> uc_ptr){
        uc_ptr_ = uc_ptr;
    }

    void SetSafeControllerSource(std::shared_ptr<SafeController> sc_ptr){
        sc_ptr_ = sc_ptr;
    }

    void Start(){
        start_thread_flag_ = true;
        publish_thread_ = std::thread(std::bind(&MotionDataPublish::Run, this));
    }

    char* WrapperMotionData(){
        char* buffer = new char[4];
        uint32_t state = msfb_->GetCurrentState();
        
        memcpy(buffer, &state, sizeof(uint32_t));
        return buffer;
    }

    void PubMotionData(){
        motion_info_pub->data_->data().gait_state().gait() = msfb_->GetCurrentGait();
        motion_info_pub->data_->data().motion_state().state() = msfb_->GetCurrentState();
        motion_info_pub->Write();
        action_info_pub->data_->value() = msfb_->GetCurrentAction();
        action_info_pub->Write();
        pose_info_pub->data_->value() = msfb_->GetCurrentPose();
        pose_info_pub->Write();
    }
   
    void Run(){
        int tfd;    //定时器描述符
        int efd;    //epoll描述符
        int fds;
        uint64_t value;
        struct epoll_event ev, *evptr;
        struct itimerspec time_intv; //用来存储时间

        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if(tfd == -1)  return ;
        
        // it_value表示定时器第一次到期的时间
        time_intv.it_value.tv_sec = 1.; 
        time_intv.it_value.tv_nsec = 0;
        // it_interval定义了定时器的周期(5Hz, 0.2s触发一次)
        time_intv.it_interval.tv_sec = 0;   
        time_intv.it_interval.tv_nsec = 200 * 1000 * 1000; // 200ms执行一次

        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
        if (efd == -1)  close(tfd);
        
        evptr = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
        if (evptr == NULL) {
            close(tfd);
            close(efd);
        }

        ev.data.fd = tfd; 
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
        while(start_thread_flag_){
            fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
            if(evptr[0].events & EPOLLIN){   
                int ret = read(evptr->data.fd, &value, sizeof(uint64_t));
                if (ret == -1) {
                    continue;
                }
                else{
                }               
            }
            run_cnt_++;
            PubMotionData();
        }
    }
};

#endif
