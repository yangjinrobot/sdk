/**
 * @file data_upload_interface.hpp
 * @brief for robot data upload
 * @author deeprobotics
 * @date 2025-01-09
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

#ifndef DATA_UPLOAD_INTERFACE_HPP_
#define DATA_UPLOAD_INTERFACE_HPP_
#include "common_types.h"
#include "motion_state_feedback.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
// #include "drdds/core/drdds_core.h"

class DataUploadInterface
{
private:
    std::thread upload_thread_;
    MotionStateFeedback *msfb_;
    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;
    bool start_thread_flag_ = false;
    int run_cnt_;

    int sockfd_;
    sockaddr_in remote_addr_;
    const int remote_port_ = 43897;
    const std::string remote_ip_ = "127.0.0.1";

public:
    DataUploadInterface(RobotName robot_name){
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(remote_port_);
        remote_addr_.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

        // 设置本地地址和端口号
        struct sockaddr_in local_addr;
        std::memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 绑定到所有网络接口
        local_addr.sin_port = htons(10101); // 设置本地端口号

        // 绑定 socket 到本地地址和端口号
        if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
            std::cerr << "Failed to bind socket" << std::endl;
        }
        if (sockfd_ < 0) {
            perror("robot sender socket creation failed");
        }
    }
    ~DataUploadInterface(){}

    void SetMotionStateFeedbackSource(MotionStateFeedback* msfb){
        msfb_ = msfb;
    }

    void SetRobotInterfaceSource(std::shared_ptr<interface::RobotInterface> ri_ptr){
        ri_ptr_ = ri_ptr;
    }

    void SetUserCommandInfaceSource(std::shared_ptr<interface::UserCommandInterface> uc_ptr){
        uc_ptr_ = uc_ptr;
    }

    void Start(){
        start_thread_flag_ = true;
        upload_thread_ = std::thread(std::bind(&DataUploadInterface::Run, this));
    }

    char* WrapperMotionData(){
        char* buffer = new char[4];
        uint32_t state = msfb_->GetCurrentState();
        
        memcpy(buffer, &state, sizeof(uint32_t));
        return buffer;
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
        
        time_intv.it_value.tv_sec = 1.; 
        time_intv.it_value.tv_nsec = 1000*1000;
        time_intv.it_interval.tv_sec = 0;   
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

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
            // uint32_t state = msfb_->GetCurrentState();
            // std::cout << "state: " << state << std::endl;
            int nbytes = sendto(sockfd_, WrapperMotionData(), 4, 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if(nbytes < 0){
                std::cerr << "Error: Could not send message" << std::endl;
            }

        }
    }

};

#endif