#ifndef CR1PRO_HARDWARE_INTERFACE_HPP_
#define CR1PRO_HARDWARE_INTERFACE_HPP_

#include "ecan_hardware_interface.hpp"

class CR1ProHardwareInterface : public EcanHardwareInterface
{
protected:
    void OnHardStopRecoveredAllJoints() override {
        // 驱动器故障复位（清除hard_stop期间latch的fault状态）
        ResetJointError();
        // 重置偏置并等待关节数据刷新后做360度wrap
        InitPositionOffset(~0);
        ResetPositionOffset();
    }

    void ResetPositionOffset(){
        memset(data_updated_, 0, dof_num_*sizeof(bool));
        this->DoSetJointCommand(MatXf::Zero(dof_num_, 5));
        usleep(100*1000);
        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        while (!IsDataUpdatedFinished()){
            if (stop_recover_.load(std::memory_order_acquire)) {
                std::cout << "stop_recover_ requested, exit ResetPositionOffset busy-loop\n";
                return;
            }
            ++cnt;
            this->RefreshRobotData();
            current_joint_pos = this->GetJointPosition();
            // std::cout<<current_joint_pos.transpose()<<std::endl;
            for(int i=0;i<dof_num_;++i){
                if(!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i)){
                    data_updated_[i] = true;
                    std::cout << "joint " << i << " data updated at " << cnt << " cnt!" << std::endl;
                }
                
            }
            last_joint_pos = current_joint_pos;
            usleep(1000);
            if(cnt >= 10000){
                std::cout << "[WARN] joint data update timeout (~10s), exit ResetPositionOffset\n";
                break;
            }
        }
        if (!IsDataUpdatedFinished()) {
            std::cout << "[WARN] Skip wrap_and_update due to incomplete joint data update\n";
            return;
        }
        WrapPositionOffset();
    }

    // 读取当前关节位置，对多整圈偏差的关节做360°修正并更新offset
    void WrapPositionOffset() override {
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        VecXf current_joint_pos = this->GetJointPosition();

        auto wrap_and_update = [&](int idx, float low_deg, float high_deg) {
            bool changed = false;
            if (current_joint_pos(idx) > Deg2Rad(high_deg)) {
                pos_offset_[idx] -= 360.f;
                changed = true;
            } else if (current_joint_pos(idx) < Deg2Rad(low_deg)) {
                pos_offset_[idx] += 360.f;
                changed = true;
            }
            if (changed) {
                std::cout << "joint " << idx << " offset is changed\n";
            }
            joint_config_[idx].offset = Deg2Rad(pos_offset_[idx]);
            joint_pos_(idx) = joint_pos_raw_(idx) + joint_config_[idx].offset;
        };

        for (int idx : {7, 14}) { // wristz上报容易多360度
            wrap_and_update(idx, -180.f, 180.f);
        }
        for (int idx : {5, 12}) { // shouderz上报容易多360度
            wrap_and_update(idx, -180.f, 180.f);
        }

        wrap_and_update(0, -120.f, 220.f);
        wrap_and_update(25, -220.f, 40.f);
        wrap_and_update(19, -40.f, 220.f);

        // 肩pitch（矢状面，不镜像，左右同量程 -190~90）
        wrap_and_update(3,  -200.f, 100.f);
        wrap_and_update(10, -200.f, 100.f);
        // 肩x/roll（镜像，左 -25~205 / 右 -205~25）
        wrap_and_update(4,  -35.f, 215.f);
        wrap_and_update(11, -215.f, 35.f);
        // 髋pitch（矢状面，不镜像，左右同量程 -160~90）
        wrap_and_update(17, -170.f, 100.f);
        wrap_and_update(23, -170.f, 100.f);
    }

public:
    CR1ProHardwareInterface(const std::string& robot_name):EcanHardwareInterface(robot_name,31,12,14,3,2){
        InitPositionOffset(~0);
    }
    ~CR1ProHardwareInterface(){}

    virtual void Start(){
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop(){
    }
    virtual void InitPositionOffset(uint64_t jointMask){
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        float init_pos_offset[31] = {-100., 0., 0.,//腰 wristX Y限位太难标了，标在0，相对准确
                                    90., -0*25., 0*170., -44., 170., -90., -90.,//左手
                                    90., 0*25., -0*170., -44., -170., -90., 90.,//右手
                                    0*90., -0*25., -30., -10., 45., 35.,//左腿
                                    0*90., 0*25., 30., -10., 45., -35.,//右腿
                                    -0., 20.};//头
        float joint_dir[31] = { -1, 1, 1,//腰
                                -1, 1, 1, -1, 1, 1, 1,//左手
                                1, 1, 1, -1, 1, 1, 1,//右手
                                1, 1, 1, 1, 1, -1,//左腿
                                -1, -1, 1, 1, 1, -1,//右腿
                                -1, -1//头
                                };

        for(int i=0;i<dof_num_;++i){
            if(jointMask & (uint64_t{1} << i))
            {
                pos_offset_[i] = init_pos_offset[i];
                joint_dir_[i] = joint_dir[i];
                data_updated_[i] = false;
                joint_config_[i].dir = joint_dir_[i];
                joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
                // WrapPositionOffset and upper-body Idle immediately read this
                // cache; do not wait for the next motor feedback callback.
                joint_pos_(i) = joint_pos_raw_(i) + joint_config_[i].offset;
            }
            
        }
    }
};





#endif
