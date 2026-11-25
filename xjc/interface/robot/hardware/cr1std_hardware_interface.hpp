#ifndef CR1STD_HARDWARE_INTERFACE_HPP_
#define CR1STD_HARDWARE_INTERFACE_HPP_

#include "ecan_hardware_interface.hpp"

class CR1StdHardwareInterface : public EcanHardwareInterface
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
        };

        for (int idx : {3, 7}) { // shoulderz上报容易多360度 (std index)
            wrap_and_update(idx, -180.f, 180.f);
        }

        // 腿部对应关节 (std index): left=11, right=17
        wrap_and_update(17, -220.f, 40.f);
        wrap_and_update(11, -40.f, 220.f);
        wrap_and_update(0, -120.f, 220.f);

        // 肩pitch（矢状面，不镜像，左右同量程 -190~90）
        wrap_and_update(1, -200.f, 100.f);
        wrap_and_update(5, -200.f, 100.f);
        // 肩x/roll（镜像，左 -25~205 / 右 -205~25）
        wrap_and_update(2, -35.f, 215.f);
        wrap_and_update(6, -215.f, 35.f);
        // 髋pitch（矢状面，不镜像，左右同量程 -160~90）
        wrap_and_update(9,  -170.f, 100.f);
        wrap_and_update(15, -170.f, 100.f);
    }

public:
    CR1StdHardwareInterface(const std::string& robot_name):EcanHardwareInterface(robot_name,21,12,8,1,0){
        InitPositionOffset(~0);
    }
    ~CR1StdHardwareInterface(){}

    virtual void Start(){
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop(){
    }
    virtual void InitPositionOffset(uint64_t jointMask){
        float init_pos_offset[21] = {-100.,
                                    90., -0*25., 0*170., -44.,//左手
                                    90., 0*25., -0*170., -44.,//右手
                                    0*90., -0*25., -30., -10., 45., 35.,//左腿
                                    0*90., 0*25., 30., -10., 45., -35.//右腿
                                    };
        float joint_dir[21] = { -1,
                                -1, 1, 1, -1,//左手
                                1, 1, 1, -1,//右手
                                1, 1, 1, 1, 1, -1,//左腿
                                -1, -1, 1, 1, 1, -1,//右腿
                                };

        for(int i=0;i<dof_num_;++i){
            if(jointMask & (1 << i))
            {
                pos_offset_[i] = init_pos_offset[i];
                joint_dir_[i] = joint_dir[i];
                data_updated_[i] = false;
                joint_config_[i].dir = joint_dir_[i];
                joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
            }
        }
    }
};





#endif
