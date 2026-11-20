#ifndef UDP_NODE_H
#define UDP_NODE_H

#include <array>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <netinet/in.h>
#include <sys/socket.h>

class UdpNode {
public:
    /**
     * @brief 构造函数
     * @param local_ip   本地IP地址
     * @param local_port 本地端口
     * @param remote_ip  远端IP地址
     * @param remote_port 远端端口
     */
    UdpNode(const std::string& local_ip, int local_port,
            const std::string& remote_ip, int remote_port);

    ~UdpNode();

    // 禁止拷贝和赋值
    UdpNode(const UdpNode&) = delete;
    UdpNode& operator=(const UdpNode&) = delete;

    // ==================== 生命周期管理 ====================
    bool start();
    void stop();
    bool isRunning() const;

    // ==================== 目标位置相关接口（可读可写） ====================
    // 单个关节
    bool setTargetPosition(int joint_index, float position);
    bool getTargetPosition(int joint_index, float& position) const;
    
    // 批量操作 - vector形式
    bool setAllTargetPositions(const std::vector<float>& positions);
    bool getAllTargetPositions(std::vector<float>& positions) const;
    
    // 批量操作 - array形式
    bool setTargetPositionsArray(const std::array<float, 14>& positions);
    bool getTargetPositionsArray(std::array<float, 14>& positions) const;

    // ==================== 当前位置相关接口（可读可写） ====================
    bool setCurrentPosition(int joint_index, float position);
    bool getCurrentPosition(int joint_index, float& position) const;
    
    bool setAllCurrentPositions(const std::vector<float>& positions);
    bool getAllCurrentPositions(std::vector<float>& positions) const;
    
    bool setCurrentPositionsArray(const std::array<float, 14>& positions);
    bool getCurrentPositionsArray(std::array<float, 14>& positions) const;

    // ==================== 当前速度相关接口（可读可写） ====================
    bool setCurrentVelocity(int joint_index, float velocity);
    bool getCurrentVelocity(int joint_index, float& velocity) const;
    
    bool setAllCurrentVelocities(const std::vector<float>& velocities);
    bool getAllCurrentVelocities(std::vector<float>& velocities) const;
    
    bool setCurrentVelocitiesArray(const std::array<float, 14>& velocities);
    bool getCurrentVelocitiesArray(std::array<float, 14>& velocities) const;

    // ==================== 当前力矩相关接口（可读可写） ====================
    bool setCurrentTorque(int joint_index, float torque);
    bool getCurrentTorque(int joint_index, float& torque) const;
    
    bool setAllCurrentTorques(const std::vector<float>& torques);
    bool getAllCurrentTorques(std::vector<float>& torques) const;
    
    bool setCurrentTorquesArray(const std::array<float, 14>& torques);
    bool getCurrentTorquesArray(std::array<float, 14>& torques) const;

    // ==================== 综合状态接口（可读可写） ====================
    // 设置单个关节的所有状态
    bool setJointState(int joint_index, float position, 
                      float velocity, float torque);
    
    // 获取单个关节的所有状态
    bool getJointState(int joint_index, float& position, 
                      float& velocity, float& torque) const;
    
    // 设置所有关节的所有状态
    bool setAllJointStates(const std::vector<float>& positions,
                          const std::vector<float>& velocities,
                          const std::vector<float>& torques);
    
    // 获取所有关节的所有状态
    bool getAllJointStates(std::vector<float>& positions,
                          std::vector<float>& velocities,
                          std::vector<float>& torques) const;

    // ==================== 数据发送接口 ====================
    // 发送目标位置到远端
    bool sendTargetPositions();
    
    // 发送当前状态（位置、速度、力矩）到远端
    bool sendCurrentStates();
    
    // 发送所有数据到远端（目标位置 + 当前状态）
    bool sendAllData();

    // ==================== 数据接收状态检查 ====================
    bool hasNewTargetData() const;
    bool consumeNewTargetData();  // 读取并清除标志（原子操作）
    bool hasNewStateData() const;
    unsigned long long getTargetDataUpdateCount() const;
    unsigned long long getStateDataUpdateCount() const;
    void resetDataUpdateCounts();

    // ==================== 连接状态检查 ====================
    bool isConnected() const;
    std::string getLocalIP() const;
    int getLocalPort() const;
    std::string getRemoteIP() const;
    int getRemotePort() const;

    // ==================== GUI 远程指令接口 ====================
    bool hasPendingKey() const;
    char consumePendingKey();
    bool hasPendingVelocity() const;
    void consumePendingVelocity(float& forward, float& side, float& turn);
    bool hasPendingEstop() const;
    bool consumePendingEstop();
    // Command ingestion is shared by the UDP receiver and offline checks.
    void queueKey(char key);
    void queueVelocity(float forward, float side, float turn);
    void queueEstop();

    // ==================== 数据统计接口 ====================
    unsigned long long getPacketsSent() const;
    unsigned long long getPacketsReceived() const;
    void resetStatistics();

private:
    void receiveLoop();  // 接收线程
    void updateStatisticsSent();
    void updateStatisticsReceived();

    int sockfd_tx = -1, sockfd_rx = -1;
    bool configured_ = false;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;
    std::atomic<bool> running_;
    std::thread recv_thread_;

    mutable std::mutex mutex_;
    
    // 可读可写的数据成员
    std::array<float, 14> target_positions_{};   // 目标位置
    std::array<float, 14> current_positions_{};  // 当前位置
    std::array<float, 14> current_velocities_{}; // 当前速度
    std::array<float, 14> current_torques_{};    // 当前力矩
    
    // 数据更新标志
    std::atomic<bool> new_target_data_;
    std::atomic<bool> new_state_data_;
    std::atomic<unsigned long long> target_data_update_count_;
    std::atomic<unsigned long long> state_data_update_count_;
    
    // 统计信息
    std::atomic<unsigned long long> packets_sent_;
    std::atomic<unsigned long long> packets_received_;

    // GUI 远程指令 (来自 PyQt 中控面板的 16 字节命令包)
    std::atomic<bool> has_pending_key_{false};
    std::atomic<char> pending_key_{'\0'};
    std::atomic<bool> has_pending_velocity_{false};
    float pending_velocity_[3] = {0.0f, 0.0f, 0.0f};
    std::atomic<bool> has_pending_estop_{false};

    // 连接信息
    std::string local_ip_;
    int local_port_;
    std::string remote_ip_;
    int remote_port_;
};

#endif // UDP_NODE_H
