#include "udp_node.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

static constexpr int JOINT_COUNT = 14;

// 目标位置数据包
struct TargetPacket {
    float target_positions[JOINT_COUNT];
};

// 状态数据包（位置、速度、力矩）
struct StatePacket {
    float current_positions[JOINT_COUNT];
    float current_velocities[JOINT_COUNT];
    float current_torques[JOINT_COUNT];
};

// 完整数据包（包含所有数据）
struct FullPacket {
    float target_positions[JOINT_COUNT];
    float current_positions[JOINT_COUNT];
    float current_velocities[JOINT_COUNT];
    float current_torques[JOINT_COUNT];
};

// GUI 远程指令包 (16 bytes, 来自 PyQt 中控面板)
// type=1: KEY_PRESS   [uint32 type][char key][3B pad]
// type=2: VELOCITY_CMD [uint32 type][float fwd][float side][float turn]
// type=3: ESTOP        [uint32 type][12B pad]
struct CommandPacket {
    uint32_t type;
    union {
        struct { char key; char pad[3]; } key_press;
        struct { float forward; float side; float turn; } velocity;
    } data;
};

UdpNode::UdpNode(const std::string& local_ip, int local_port,
                 const std::string& remote_ip, int remote_port)
    : running_(false), new_target_data_(false), new_state_data_(false),
      target_data_update_count_(0), state_data_update_count_(0),
      packets_sent_(0), packets_received_(0),
      local_ip_(local_ip), local_port_(local_port),
      remote_ip_(remote_ip), remote_port_(remote_port) {
    const auto fail = [&](const char* error) {
        std::cerr << "UdpNode: " << error << std::endl;
        if (sockfd_tx >= 0) { close(sockfd_tx); sockfd_tx = -1; }
        if (sockfd_rx >= 0) { close(sockfd_rx); sockfd_rx = -1; }
    };
    if (local_port < 0 || local_port > 65535 || remote_port <= 0 || remote_port > 65535) {
        fail("invalid UDP port");
        return;
    }
    local_addr_ = {};
    remote_addr_ = {};
    local_addr_.sin_family = remote_addr_.sin_family = AF_INET;
    local_addr_.sin_port = htons(local_port);
    remote_addr_.sin_port = htons(remote_port);
    if (inet_pton(AF_INET, local_ip.c_str(), &local_addr_.sin_addr) != 1 ||
        inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr) != 1) {
        fail("invalid local or remote IP address");
        return;
    }
    sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0);
    sockfd_rx = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_tx < 0 || sockfd_rx < 0) {
        fail("failed to create UDP sockets");
        return;
    }
    if (bind(sockfd_rx, reinterpret_cast<sockaddr*>(&local_addr_), sizeof(local_addr_)) != 0) {
        fail("failed to bind local UDP endpoint");
        return;
    }
    // A bounded receive timeout lets stop() join without closing a descriptor
    // while the receive thread is using it. Both descriptors close once below.
    timeval timeout{};
    timeout.tv_usec = 100000;
    if (setsockopt(sockfd_rx, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        fail("failed to set UDP receive timeout");
        return;
    }
    configured_ = true;
    std::cout << "UDP: " << local_ip << ':' << local_port
              << " <-> " << remote_ip << ':' << remote_port << std::endl;
}

UdpNode::~UdpNode() {
    stop();
    if (sockfd_tx >= 0) { close(sockfd_tx); sockfd_tx = -1; }
    if (sockfd_rx >= 0) { close(sockfd_rx); sockfd_rx = -1; }
}

bool UdpNode::start() {
    if (!configured_ || sockfd_tx < 0 || sockfd_rx < 0) return false;
    if (running_.load(std::memory_order_acquire)) return true;
    if (recv_thread_.joinable()) recv_thread_.join();
    running_.store(true, std::memory_order_release);
    try {
        recv_thread_ = std::thread([this] {
            try {
                receiveLoop();
            } catch (const std::exception& error) {
                std::cerr << "UDP receive failure: " << error.what() << std::endl;
            } catch (...) {
                std::cerr << "UDP receive failure" << std::endl;
            }
            running_.store(false, std::memory_order_release);
        });
    } catch (const std::exception& error) {
        running_.store(false, std::memory_order_release);
        std::cerr << "UDP thread start failed: " << error.what() << std::endl;
        return false;
    }
    return true;
}

void UdpNode::stop() {
    running_.store(false, std::memory_order_release);
    if (recv_thread_.joinable()) recv_thread_.join();
}

bool UdpNode::isRunning() const {
    return running_;
}

// ==================== 目标位置相关接口实现 ====================
bool UdpNode::setTargetPosition(int joint_index, float position) {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    target_positions_[joint_index] = position;
    return true;
}

bool UdpNode::getTargetPosition(int joint_index, float& position) const {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    position = target_positions_[joint_index];
    return true;
}

bool UdpNode::setAllTargetPositions(const std::vector<float>& positions) {
    if (positions.size() != JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        target_positions_[i] = positions[i];
    }
    return true;
}

bool UdpNode::getAllTargetPositions(std::vector<float>& positions) const {
    positions.resize(JOINT_COUNT);
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(target_positions_.begin(), target_positions_.end(), positions.begin());
    return true;
}

bool UdpNode::setTargetPositionsArray(const std::array<float, 14>& positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_positions_ = positions;
    return true;
}

bool UdpNode::getTargetPositionsArray(std::array<float, 14>& positions) const {
    std::lock_guard<std::mutex> lock(mutex_);
    positions = target_positions_;
    return true;
}

// ==================== 当前位置相关接口实现 ====================
bool UdpNode::setCurrentPosition(int joint_index, float position) {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_positions_[joint_index] = position;
    return true;
}

bool UdpNode::getCurrentPosition(int joint_index, float& position) const {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    position = current_positions_[joint_index];
    return true;
}

bool UdpNode::setAllCurrentPositions(const std::vector<float>& positions) {
    if (positions.size() != JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        current_positions_[i] = positions[i];
    }
    return true;
}

bool UdpNode::getAllCurrentPositions(std::vector<float>& positions) const {
    positions.resize(JOINT_COUNT);
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(current_positions_.begin(), current_positions_.end(), positions.begin());
    return true;
}

bool UdpNode::setCurrentPositionsArray(const std::array<float, 14>& positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_positions_ = positions;
    return true;
}

bool UdpNode::getCurrentPositionsArray(std::array<float, 14>& positions) const {
    std::lock_guard<std::mutex> lock(mutex_);
    positions = current_positions_;
    return true;
}

// ==================== 当前速度相关接口实现 ====================
bool UdpNode::setCurrentVelocity(int joint_index, float velocity) {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_velocities_[joint_index] = velocity;
    return true;
}

bool UdpNode::getCurrentVelocity(int joint_index, float& velocity) const {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    velocity = current_velocities_[joint_index];
    return true;
}

bool UdpNode::setAllCurrentVelocities(const std::vector<float>& velocities) {
    if (velocities.size() != JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        current_velocities_[i] = velocities[i];
    }
    return true;
}

bool UdpNode::getAllCurrentVelocities(std::vector<float>& velocities) const {
    velocities.resize(JOINT_COUNT);
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(current_velocities_.begin(), current_velocities_.end(), velocities.begin());
    return true;
}

bool UdpNode::setCurrentVelocitiesArray(const std::array<float, 14>& velocities) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_velocities_ = velocities;
    return true;
}

bool UdpNode::getCurrentVelocitiesArray(std::array<float, 14>& velocities) const {
    std::lock_guard<std::mutex> lock(mutex_);
    velocities = current_velocities_;
    return true;
}

// ==================== 当前力矩相关接口实现 ====================
bool UdpNode::setCurrentTorque(int joint_index, float torque) {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_torques_[joint_index] = torque;
    return true;
}

bool UdpNode::getCurrentTorque(int joint_index, float& torque) const {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    torque = current_torques_[joint_index];
    return true;
}

bool UdpNode::setAllCurrentTorques(const std::vector<float>& torques) {
    if (torques.size() != JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        current_torques_[i] = torques[i];
    }
    return true;
}

bool UdpNode::getAllCurrentTorques(std::vector<float>& torques) const {
    torques.resize(JOINT_COUNT);
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(current_torques_.begin(), current_torques_.end(), torques.begin());
    return true;
}

bool UdpNode::setCurrentTorquesArray(const std::array<float, 14>& torques) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_torques_ = torques;
    return true;
}

bool UdpNode::getCurrentTorquesArray(std::array<float, 14>& torques) const {
    std::lock_guard<std::mutex> lock(mutex_);
    torques = current_torques_;
    return true;
}

// ==================== 综合状态接口实现 ====================
bool UdpNode::setJointState(int joint_index, float position, 
                           float velocity, float torque) {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    current_positions_[joint_index] = position;
    current_velocities_[joint_index] = velocity;
    current_torques_[joint_index] = torque;
    return true;
}

bool UdpNode::getJointState(int joint_index, float& position, 
                           float& velocity, float& torque) const {
    if (joint_index < 0 || joint_index >= JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    position = current_positions_[joint_index];
    velocity = current_velocities_[joint_index];
    torque = current_torques_[joint_index];
    return true;
}

bool UdpNode::setAllJointStates(const std::vector<float>& positions,
                               const std::vector<float>& velocities,
                               const std::vector<float>& torques) {
    if (positions.size() != JOINT_COUNT || 
        velocities.size() != JOINT_COUNT || 
        torques.size() != JOINT_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        current_positions_[i] = positions[i];
        current_velocities_[i] = velocities[i];
        current_torques_[i] = torques[i];
    }
    return true;
}

bool UdpNode::getAllJointStates(std::vector<float>& positions,
                               std::vector<float>& velocities,
                               std::vector<float>& torques) const {
    positions.resize(JOINT_COUNT);
    velocities.resize(JOINT_COUNT);
    torques.resize(JOINT_COUNT);
    std::lock_guard<std::mutex> lock(mutex_);
    std::copy(current_positions_.begin(), current_positions_.end(), positions.begin());
    std::copy(current_velocities_.begin(), current_velocities_.end(), velocities.begin());
    std::copy(current_torques_.begin(), current_torques_.end(), torques.begin());
    return true;
}

// ==================== 数据发送接口实现 ====================
bool UdpNode::sendTargetPositions() {
    if (sockfd_tx < 0) return false;
    
    TargetPacket pkt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::copy(target_positions_.begin(), target_positions_.end(), pkt.target_positions);
    }
    
    ssize_t sent = sendto(sockfd_tx, &pkt, sizeof(pkt), 0,
                          (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (sent != sizeof(pkt)) {
        std::cerr << "Failed to send target packet, sent " << sent << " bytes" << std::endl;
        return false;
    }
    updateStatisticsSent();
    return true;
}

bool UdpNode::sendCurrentStates() {
    if (sockfd_tx < 0) return false;
    
    StatePacket pkt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::copy(current_positions_.begin(), current_positions_.end(), pkt.current_positions);
        std::copy(current_velocities_.begin(), current_velocities_.end(), pkt.current_velocities);
        std::copy(current_torques_.begin(), current_torques_.end(), pkt.current_torques);
    }
    
    ssize_t sent = sendto(sockfd_tx, &pkt, sizeof(pkt), 0,
                          (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (sent != sizeof(pkt)) {
        std::cerr << "Failed to send state packet, sent " << sent << " bytes" << std::endl;
        return false;
    }
    updateStatisticsSent();
    return true;
}

bool UdpNode::sendAllData() {
    if (sockfd_tx < 0) return false;
    
    FullPacket pkt;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::copy(target_positions_.begin(), target_positions_.end(), pkt.target_positions);
        std::copy(current_positions_.begin(), current_positions_.end(), pkt.current_positions);
        std::copy(current_velocities_.begin(), current_velocities_.end(), pkt.current_velocities);
        std::copy(current_torques_.begin(), current_torques_.end(), pkt.current_torques);
    }
    
    ssize_t sent = sendto(sockfd_tx, &pkt, sizeof(pkt), 0,
                          (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    if (sent != sizeof(pkt)) {
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET,&(remote_addr_.sin_addr),ip,INET_ADDRSTRLEN);
        int port=ntohs(remote_addr_.sin_port);
        std::cout << "UDP addr is "<< ip <<" port "<<port<<std::endl;
        std::cerr << "Failed to send full packet, sent " << sent << " bytes" << std::endl;
        return false;
    }
    updateStatisticsSent();
    return true;
}

// ==================== 数据接收状态检查 ====================
bool UdpNode::hasNewTargetData() const {
    return new_target_data_.load();
}

bool UdpNode::consumeNewTargetData() {
    return new_target_data_.exchange(false);
}

bool UdpNode::hasNewStateData() const {
    return new_state_data_.load();
}

unsigned long long UdpNode::getTargetDataUpdateCount() const {
    return target_data_update_count_.load();
}

unsigned long long UdpNode::getStateDataUpdateCount() const {
    return state_data_update_count_.load();
}

void UdpNode::resetDataUpdateCounts() {
    target_data_update_count_ = 0;
    state_data_update_count_ = 0;
    new_target_data_ = false;
    new_state_data_ = false;
}

// ==================== 连接状态检查 ====================
bool UdpNode::isConnected() const {
    return sockfd_tx >= 0 && running_;
}

std::string UdpNode::getLocalIP() const {
    return local_ip_;
}

int UdpNode::getLocalPort() const {
    return local_port_;
}

std::string UdpNode::getRemoteIP() const {
    return remote_ip_;
}

int UdpNode::getRemotePort() const {
    return remote_port_;
}

// ==================== 数据统计接口 ====================
unsigned long long UdpNode::getPacketsSent() const {
    return packets_sent_.load();
}

unsigned long long UdpNode::getPacketsReceived() const {
    return packets_received_.load();
}

void UdpNode::resetStatistics() {
    packets_sent_ = 0;
    packets_received_ = 0;
}

void UdpNode::updateStatisticsSent() {
    packets_sent_++;
}

void UdpNode::updateStatisticsReceived() {
    packets_received_++;
}

// ==================== 接收线程实现 ====================
void UdpNode::receiveLoop() {
    // 使用union来接收不同类型的数据包
    union {
        TargetPacket target;
        StatePacket state;
        FullPacket full;
        CommandPacket cmd;
    } packet;
    
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms超时
    setsockopt(sockfd_rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    while (running_) {
        ssize_t n = recvfrom(sockfd_rx, &packet, sizeof(packet), 0,
                             (struct sockaddr*)&from_addr, &from_len);
        if (n < 0) {
            continue;
        }
        
        // 根据接收到的数据包大小判断类型
        if (n == sizeof(TargetPacket)) {
            // 接收到目标位置数据
            std::lock_guard<std::mutex> lock(mutex_);
            std::copy(packet.target.target_positions, 
                     packet.target.target_positions + JOINT_COUNT,
                     target_positions_.begin());
            new_target_data_ = true;
            target_data_update_count_++;
            updateStatisticsReceived();
        }
        else if (n == sizeof(StatePacket)) {
            // 接收到状态数据
            std::lock_guard<std::mutex> lock(mutex_);
            std::copy(packet.state.current_positions,
                     packet.state.current_positions + JOINT_COUNT,
                     current_positions_.begin());
            std::copy(packet.state.current_velocities,
                     packet.state.current_velocities + JOINT_COUNT,
                     current_velocities_.begin());
            std::copy(packet.state.current_torques,
                     packet.state.current_torques + JOINT_COUNT,
                     current_torques_.begin());
            new_state_data_ = true;
            state_data_update_count_++;
            updateStatisticsReceived();
        }
        else if (n == sizeof(FullPacket)) {
            // 接收到完整数据包
            std::lock_guard<std::mutex> lock(mutex_);
            std::copy(packet.full.target_positions,
                     packet.full.target_positions + JOINT_COUNT,
                     target_positions_.begin());
            std::copy(packet.full.current_positions,
                     packet.full.current_positions + JOINT_COUNT,
                     current_positions_.begin());
            std::copy(packet.full.current_velocities,
                     packet.full.current_velocities + JOINT_COUNT,
                     current_velocities_.begin());
            std::copy(packet.full.current_torques,
                     packet.full.current_torques + JOINT_COUNT,
                     current_torques_.begin());
            new_target_data_ = true;
            new_state_data_ = true;
            target_data_update_count_++;
            state_data_update_count_++;
            updateStatisticsReceived();
        }
        else if (n == sizeof(CommandPacket)) {
            // 接收到 GUI 远程指令包
            uint32_t pkt_type = packet.cmd.type;
            if (pkt_type == 1) {  // KEY_PRESS
                char key = packet.cmd.data.key_press.key;
                queueKey(key);
                std::cout << "[UDP] GUI key press: '" << key << "'" << std::endl;
            } else if (pkt_type == 2) {  // VELOCITY_CMD
                float fwd  = packet.cmd.data.velocity.forward;
                float side = packet.cmd.data.velocity.side;
                float turn = packet.cmd.data.velocity.turn;
                queueVelocity(fwd, side, turn);
                std::cout << "[UDP] GUI velocity cmd: fwd=" << fwd
                          << " side=" << side << " turn=" << turn << std::endl;
            } else if (pkt_type == 3) {  // ESTOP
                queueEstop();
                std::cout << "[UDP] GUI ESTOP received" << std::endl;
            } else {
                std::cerr << "[UDP] Unknown command type: " << pkt_type << std::endl;
            }
            updateStatisticsReceived();
        }
        else {
            std::cerr << "Received packet of unexpected size: " << n << std::endl;
        }
    }
}

// ==================== GUI 远程指令接口实现 ====================
void UdpNode::queueKey(char key) {
    pending_key_.store(key, std::memory_order_release);
    has_pending_key_.store(true, std::memory_order_release);
}

void UdpNode::queueVelocity(float forward, float side, float turn) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_velocity_[0] = forward;
    pending_velocity_[1] = side;
    pending_velocity_[2] = turn;
    has_pending_velocity_.store(true, std::memory_order_release);
}

void UdpNode::queueEstop() {
    has_pending_estop_.store(true, std::memory_order_release);
}

bool UdpNode::hasPendingKey() const {
    return has_pending_key_.load(std::memory_order_acquire);
}

char UdpNode::consumePendingKey() {
    has_pending_key_.store(false, std::memory_order_release);
    return pending_key_.load(std::memory_order_acquire);
}

bool UdpNode::hasPendingVelocity() const {
    return has_pending_velocity_.load(std::memory_order_acquire);
}

void UdpNode::consumePendingVelocity(float& forward, float& side, float& turn) {
    has_pending_velocity_.store(false, std::memory_order_release);
    std::lock_guard<std::mutex> lock(mutex_);
    forward = pending_velocity_[0];
    side = pending_velocity_[1];
    turn = pending_velocity_[2];
}

bool UdpNode::hasPendingEstop() const {
    return has_pending_estop_.load(std::memory_order_acquire);
}

bool UdpNode::consumePendingEstop() {
    return has_pending_estop_.exchange(false, std::memory_order_acq_rel);
}
