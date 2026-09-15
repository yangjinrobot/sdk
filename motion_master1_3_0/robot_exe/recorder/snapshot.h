///
/// @file Snapshot.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2024-12-19
/// 
/// @copyright Copyright (c) 2024
/// 
///


#ifndef _SNAPSHOT_H_
#define _SNAPSHOT_H_

#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <stdexcept>
#ifdef USE_SIMULATION
#else
    #include "base_actor.h"
    #include "motor.h"
    #include "robot_hwi.hpp"
#endif
#include "drdds/core/drdds_core.h"
#include "log4cplus_dr/dr_logger.hpp"
// #include "wt_dog.h"
// #include "controller/controller.h"
// #include "ecattypes.h"

  typedef union {
    struct{
      float x;
      float y;
      float z;
      float roll;
      float pitch;
      float yaw;
    };
    float data[6];
  }SteerData;
  struct SnapshotFrame {
    double timestamp;                  ///< 时间戳
    std::vector<JointUnit> joint_data;    ///< 关节数据
    std::vector<JointUnit> joint_command; ///< 关节指令
    ImuData Arm_imu_data;                     ///< IMU 数据
    ImuData Leg_imu_data;                     ///< IMU 数据
    SteerData steer_data;
    uint32_t motion_gait[2];              ///< index 0：目标步态 1：当前步态
    int32_t motion_state[2];              ///< index 0：目标状态 1：当前状态
  };

/**
 * @brief 环形队列模板类，支持覆盖最旧数据。
 * 
 * @tparam T 队列中存储的元素类型。
 */
template <typename T>
class CircularQueue {
 public:
  /**
   * @brief 构造函数，初始化队列容量。
   * @param capacity 队列的最大容量。
   * @throws std::invalid_argument 如果容量小于等于 0。
   */
  explicit CircularQueue(int capacity)
      : capacity_(capacity), size_(0), head_(0), tail_(0) {
    if (capacity <= 0) {
      throw std::invalid_argument("Capacity must be greater than 0");
    }
    queue_.resize(capacity);
  }

  /**
   * @brief 判断队列是否为空。
   * @return true 如果队列为空。
   * @return false 如果队列不为空。
   */
  bool IsEmpty() const { return size_ == 0; }

  /**
   * @brief 获取队列的当前大小。
   * @return 当前队列中的元素个数。
   */
  int GetSize() const { return size_; }

  /**
   * @brief 获取队列的容量。
   * @return 队列的最大容量。
   */
  int GetCapacity() const { return capacity_; }

  /**
   * @brief 入队操作。如果队列已满，将覆盖最旧的数据。
   * @param value 入队的元素值。
   */
  void Enqueue(const T& value) {
    if (size_ == capacity_) {
      // 覆盖最旧数据，更新 head 指针
      head_ = (head_ + 1) % capacity_;
    } else {
      ++size_;
    }
    queue_[tail_] = value;
    tail_ = (tail_ + 1) % capacity_;
  }

  /**
   * @brief 出队操作。
   * @return 队列头部的元素。
   * @throws std::out_of_range 如果队列为空。
   */
  T Dequeue() {
    if (IsEmpty()) {
      throw std::out_of_range("Queue is empty!");
    }
    T value = queue_[head_];
    head_ = (head_ + 1) % capacity_;
    --size_;
    return value;
  }

  /**
   * @brief 获取队列头部的元素。
   * @return 队列头部的元素。
   * @throws std::out_of_range 如果队列为空。
   */
  T Front() const {
    if (IsEmpty()) {
      throw std::out_of_range("Queue is empty!");
    }
    return queue_[head_];
  }

  /**
   * @brief 获取队列尾部的元素。
   * @return 队列尾部的元素。
   * @throws std::out_of_range 如果队列为空。
   */
  T Back() const {
    if (IsEmpty()) {
      throw std::out_of_range("Queue is empty!");
    }
    return queue_[(tail_ - 1 + capacity_) % capacity_];
  }

 private:
  int capacity_;       ///< 队列容量。
  int size_;           ///< 当前队列大小。
  int head_;           ///< 队列头部索引。
  int tail_;           ///< 队列尾部索引。
  std::vector<T> queue_; ///< 存储队列数据的容器。
};


  class Snapshot : public BaseActor {
  public:
    Snapshot (int thread_priority = 10, int record_time = 300);

  /**
   * @brief Initializes DDS channels for different data types (Joint, IMU, Battery).
   * 
   * This function creates DDS channels for all supported data types like JointsData,
   * IMUData, and BatteryInfo. It can be extended in the future for other types.
   */
    void initializeDDSChannels() ;
    void DDSJointCmdHandler(const drdds::msg::JointsCmd *data, uint32_t size);
    void DDSJointRawDataHandler(const drdds::msg::Joints* data, uint32_t size);
    void DDSArmImuDataHandler(const sensor_msgs::msg::Imu *data);
    void DDSLegImuDataHandler(const drdds::msg::ImuData *data);
    void DDSBatteryDataHandler(const BatteryInfo& battery_info, uint32_t index);
    void DDSSteerDataHandler(const drdds::msg::Steer* data);
    void DDSMotionSattusHandler(const drdds::msg::MotionInfo* data);
    void DDSMotionStateHandler(const drdds::msg::MotionState* data);
    void DDSMotionGait(const drdds::msg::Gait* data);    
    void Write();

    ~Snapshot (){

    }
    std::vector<JointUnit>& getJointVector(){
      return snapshot_frame_.joint_data;
    }
    BatteryInfo& GetBatteryInfo() {
        return battery_info_;
    }

    std::string GetTime(void);
    void WirteToFile();

  protected:
    virtual void* RunRoutine(void*);
  private:
    ChannelMotionState* ch_motion_state_;
    ChannelGait* ch_gait_;
    ChannelSteer* ch_steer_;
    BatteryInfo battery_info_;
    std::ofstream outFile;
    int record_time_;
    CircularQueue<SnapshotFrame>* snapshot_vector_;
    bool is_writing_to_file_ = false;
    SnapshotFrame snapshot_frame_;
    DrLogger* logger_;

  };

#endif /* DEFERRED_RECORDER_H_ */
