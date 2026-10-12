/**
 * @file data_recorder.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DATA_RECORDER_H_
#define DATA_RECORDER_H_

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
#include "base_actor.h"
#include "motor.h"
#include "robot_hwi.hpp"

#include "drdds/core/drdds_core.h"

// #include "controller/controller.h"
// #include "ecattypes.h"

  class DataRecorder: public BaseActor {
  public:
    DataRecorder();
    DataRecorder(int thread_priority);
    void SetBatteryData(BatteryInfo* info){
      battery_info_ = info;
    }
    void SetJointRawData(std::vector<JointUnit>* data){
      raw_data_vector_ = data;
    }
    ~DataRecorder();
    int DataMapInit(std::map<uint8_t, XYZ> InitMap);

  protected:
    virtual void* RunRoutine(void*);
  private:

    std::ofstream outFile;
    const BatteryInfo* battery_info_;
    const JointRawData* raw_data_;
    std::vector<JointUnit>* raw_data_vector_;
    uint32_t raw_data_index_[kJointCount];
    uint32_t raw_joint_num_ = 0;
    bool isFirefly3588(void);
    bool is_firefly_3588_ = false;
    std::vector<int> getCPUFrequencies();
    std::vector<int> getTemperatures();
    void WriteHeader(void);
    void WriteBattery(void);
    void WriteCPUInfo(void);
    void WriteTime(void);
    void WriteMotorTemperature(void);
    void WriteDriverTemperature(void);
    std::string GetTime(void);
  };


#endif /* DEFERRED_RECORDER_H_ */
