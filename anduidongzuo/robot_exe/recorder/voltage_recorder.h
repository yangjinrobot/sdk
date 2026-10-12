/**
 * @file voltage_recorder.h
 * @brief 1Hz 电压数据 CSV 记录器
 * @version 0.1
 * @date 2026-07-11
 *
 * @copyright Copyright (c) 2026 DeepRobotics
 *
 * 按固定 1Hz 周期将最新电压快照写入 CSV 文件：
 *   /var/opt/robot/log/system/voltage_1Hz.csv
 * 文件名固定，追加写入；首次创建或空文件时写表头。
 */

#ifndef VOLTAGE_RECORDER_H_
#define VOLTAGE_RECORDER_H_

#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <mutex>
#include <filesystem>
#include "base_actor.h"
#include "types/data_structures.h"

class VoltageRecorder : public BaseActor {
public:
    VoltageRecorder();
    VoltageRecorder(int thread_priority);
    ~VoltageRecorder();

    /// 注入电压数据源：voltage 指向最新缓存，mtx 用于读写互斥
    void SetVoltageData(ecan::VoltageStatus* voltage, std::mutex* mtx) {
        voltage_ = voltage;
        mtx_ = mtx;
    }

protected:
    virtual void* RunRoutine(void*);

private:
    std::ofstream outFile;
    ecan::VoltageStatus* voltage_ = nullptr;
    std::mutex* mtx_ = nullptr;
    std::string GetTime(void);
    static bool NeedHeader(const std::string& path);
};

#endif /* VOLTAGE_RECORDER_H_ */
