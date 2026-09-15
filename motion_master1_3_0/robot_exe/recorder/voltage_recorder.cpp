/**
 * @file voltage_recorder.cpp
 * @brief 1Hz 电压数据 CSV 记录器实现
 * @version 0.1
 * @date 2026-07-11
 *
 * @copyright Copyright (c) 2026 DeepRobotics
 */

#include <iomanip>
#include "voltage_recorder.h"

VoltageRecorder::VoltageRecorder() {}

VoltageRecorder::VoltageRecorder(int thread_priority)
    : BaseActor(thread_priority) {}

VoltageRecorder::~VoltageRecorder() {}

std::string VoltageRecorder::GetTime(void) {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

bool VoltageRecorder::NeedHeader(const std::string& path) {
    std::ifstream check(path);
    if (!check.good()) return true;          // 文件不存在
    if (check.peek() == std::ifstream::traits_type::eof()) return true;  // 文件为空
    return false;
}

void* VoltageRecorder::RunRoutine(void*) {
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd == -1) return nullptr;

    // 1Hz 周期定时器
    struct itimerspec time_intv;
    time_intv.it_value.tv_sec = 1;
    time_intv.it_value.tv_nsec = 0;
    time_intv.it_interval.tv_sec = 1;
    time_intv.it_interval.tv_nsec = 0;
    timerfd_settime(tfd, 0, &time_intv, NULL);

    int efd = epoll_create1(0);
    if (efd == -1) {
        close(tfd);
        return nullptr;
    }

    struct epoll_event ev, *evptr;
    evptr = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
    if (evptr == nullptr) {
        close(tfd);
        close(efd);
        return nullptr;
    }
    ev.data.fd = tfd;
    ev.events = EPOLLIN;
    epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev);

    // 固定路径，追加模式；首次或空文件时写表头
    const std::string file_path = "/var/opt/robot/log/system/voltage_1Hz.csv";
    std::error_code ec;
    std::filesystem::create_directories("/var/opt/robot/log/system", ec);

    bool need_header = NeedHeader(file_path);
    outFile.open(file_path, std::ios::out | std::ios::app);
    if (need_header) {
        outFile << "timestamp,bat_voltage,up_voltage,down_voltage,valid" << std::endl;
    }

    // 与其它 BaseActor 同步启动
    pthread_barrier_wait(&barrier_);

    while (1) {
        int fds = epoll_wait(efd, evptr, 1, -1);
        if (fds > 0 && (evptr[0].events & EPOLLIN)) {
            uint64_t value;
            read(evptr->data.fd, &value, sizeof(uint64_t));
        }

        // 加锁拷贝最新电压快照（handlVoltageStatus 在另一线程写入）
        ecan::VoltageStatus snap;
        if (mtx_ && voltage_) {
            std::lock_guard<std::mutex> lock(*mtx_);
            snap = *voltage_;
        }

        outFile << GetTime() << ","
                << snap.bat_voltage << ","
                << snap.up_voltage << ","
                << snap.down_voltage << ","
                << static_cast<int>(snap.valid)
                << std::endl;
        outFile.flush();
    }
    return nullptr;
}
