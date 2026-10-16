/**
 * @file basic_function.hpp
 * @brief basic function
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once
#include "common_types.h"
#include "custom_types.h"

using namespace types;

class TimeTool {
private:
    int tfd;    /**< Timer descriptor.*/
    int efd;    /**< Epoll descriptor.*/
    int fds, ret; /**< Variables used to initialize the timer.*/
    uint64_t value; /**< Variables used to initialize the timer.*/
    struct epoll_event ev, *evptr; /**< Variables used to initialize the timer.*/
    struct itimerspec time_intv;  /**< Variables used to initialize the timer.*/
public:
    timespec system_time; /**< A class for accurately obtaining time.*/
    /**
    * @brief Initialize timer, input cycle(ms).
    * @param Cycle time unit: ms
    */
    void time_init(int ms) {
        tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd == -1) {
            printf("create timer fd fail \r\n");
        }
        time_intv.it_value.tv_sec = 0;
        time_intv.it_value.tv_nsec = 1000 * 1000 * ms;
        time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        printf("timer start ...\n");
        timerfd_settime(tfd, 0, &time_intv, NULL);

        efd = epoll_create1(0);
        if (efd == -1) {
            printf("create epoll fail \r\n");
            close(tfd);
        }

        evptr = (struct epoll_event *) calloc(1, sizeof(struct epoll_event));
        if (evptr == NULL) {
            printf("epoll event calloc fail \r\n");
            close(tfd);
            close(efd);
        }

        ev.data.fd = tfd;
        ev.events = EPOLLIN;
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev);
    }

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */
    int time_interrupt() {
        fds = epoll_wait(efd, evptr, 1, 10);
        if (evptr[0].events & EPOLLIN) {
            ret = read(evptr->data.fd, &value, sizeof(uint64_t));
            if (ret == -1) {
                printf("read return 1 -1, errno :%d \r\n", errno);
            }
            return 1;
        }
        return 0;
    } /**< Acquire interrupt signal.*/

    /**
    * @brief How long has it been
    * @param Initial time
    */
    double get_now_time(double start_time) {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9 - start_time;
    }

    /**
    * @brief Get current time
    */
    double get_start_time() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    } /**< Get start time.*/
};


inline float Deg2Rad(float deg) { return deg / 180.f * M_PI; }

inline float Rad2Deg(float rad) { return rad / M_PI * 180.; }

inline void ClipNumber(float& num, float low, float up) {
    if (low > up) std::cerr << "error clip" << std::endl;
    if (num < low) num = low;
    if (num > up) num = up;
}

inline float RetLimitNumber(float data, float low, float high){
    float ret = 0;
    if(low > high){
        std::cerr << "error limit range" << std::endl;
        ret = data;
        return ret;
    }
    if(data > high) ret = high;
    else if(data < low) ret = low;
    return ret;
}

inline Mat3f RpyToRm(const Vec3f &rpy) {
  Eigen::AngleAxisf yawAngle(rpy(2), Vec3f::UnitZ());
  Eigen::AngleAxisf pitchAngle(rpy(1), Vec3f::UnitY());
  Eigen::AngleAxisf rollAngle(rpy(0), Vec3f::UnitX());
  Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
  return q.matrix();
}

inline float NormalizeAngle(float angle) {
  float result = std::fmod(angle, 2 * M_PI);
  if (result < 0) {
    result += 2 * M_PI;
  }
  if (result > M_PI) {
    result -= 2 * M_PI;
  }
  return result;
}

inline float Sign(const float &i) {
  if (i > 0) {
    return 1.;
  } else if (i == 0) {
    return 0.;
  } else {
    return -1.;
  }
}

inline float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T) {
  if (t >= T) return xf;
  float a, b, c, d;
  d = x0;
  c = v0;
  a = (vf * T - 2 * xf + v0 * T + 2 * x0) / pow(T, 3);
  b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / pow(T, 2);
  return a * pow(t, 3) + b * pow(t, 2) + c * t + d;
}

inline float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T) {
  if (t >= T) return 0;
  float a, b, c;
  c = v0;
  a = (vf * T - 2 * xf + v0 * T + 2 * x0) / pow(T, 3);
  b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / pow(T, 2);
  return 3. * a * pow(t, 2) + 2. * b * t + c;
}

inline double GetTimestampMs() {
  static timespec startup_timestamp;
  timespec now_timestamp;
  if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
    clock_gettime(CLOCK_MONOTONIC, &startup_timestamp);
  }
  clock_gettime(CLOCK_MONOTONIC, &now_timestamp);
  return (now_timestamp.tv_sec - startup_timestamp.tv_sec) * 1e3 + (now_timestamp.tv_nsec - startup_timestamp.tv_nsec) / 1e6;
}
