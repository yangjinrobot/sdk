/**
 * @file basic_function.hpp
 * @brief basic function
 * @author deeprobotics
 * @version 1.0
 * @date 2024-06-05
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef BASIC_FUNCTION_HPP_
#define BASIC_FUNCTION_HPP_
#include "common_types.h"
#include "custom_types.h"
#include "parameters/control_parameters.h"
#include <bitset>

using namespace types;

class FirstOrderLowPassFilter {
private:
    double alpha;  // 平滑因子
    double y_prev; // 上一个输出值
    
public:
    FirstOrderLowPassFilter(double cutoffFreq, double sampleRate) {
        // 计算平滑因子
        double dt = 1.0 / sampleRate;
        double RC = 1.0 / (2 * M_PI * cutoffFreq);
        alpha = dt / (dt + RC);
        y_prev = 0.0;
    }
    
    double process(double x) {
        // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        double y = alpha * x + (1 - alpha) * y_prev;
        y_prev = y;
        return y;
    }
    
    void reset() {
        y_prev = 0.0;
    }
};


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
        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if (tfd == -1) {
            printf("create timer fd fail \r\n");
        }
        time_intv.it_value.tv_sec = 0; //设定2s超时
        time_intv.it_value.tv_nsec = 1000 * 1000 * ms;
        time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        printf("timer start ...\n");
        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
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
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
    }

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */
    int time_interrupt() {
        fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
        if (evptr[0].events & EPOLLIN) {
            ret = read(evptr->data.fd, &value, sizeof(uint64_t));
            if (ret == -1) {
                printf("read return 1 -1, errno :%d \r\n", errno);
                return 1;
            }
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


inline void limitEuler(double &target){
    if (target > M_PI){
        target -= 2*M_PI;
    }
    else if (target < -M_PI){
        target += 2*M_PI;
    }
};

inline void limitEuler(float &target){
    if (target > M_PI){
        target -= 2*M_PI;
    }
    else if (target < -M_PI){
        target += 2*M_PI;
    }
};

inline float Deg2Rad(float deg){
    return deg /180.f * M_PI;
}

inline float Rad2Deg(float rad){
    return rad / M_PI * 180.;
}

inline float LimitNumber(float data, float limit){  
    limit = fabs(limit);
    if(data > limit) data = limit;
    else if(data < limit) data = -limit;
    return data;
}

inline void LimitNumber(float &data, float low, float high){
    if(low > high){
        std::cerr << "error limit range" << std::endl;
        return;
    }
    if(data > high) data = high;
    else if(data < low) data = low;
}
inline float RetLimitNumber(float data, float low, float high){
    if(low > high){
        std::cerr << "error limit range" << std::endl;
        return data;
    }
    if(data > high) return high;
    if(data < low)  return low;
    return data;
}
inline Mat3f RpyToRm(const Vec3f &rpy){
    // Mat3f rm;
    Eigen::AngleAxisf yawAngle(rpy(2), Vec3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(rpy(1), Vec3f::UnitY());
    Eigen::AngleAxisf rollAngle(rpy(0), Vec3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle*pitchAngle*rollAngle;
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

inline float Sign(const float &i){
    if (i > 0) {
        return 1.;
    } else if (i == 0) {
        return 0.;
    } else {
        return -1.;
    }
}

inline void limitRange(const double &min, const double &max, double &own){
    if (min > max) {
        std::cout << "[Error] Value Setting Error!! Please Check!!!" << std::endl;
    } else {
        if (own < min) {
            own = min;
        } else if (own > max) {
            own = max;
        }
    }
};

inline void limitRange(const float &min, const float &max, float &own){
    if (min > max) {
        std::cout << "[Error] Value Setting Error!! Please Check!!!" << std::endl;
    } else {
        if (own < min) {
            own = min;
        } else if (own > max) {
            own = max;
        }
    }
};

inline double GetTimestampMs(){
    static timespec startup_timestamp;
    timespec now_timestamp;
    if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
        clock_gettime(CLOCK_MONOTONIC, &startup_timestamp);
    }
    clock_gettime(CLOCK_MONOTONIC, &now_timestamp);
    return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
}

inline float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
}
inline float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
    if(t >= T) return 0;
    float a, b, c;
    c = v0;
    a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
    b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
    return 3.*a*pow(t, 2) + 2.*b*t + c;
}
inline float GetSinSplinePos(){
    return 0.0;
}

   

inline float GetHipYPosByHeight(float h, std::shared_ptr<ControlParameters> cp_ptr_){
    float l1 = cp_ptr_->leg_link_len_(1)+cp_ptr_->leg_link_len_(2);
    float l2 = cp_ptr_->leg_link_len_(3);
    // std::cout<<"l1:"<<l1<<", l2:"<<l2<<std::endl;
    if(fabs(h) >= l1 + l2) {
        std::cerr << "error height input" << std::endl;
        return 0;
    }
    float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
    LimitNumber(theta, cp_ptr_->leg_joint_lower_(0), cp_ptr_->leg_joint_upper_(0));
    return theta;
}

inline float GetKneePosByHeight(float h, std::shared_ptr<ControlParameters> cp_ptr_){
    float l1 = cp_ptr_->leg_link_len_(1)+cp_ptr_->leg_link_len_(2);
    float l2 = cp_ptr_->leg_link_len_(3);
    if(fabs(h) >= l1 + l2) {
        std::cerr << "error height input" << std::endl;
        return 0;
    }
    float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
    LimitNumber(theta, cp_ptr_->leg_joint_lower_(3), cp_ptr_->leg_joint_upper_(3));
    return theta;
}

inline Vec3f LegInverseKinematic(Vec3f goal_xyz, std::shared_ptr<ControlParameters> cp_ptr_) 
{ 
    Vec3f angles; 
    float x, y, z; 
    float l0, l1, l2; 
    float s1, s2, s3; 

    l0 = cp_ptr_->leg_link_len_(0); l1 = cp_ptr_->leg_link_len_(1)+cp_ptr_->leg_link_len_(2); l2 = cp_ptr_->leg_link_len_(3);
    x = -goal_xyz[0]; y = -goal_xyz[1]; z = -goal_xyz[2]; 

    s1 = atan(y / z) - asin(l0 / sqrt(y*y + z*z)); 
    s3 = acos((x*x + y*y + z*z - l0*l0 - l1*l1 - l2*l2) / (2 * l1*l2)); 
    s2 = asin(x / sqrt((l1 + l2*cos(s3))*(l1 + l2*cos(s3)) + l2*sin(s3)*l2*sin(s3))) - atan(l2*sin(s3) / (l1 + l2*cos(s3))); 

    LimitNumber(s1, cp_ptr_->leg_joint_lower_(1), cp_ptr_->leg_joint_upper_(1));//hipx
    LimitNumber(s2, cp_ptr_->leg_joint_lower_(0), cp_ptr_->leg_joint_upper_(0));//hipy
    LimitNumber(s3, cp_ptr_->leg_joint_lower_(3), cp_ptr_->leg_joint_upper_(3));//knee
    angles << s1, s2, s3; 
    return angles; 
} 
inline void PrintEigenVectorf(const VecXf& vec, int width=6){
    for(int i=0;i<vec.size();++i){
        std::cout << std::setw(width)  
                << std::setprecision(3) 
                << std::right 
                << vec(i) << " ";  
    }
    std::cout << "\n";
}
inline std::string intToBinary(int num) {
    std::bitset<32> bits(num); 
    std::string binary = bits.to_string();
    size_t first_one = binary.find('1');
    if (first_one != std::string::npos) {
        binary = binary.substr(first_one);
    } else {
        binary = "0"; 
    }
    return binary;
}

#endif