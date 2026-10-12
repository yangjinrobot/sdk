/**
 * @file data_streaming.hpp
 * @brief 
 * @author deeprobotics
 * @version 1.0
 * @date 2024-04-28
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef DATA_STREAMING_HPP_
#define DATA_STREAMING_HPP_

#include "json.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "basic_function.hpp"
#include "toml/toml.hpp"
#include "runtime_params.hpp"
#include <iomanip> 
#include <filesystem>
#include <mutex>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>

#define LOCAL_PORT 10121
using json = nlohmann::json;

class DataStreaming{
private:
    int sockfd_;
    sockaddr_in remote_addr_;
    int remote_port_;
    std::string remote_ip_;

    int record_freq_ = 5;

    std::thread record_thread_;

    FILE *fp_;
    std::string file_name_;
    std::ofstream file_;
    std::vector<float> scope_data_;
    
    nlohmann::json data_json_;
    int run_cnt_ = 0;
    int overrun_count_ = 0;
    long overrun_max_us_ = 0;
    std::chrono::steady_clock::time_point last_overrun_log_time_ = std::chrono::steady_clock::now();
    bool enable_online_plot_;
    bool enable_data_record_;

    float time_stamp_;

    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;

    // 文件轮转相关变量（base_directory_ 仅用于轮转 CSV 与 CleanupOldFiles）
    std::string base_directory_;
    std::string record_directory_;    // enable_data_record 单文件 CSV 目录，与轮转隔离
    std::string file_prefix_;
    long max_file_size_;              // 单个文件最大大小（字节）
    int max_file_count_;              // 最大文件数量
    bool enable_file_rotation_;       // 是否启用文件轮转
    int rotation_fd_ = -1;            // 当前轮转文件描述符（O_DIRECT直写）
    std::string current_file_path_;   // 当前文件路径
    long current_file_size_;          // 当前文件大小
    std::mutex file_mutex_;           // 文件访问互斥锁

    // O_DIRECT 对齐缓冲区
    static constexpr size_t DIRECT_BLOCK_SIZE = 4096;
    static constexpr size_t DIRECT_BUF_SIZE = DIRECT_BLOCK_SIZE * 16;  // 64KB
    char* direct_buf_ = nullptr;
    size_t direct_buf_pos_ = 0;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf posraw = ri_ptr_->GetJointPositionRaw();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        VecXf temp = ri_ptr_->GetMotorTemperture();
        VecXf d_temp = ri_ptr_->GetDriverTemperture();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();
        std::vector<uint16_t> battery_data = ri_ptr_->GetBatteryData();

        this->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());

        this->InsertBatteryData("battery_level", battery_data[0]);
        this->InsertBatteryData("battery_voltage", battery_data[1]/100.);
        this->InsertBatteryData("battery_current", battery_data[2]/100.);
        this->InsertBatteryData("battery_state", battery_data[3]);

        // ========== 关节空间 ==========
        const char* q_key   = "q";
        const char* dq_key  = "dq";
        const char* tau_key = "tau";
        const char* q_cmd_key   = "q_cmd";
        const char* dq_cmd_key  = "dq_cmd";
        const char* tau_cmd_key = "tau_cmd";
        this->InsertJointData(q_key, pos);       // 关节位置
        this->InsertJointData(dq_key, vel);      // 关节速度
        this->InsertJointData(tau_key, tau);     // 关节力矩
        this->InsertJointData(q_cmd_key, jc.col(1));   // 位置指令
        this->InsertJointData(dq_cmd_key, jc.col(3));  // 速度指令
        VecXf tau_cmd = jc.col(0).cwiseProduct(jc.col(1) - pos)
                      + jc.col(2).cwiseProduct(jc.col(3) - vel) + jc.col(4);
        this->InsertJointData(tau_cmd_key, tau_cmd);  // 力矩指令（PD计算值）

        // ========== 其他 ==========
        this->InsertJointData("motor_emperture", temp);
        this->InsertJointData("driver_emperture", d_temp);

        this->InsertImuData("rpy", rpy);
        this->InsertImuData("acc", acc);
        this->InsertImuData("omg", omg);
        this->InsertCommandData("forward_vel_scale", uc_ptr_->GetUserCommand()->forward_vel_scale);
        this->InsertCommandData("side_vel_scale", uc_ptr_->GetUserCommand()->side_vel_scale);
        this->InsertCommandData("turnning_vel_scale", uc_ptr_->GetUserCommand()->turnning_vel_scale);
        if(!uc_ptr_) return;
        // this->InsertCommandData("target_mode", float(int(uc_ptr_->GetUserCommand()->target_mode)));
        // this->InsertStateData("current_state", float(int(StateBase::msfb_.current_state)));
        // this->SendData();
    }

    // 生成带时间戳的新文件名
    std::string GenerateNewFileName() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
        
        return (std::filesystem::path(base_directory_) / (file_prefix_ + "_" + ss.str() + ".csv")).string();
    }

    // O_DIRECT 对齐写入：数据先进缓冲区，满块时直写磁盘
    void DirectWrite(const std::string& data) {
        if (rotation_fd_ < 0 || !direct_buf_) return;
        const char* src = data.c_str();
        size_t remaining = data.size();

        while (remaining > 0) {
            size_t space = DIRECT_BUF_SIZE - direct_buf_pos_;
            size_t to_copy = std::min(remaining, space);
            ::memcpy(direct_buf_ + direct_buf_pos_, src, to_copy);
            direct_buf_pos_ += to_copy;
            src += to_copy;
            remaining -= to_copy;

            // 写出已满的对齐块
            size_t aligned = (direct_buf_pos_ / DIRECT_BLOCK_SIZE) * DIRECT_BLOCK_SIZE;
            if (aligned > 0) {
                ssize_t written = ::write(rotation_fd_, direct_buf_, aligned);
                if (written > 0) current_file_size_ += written;
                size_t leftover = direct_buf_pos_ - aligned;
                if (leftover > 0) ::memmove(direct_buf_, direct_buf_ + aligned, leftover);
                direct_buf_pos_ = leftover;
            }
        }
    }

    // 刷出缓冲区剩余非对齐数据：临时关闭 O_DIRECT 写出尾部
    void DirectFlushRemaining() {
        if (direct_buf_pos_ > 0 && rotation_fd_ >= 0) {
            int flags = ::fcntl(rotation_fd_, F_GETFL);
            ::fcntl(rotation_fd_, F_SETFL, flags & ~O_DIRECT);
            ssize_t written = ::write(rotation_fd_, direct_buf_, direct_buf_pos_);
            ::fdatasync(rotation_fd_);
            ::fcntl(rotation_fd_, F_SETFL, flags);
            if (written > 0) current_file_size_ += written;
            direct_buf_pos_ = 0;
        }
    }

    // 检查并轮转文件
    void CheckAndRotateFile() {
        if (!enable_file_rotation_) return;

        std::lock_guard<std::mutex> lock(file_mutex_);

        // 如果当前文件不存在或大小超过限制，则创建新文件
        if (rotation_fd_ < 0 || current_file_size_ >= max_file_size_) {
            // 刷出剩余数据并关闭当前文件
            if (rotation_fd_ >= 0) {
                DirectFlushRemaining();
                ::close(rotation_fd_);
                rotation_fd_ = -1;
            }

            // 生成新文件名
            current_file_path_ = GenerateNewFileName();
            
            // 以 O_DIRECT 模式创建新文件
            rotation_fd_ = ::open(current_file_path_.c_str(),
                                  O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
            if (rotation_fd_ >= 0) {
                direct_buf_pos_ = 0;
                WriteRotationFileHeader();
                current_file_size_ = GetFileSize(current_file_path_);
                
                std::cout << "Created new rotation file: " << current_file_path_ 
                          << ", Size: " << current_file_size_ << " bytes" << std::endl;
            } else {
                std::cerr << "Failed to create rotation file: " << current_file_path_ << std::endl;
                return;
            }

            // 检查并清理旧文件
            CleanupOldFiles();
        }
    }

    // 写入文件头部（POSIX直写）
    void WriteRotationFileHeader() {
        if (rotation_fd_ < 0) return;

        std::ostringstream oss;
        oss << "run_cnt" << "," << "t_r" << "," << "t_i" << ",";
        for(int i=0;i<3;++i){
            oss << "rpy"+std::to_string(i) << ",";
        }
        for(int i=0;i<3;++i){
            oss << "acc"+std::to_string(i) << ",";
        }
        for(int i=0;i<3;++i){
            oss << "omg"+std::to_string(i) << ",";
        }
        std::string q_joint = "q";
        std::string dq_joint = "dq";
        std::string tau_joint = "tau";
        std::string qcmd = "q_cmd";
        std::string dqcmd = "dq_cmd";
        std::string taucmd = "tau_cmd";
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << q_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << qcmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << dqcmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << dq_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << tau_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << taucmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << "motor_emperture"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            oss << "driver_emperture"+std::to_string(i) << ",";
        }
        oss << "battery_level"<<","<< "battery_vol"<<","<< "battery_cur";
        oss << "\n";

        DirectWrite(oss.str());
    }

    // 清理旧文件
    void CleanupOldFiles() {
        if (max_file_count_ <= 0) return;

        // 获取目录下所有匹配的文件
        std::vector<std::string> files;
        std::string search_pattern = file_prefix_ + "_";
        
        for (const auto& entry : std::filesystem::directory_iterator(base_directory_)) {
            if (entry.is_regular_file() && 
                entry.path().extension() == ".csv" &&
                entry.path().filename().string().substr(0, search_pattern.length()) == search_pattern) {
                
                // 检查是否不是当前正在写的文件
                if (entry.path().string() != current_file_path_) {
                    files.push_back(entry.path().string());
                }
            }
        }

        // 按修改时间排序（最早的在前面）
        std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
            return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
        });

        // 删除多余的旧文件
        int files_to_delete = static_cast<int>(files.size()) - max_file_count_ + 1; // +1 因为我们刚创建了一个新文件
        if (files_to_delete > 0) {
            for (int i = 0; i < files_to_delete; ++i) {
                try {
                    std::filesystem::remove(files[i]);
                    std::cout << "Deleted old rotation file: " << files[i] << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Failed to delete file " << files[i] 
                              << ": " << e.what() << std::endl;
                }
            }
        }
    }

    // 获取文件大小
    long GetFileSize(const std::string& filepath) {
        struct stat st;
        if (stat(filepath.c_str(), &st) == 0) {
            return st.st_size;
        }
        return 0;
    }

    // 写入轮转文件数据
    void WriteDataToRotationFile() {
        if (!enable_file_rotation_ || !ri_ptr_) return;

        // 检查是否需要轮转文件
        CheckAndRotateFile();

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);

        // 构建数据行
        auto writeJsonFloatArray = [&](const json& arr) {
            for (size_t i = 0; i < arr.size(); ++i) {
                oss << static_cast<float>(arr[i]);
                if (i != arr.size() - 1) {
                    oss << ",";
                }
            }
        };

        oss << static_cast<float>(run_cnt_) << ","
            << static_cast<float>(data_json_["t_r"]) << ","
            << static_cast<float>(data_json_["t_i"]);

        if (!data_json_["imu"]["rpy"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["rpy"]);
        }
        if (!data_json_["imu"]["acc"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["acc"]);
        }
        if (!data_json_["imu"]["omg"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["omg"]);
        }

        const char* q_key   = "q";
        const char* dq_key  = "dq";
        const char* tau_key = "tau";
        const char* q_cmd_key   = "q_cmd";
        const char* dq_cmd_key  = "dq_cmd";
        const char* tau_cmd_key = "tau_cmd";

        if (!data_json_["joint"][q_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][q_key]);
        }
        if (!data_json_["joint"][q_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][q_cmd_key]);
        }
        if (!data_json_["joint"][dq_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][dq_cmd_key]);
        }
        if (!data_json_["joint"][dq_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][dq_key]);
        }
        if (!data_json_["joint"][tau_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][tau_key]);
        }
        if (!data_json_["joint"][tau_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][tau_cmd_key]);
        }
        if (!data_json_["joint"]["motor_emperture"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"]["motor_emperture"]);
        }
        if (!data_json_["joint"]["driver_emperture"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"]["driver_emperture"]);
        }

        if (!data_json_["battery"].empty()) {
            oss << "," << static_cast<float>(data_json_["battery"]["battery_level"]) << ","
                << static_cast<float>(data_json_["battery"]["battery_voltage"]) << ","
                << static_cast<float>(data_json_["battery"]["battery_current"]);
        }

        oss << "\n";

        std::string data = oss.str();
        std::lock_guard<std::mutex> lock(file_mutex_);
        if (rotation_fd_ >= 0) {
            DirectWrite(data);
        }
    }

public:
    DataStreaming(bool enable_online_plot=false, bool enable_data_record=false, 
                  const std::string& ip="10.21.41.208", int port=9870)://虚拟机无线ip172.16.12.242，有线ip192.168.1.165
    enable_online_plot_(enable_online_plot),
    enable_data_record_(enable_data_record),
    remote_port_(port),
    remote_ip_(ip),
    base_directory_("../data"),      // 默认目录
    file_prefix_("data"),           // 默认前缀
    max_file_size_(50*1024*1024),  // 默认50MB
    max_file_count_(10),            // 默认最多10个文件
    enable_file_rotation_(false) {  // 默认不启用文件轮转
    
        std::string config_path;
        for (const char* candidate : {
                //  "/opt/robot/share/motion_master/conf/config.toml"}) {
                "/var/opt/robot/conf/motion_master/config.toml"}) {
            if (std::filesystem::exists(candidate)) {
                config_path = candidate;
                break;
            }
        }

        if (!config_path.empty()) {
        try {
            auto value = toml::parse(config_path);
            std::cout << "data_stream: config file " << config_path << std::endl;
            
            // 解析在线绘图相关配置
            std::string model_ = toml::find<std::string>(value, "data_stream", "enable_online_plot");
            if(model_=="true") {enable_online_plot_ = true; std::cout<<"enable_online_plot!"<<std::endl;}

            // 解析数据记录相关配置
            model_ = toml::find<std::string>(value, "data_stream", "enable_data_record");
            if(model_=="true") {enable_data_record_ = true; std::cout<<"enable_data_record!"<<std::endl;}

            // 解析记录频率
            model_ = toml::find<std::string>(value, "data_stream", "record_freq");
            record_freq_ = stoi(model_);
            
            // 解析IP和端口
            model_ = toml::find<std::string>(value, "data_stream", "ip");
            remote_ip_ = model_;

            model_ = toml::find<std::string>(value, "data_stream", "port");
            remote_port_ = stoi(model_);

            // 解析文件轮转相关配置
            try {
                // 启用文件轮转
                model_ = toml::find<std::string>(value, "data_stream", "enable_file_rotation");
                if(model_=="true") {enable_file_rotation_ = true; std::cout<<"enable_file_rotation!"<<std::endl;}

                // 最大文件大小 (单位MB)
                model_ = toml::find<std::string>(value, "data_stream", "max_file_size_mb");
                if(!model_.empty()) {
                    max_file_size_ = static_cast<long>(stoi(model_)) * 1024 * 1024;
                    std::cout << "Max file size set to: " << max_file_size_ / (1024*1024) << " MB" << std::endl;
                }

                // 最大文件数量
                model_ = toml::find<std::string>(value, "data_stream", "max_file_count");
                if(!model_.empty()) {
                    max_file_count_ = stoi(model_);
                    std::cout << "Max file count set to: " << max_file_count_ << std::endl;
                }

                // 基础目录
                model_ = toml::find<std::string>(value, "data_stream", "base_directory");
                if(!model_.empty()) {
                    base_directory_ = model_;
                    std::cout << "Base directory set to: " << base_directory_ << std::endl;
                }

                // 文件前缀
                model_ = toml::find<std::string>(value, "data_stream", "file_prefix");
                if(!model_.empty()) {
                    file_prefix_ = model_;
                    std::cout << "File prefix set to: " << file_prefix_ << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "File rotation configuration not found, using defaults" << std::endl;
            }

            try {
                model_ = toml::find<std::string>(value, "data_stream", "record_directory");
                if (!model_.empty()) {
                    record_directory_ = model_;
                    std::cout << "Record directory set to: " << record_directory_ << std::endl;
                }
            } catch (const std::exception&) {}

        } catch (const toml::syntax_error& e) {
            std::cout << "Warning: Error parsing TOML config: " << e.what() << std::endl;
            std::cout << "Using default values" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Warning: Error reading TOML config: " << e.what() << std::endl;
            std::cout << "Using default values" << std::endl;
        }
        } else {
            std::cout << "Warning: data_stream config not found (/opt/... or conf/config.toml), using defaults"
                      << std::endl;
        }

        if (record_directory_.empty()) {
            record_directory_ = base_directory_ + "/record";
        }

        // 数值参数合法性校验，非法时回退安全默认值
        if (record_freq_ <= 0) {
            std::cout << "Warning: invalid record_freq=" << record_freq_ << ", reset to 5" << std::endl;
            record_freq_ = 5;
        }
        if (remote_port_ <= 0 || remote_port_ > 65535) {
            std::cout << "Warning: invalid port=" << remote_port_ << ", reset to 9870" << std::endl;
            remote_port_ = 9870;
        }
        if (max_file_size_ <= 0) {
            std::cout << "Warning: invalid max_file_size_mb, reset to 50 MB" << std::endl;
            max_file_size_ = 50L * 1024 * 1024;
        }
        if (max_file_count_ <= 0) {
            std::cout << "Warning: invalid max_file_count=" << max_file_count_ << ", reset to 10" << std::endl;
            max_file_count_ = 10;
        }

        // 上报实际生效值至运行期参数快照
        {
            auto& rp = RuntimeParams::Instance();
            rp.SetBool  ("data_stream", "enable_online_plot",   enable_online_plot_);
            rp.SetBool  ("data_stream", "enable_data_record",   enable_data_record_);
            rp.SetBool  ("data_stream", "enable_file_rotation", enable_file_rotation_);
            rp.SetInt   ("data_stream", "record_freq",          static_cast<long long>(record_freq_));
            rp.SetString("data_stream", "ip",                   remote_ip_);
            rp.SetInt   ("data_stream", "port",                 static_cast<long long>(remote_port_));
        }

        for(int i=0;i<100;++i){//初始化，原因未知，貌似只在3588上出现，ubuntu版本问题或者arm问题
            this->InsertImuData("rpy", Vec3f::Zero());
            this->InsertImuData("acc", Vec3f::Zero());
            this->InsertImuData("omg", Vec3f::Zero());
            // this->InsertStateData("current_state", 0.f);
        }
    }
    
    ~DataStreaming(){
        if (rotation_fd_ >= 0) {
            DirectFlushRemaining();
            ::close(rotation_fd_);
            rotation_fd_ = -1;
        }
        if (direct_buf_) {
            ::free(direct_buf_);
            direct_buf_ = nullptr;
        }
    }

    void SetRobotDataSource(std::shared_ptr<interface::RobotInterface> ri){
        ri_ptr_ = ri;
    }
    void SetUserCommandDataSource(std::shared_ptr<interface::UserCommandInterface> uc){
        uc_ptr_ = uc;
    }

    void Start(){
        if(enable_online_plot_){
            std::cout << "You can plot data online on plotjuggler" << std::endl;
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&remote_addr_, 0, sizeof(remote_addr_));
            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(remote_port_);
            remote_addr_.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

            // 设置本地地址和端口号
            struct sockaddr_in local_addr;
            std::memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 绑定到所有网络接口
            local_addr.sin_port = htons(LOCAL_PORT); // 设置本地端口号

            // 绑定 socket 到本地地址和端口号
            if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
                std::cerr << "Failed to bind socket" << std::endl;
            }
            if (sockfd_ < 0) {
                perror("robot sender socket creation failed");
            }
        }

        if(enable_data_record_){
            if (!std::filesystem::exists(record_directory_)) {
                std::filesystem::create_directories(record_directory_);
            }
            std::time_t now = std::time(nullptr);
            std::tm* localTime = std::localtime(&now);
            char buffer[100];
            std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", localTime);
            file_name_ = (std::filesystem::path(record_directory_) / (std::string(buffer) + ".csv")).string();

            std::cout << "data is recorded in " << file_name_ << std::endl;
            file_.open(file_name_);
            file_.close();

            char username[256];
            if (getlogin_r(username, sizeof(username)) == 0) {
                std::cout << "Username: " << username << std::endl;
            } else {
                std::cerr << "Failed to get username" << std::endl;
            }

            std::string command = "chown " + std::string(username) + ":" + std::string(username) + " " + file_name_;
            int result = system(command.c_str());
            if (result == 0) {
                std::cout << "File permissions changed to 0666 (read and write for all users)" << std::endl;
            } else {
                std::cerr << "Failed to change file permissions" << std::endl;
            }

            file_.open(file_name_);
            if (!file_.is_open()) {
                std::cerr << "Failed to open file " << file_name_ << std::endl;
            }
        }

        // 初始化文件轮转系统
        if(enable_file_rotation_){
            // 确保基础目录存在
            if (!std::filesystem::exists(base_directory_)) {
                std::filesystem::create_directories(base_directory_);
            }
            
            // 分配 O_DIRECT 对齐缓冲区
            if (!direct_buf_) {
                if (::posix_memalign((void**)&direct_buf_, DIRECT_BLOCK_SIZE, DIRECT_BUF_SIZE) != 0) {
                    std::cerr << "Failed to allocate aligned buffer for O_DIRECT" << std::endl;
                    enable_file_rotation_ = false;
                    return;
                }
                direct_buf_pos_ = 0;
            }

            // 以 O_DIRECT 模式创建第一个轮转文件
            current_file_path_ = GenerateNewFileName();
            rotation_fd_ = ::open(current_file_path_.c_str(),
                                  O_WRONLY | O_CREAT | O_TRUNC | O_DIRECT, 0644);
            if (rotation_fd_ < 0) {
                std::cerr << "Failed to open rotation file " << current_file_path_ << std::endl;
                enable_file_rotation_ = false;
                ::free(direct_buf_);
                direct_buf_ = nullptr;
                return;
            }
            
            WriteRotationFileHeader();
            current_file_size_ = GetFileSize(current_file_path_);
            
            std::cout << "Started file rotation recording:" << std::endl;
            std::cout << "  File: " << current_file_path_ << std::endl;
            std::cout << "  Max file size: " << max_file_size_ / (1024*1024) << " MB" << std::endl;
            std::cout << "  Max file count: " << max_file_count_ << std::endl;
            std::cout << "  Rotation directory: " << base_directory_ << std::endl;
        }

        scope_data_.resize(10, 0);        

        time_stamp_ = 0.f;
        record_thread_ = std::thread(std::bind(&DataStreaming::SendData, this));
    }

    void Stop(){
        record_thread_.join();
        
        if (rotation_fd_ >= 0) {
            DirectFlushRemaining();
            ::close(rotation_fd_);
            rotation_fd_ = -1;
        }
        
        if (file_.is_open()) {
            file_.close();
        }
    }

    void SendData(){
        while(true){
            auto loop_start = std::chrono::steady_clock::now();

            if(data_json_["t_i"]!=time_stamp_){
                ++run_cnt_;
                // std::cout << "run_cnt: " <<run_cnt_ << std::endl;
                if(run_cnt_ == 1&&enable_data_record_) WriteItemNameToCsv();
            }
            if(run_cnt_%record_freq_==0){
                this->GetDataStreaming();
                float ts = GetTimestampMs();
                data_json_["t_r"] = ts/1000.;
                data_json_["scope"] = scope_data_;
                
                if(enable_data_record_){
                    WriteDataToCsv();
                }
                
                // 新增：写入轮转文件
                if(enable_file_rotation_){
                    WriteDataToRotationFile();
                }
                
                if(enable_online_plot_){
                    // [DEBUG] 验证tau字段是否存在于JSON中（确认后删除）
                    static int dbg_cnt = 0;
                    if (++dbg_cnt % 500 == 1) {
                        auto& j = data_json_["joint"];
                        std::cout << "[JSON-DBG] tau_rp_cmd exists=" << j.contains("tau_rp_cmd")
                                  << " size=" << (j.contains("tau_rp_cmd") ? j["tau_rp_cmd"].size() : -1)
                                  << " | tau_ab_cmd exists=" << j.contains("tau_ab_cmd")
                                  << " size=" << (j.contains("tau_ab_cmd") ? j["tau_ab_cmd"].size() : -1)
                                  << " | tau_rp exists=" << j.contains("tau_rp")
                                  << " size=" << (j.contains("tau_rp") ? j["tau_rp"].size() : -1)
                                  << " | json_len=" << data_json_.dump().size()
                                  << std::endl;
                    }
                    std::string json_str = data_json_.dump();
                    int nbytes = sendto(sockfd_, json_str.c_str(), json_str.length(), 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
                    if(nbytes < 0){
                        std::cerr << "Error: Could not send message" << std::endl;
                    }
                }
            }
            auto loop_end = std::chrono::steady_clock::now();
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
            if (elapsed_us > 1000) {
                ++overrun_count_;
                if (elapsed_us > overrun_max_us_) overrun_max_us_ = elapsed_us;
                auto now_sec = std::chrono::steady_clock::now();
                auto since_last = std::chrono::duration_cast<std::chrono::seconds>(now_sec - last_overrun_log_time_).count();
                if (since_last >= 5) {
                    std::cerr << "[DataStreaming WARN] loop overrun in last 5s: count=" << overrun_count_
                              << ", max=" << overrun_max_us_ << " us, run_cnt=" << run_cnt_ << std::endl;
                    overrun_count_ = 0;
                    overrun_max_us_ = 0;
                    last_overrun_log_time_ = now_sec;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    void InsertInterfaceTime(float data){
        data_json_["t_i"] = data;
    }

    void InsertJointData(const std::string &name, float data){
        data_json_["joint"][name] = data;
    }
        
    void InsertJointData(const std::string &name, const VecXf& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["joint"][name] = vec;
    }    

    void InsertImuData(const std::string &name, float data){
        data_json_["imu"][name] = data;
    }

    void InsertImuData(const std::string &name, const Vec3f& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["imu"][name] = vec;
    }

    void InsertCommandData(const std::string &name, float data){
        data_json_["usr_cmd"][name] = data;
    }

    void InsertBatteryData(const std::string &name, float data){
        data_json_["battery"][name] = data;
    }

    void InsertScopeData(int index, float data){
        scope_data_[index] = data;
    }

    void WriteItemNameToCsv(){
        file_ << "run_cnt" << "," << "t_r" << "," << "t_i" << ",";
        for(int i=0;i<3;++i){
            file_ << "rpy"+std::to_string(i) << ",";
        }
        for(int i=0;i<3;++i){
            file_ << "acc"+std::to_string(i) << ",";
        }
        for(int i=0;i<3;++i){
            file_ << "omg"+std::to_string(i) << ",";
        }
        std::string q_joint = "q";
        std::string dq_joint = "dq";
        std::string tau_joint = "tau";
        std::string qcmd = "q_cmd";
        std::string dqcmd = "dq_cmd";
        std::string taucmd = "tau_cmd";
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << q_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << qcmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << dqcmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << dq_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << tau_joint+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << taucmd+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "motor_emperture"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "driver_emperture"+std::to_string(i) << ",";
        }
        file_ << "battery_level"<<","<< "battery_vol"<<","<< "battery_cur";
        file_ << "\n";
    }
    
    void WriteDataToCsv() {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);

        auto writeJsonFloatArray = [&](const json& arr) {
            for (size_t i = 0; i < arr.size(); ++i) {
                oss << static_cast<float>(arr[i]);
                if (i != arr.size() - 1) {
                    oss << ",";
                }
            }
        };

        oss << static_cast<float>(run_cnt_) << ","
            << static_cast<float>(data_json_["t_r"]) << ","
            << static_cast<float>(data_json_["t_i"]);

        if (!data_json_["imu"]["rpy"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["rpy"]);
        }
        if (!data_json_["imu"]["acc"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["acc"]);
        }
        if (!data_json_["imu"]["omg"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["imu"]["omg"]);
        }

        const char* q_key   = "q";
        const char* dq_key  = "dq";
        const char* tau_key = "tau";
        const char* q_cmd_key   = "q_cmd";
        const char* dq_cmd_key  = "dq_cmd";
        const char* tau_cmd_key = "tau_cmd";

        if (!data_json_["joint"][q_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][q_key]);
        }
        if (!data_json_["joint"][q_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][q_cmd_key]);
        }
        if (!data_json_["joint"][dq_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][dq_cmd_key]);
        }
        if (!data_json_["joint"][dq_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][dq_key]);
        }
        if (!data_json_["joint"][tau_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][tau_key]);
        }
        if (!data_json_["joint"][tau_cmd_key].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"][tau_cmd_key]);
        }
        if (!data_json_["joint"]["motor_emperture"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"]["motor_emperture"]);
        }
        if (!data_json_["joint"]["driver_emperture"].empty()) {
            oss << ",";
            writeJsonFloatArray(data_json_["joint"]["driver_emperture"]);
        }

        if (!data_json_["battery"].empty()) {
            oss << "," << static_cast<float>(data_json_["battery"]["battery_level"]) << ","
                << static_cast<float>(data_json_["battery"]["battery_voltage"]) << ","
                << static_cast<float>(data_json_["battery"]["battery_current"]);
        }

        oss << "\n";
        file_ << oss.str() << std::flush;
    }

    // 获取当前轮转文件信息
    std::string GetCurrentRotationFile() const {
        return current_file_path_;
    }

    long GetCurrentFileSize() const {
        return current_file_size_;
    }

    int GetCurrentFileCount() const {
        int count = 0;
        std::string search_pattern = file_prefix_ + "_";
        
        for (const auto& entry : std::filesystem::directory_iterator(base_directory_)) {
            if (entry.is_regular_file() && 
                entry.path().extension() == ".csv" &&
                entry.path().filename().string().substr(0, search_pattern.length()) == search_pattern) {
                count++;
            }
        }
        return count;
    }

    // 获取所有轮转文件列表
    std::vector<std::string> GetAllRotationFiles() const {
        std::vector<std::string> files;
        std::string search_pattern = file_prefix_ + "_";
        
        for (const auto& entry : std::filesystem::directory_iterator(base_directory_)) {
            if (entry.is_regular_file() && 
                entry.path().extension() == ".csv" &&
                entry.path().filename().string().substr(0, search_pattern.length()) == search_pattern) {
                files.push_back(entry.path().string());
            }
        }
        
        // 按修改时间排序（最新的在前面）
        std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
            return std::filesystem::last_write_time(a) > std::filesystem::last_write_time(b);
        });
        
        return files;
    }
};
#endif