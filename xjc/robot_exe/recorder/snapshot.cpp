///
/// @file motion_data_recorder.cpp
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2024-12-16
/// 
/// @copyright Copyright (c) 2024
/// 
///
#include <time.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
 
#include "snapshot.h"
#include "robot_model.hpp"


using namespace ecan;
using namespace std;

Snapshot::Snapshot(int thread_priority, int record_time):
  BaseActor(thread_priority),
  record_time_(record_time),
  is_writing_to_file_(false) {
  
  initializeDDSChannels();
}
void* Snapshot::RunRoutine(void*){

  int tfd;    //定时器描述符
  int efd;    //epoll描述符
  int fds, ret;
  uint64_t value;
  struct epoll_event ev, *evptr;
  struct itimerspec time_intv; //用来存储时间

  tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
  if(tfd == -1) {
  }

  time_intv.it_value.tv_sec = 0; 
  time_intv.it_value.tv_nsec = 5*1000*1000;
  time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   
  time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

  timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器
  
  efd = epoll_create1(0); //创建epoll实例
  if (efd == -1) {
      close(tfd);
  }
  
  evptr = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
  if (evptr == NULL) {
      close(tfd);
      close(efd);
  }

  ev.data.fd = tfd; 
  ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
  epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中

  pthread_barrier_wait(&barrier_);
  while(1){

    fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
    if(evptr[0].events & EPOLLIN){   
      ret = read(evptr->data.fd, &value, sizeof(uint64_t));
      if (ret == -1) {
          continue;
      }
      else{

      }               
    }
    timespec now_time;
    clock_gettime(CLOCK_MONOTONIC, &now_time);
    snapshot_frame_.timestamp = now_time.tv_sec + now_time.tv_nsec/1e9;
    if(!is_writing_to_file_){
      snapshot_vector_->Enqueue(snapshot_frame_);
    }
    if(is_writing_to_file_ == true){
      WirteToFile();
    }
    
  }
}

std::string Snapshot::GetTime(void){
  auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
//转为字符串
  std::stringstream ss;
  // 可以分别以不同的形式进行显示
//	ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
  ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
//	ss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");

  std::string str_time = ss.str();
  return str_time;
}
void Snapshot::WirteToFile(){
  is_writing_to_file_ = true;
  int size_vector = snapshot_vector_->GetSize();
  double start_time = snapshot_vector_->Front().timestamp;
  std::string filename = "/var/opt/robot/log/system/snapshot_"+GetTime()+".csv";
  logger_->info("Writing to file %s , with a total recording duration of %+3.3f seconds", filename.c_str(), (float)size_vector/200.0);
  outFile.open(filename, std::ios::out);
  outFile << "Time s" << "," ;
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Pos " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Vel " << i << "," ;
  }      
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Tor " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint status " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint motor temp " << i << "," ;
  }
    for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint driver temp " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Cmd Pos " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Cmd Vel " << i << "," ;
  }      
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint Cmd Tor " << i << "," ;
  }
  for(int i = 0; i < snapshot_frame_.joint_data.size(); i++){
    outFile << "Joint control " << i << "," ;
  }
  outFile << "Arm_IMU acc_x"  << ","
          << "Arm_IMU acc_y"  << ","
          << "Arm_IMU acc_z"  << ","
          << "Arm_IMU roll"  << ","
          << "Arm_IMU pitch"  << ","
          << "Arm_IMU yaw"  << ","
          << "Arm_IMU omega_x"  << ","
          << "Arm_IMU omega_y"  << ","
          << "Arm_IMU omega_z" << ",";
  outFile << "LEG_IMU acc_x"  << ","
          << "LEG_IMU acc_y"  << ","
          << "LEG_IMU acc_z"  << ","
          << "LEG_IMU roll"  << ","
          << "LEG_IMU pitch"  << ","
          << "LEG_IMU yaw"  << ","
          << "LEG_IMU omega_x"  << ","
          << "LEG_IMU omega_y"  << ","
          << "LEG_IMU omega_z" << ",";
  outFile << "Steer x"  << ","
          << "Steer y"  << ","
          << "Steer z"  << ","
          << "Steer roll"  << ","
          << "Steer pitch"  << ","
          << "Steer yaw" << ",";
  outFile << "Target State"  << ","
          << "Current State"  << ","
          << "Target Gait"  << ","
          << "Current Gait";        

  outFile << std::endl;

  for(int i = 0; i < size_vector; i++){
    SnapshotFrame frame = snapshot_vector_->Dequeue();
    outFile << frame.timestamp - start_time << ",";
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].joint_data.pos << ",";
    }        
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].joint_data.vel << ",";
    }        
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].joint_data.tor << ",";
    }
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].status_word << ",";
    }
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].joint_data.motor_temp << ",";
    }
    for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_data[j].joint_data.driver_temp<< ",";
    }
    for(int j = 0; j < frame.joint_command.size(); j++){
      outFile << frame.joint_command[j].joint_data.pos << ",";
    }
    for(int j = 0; j < frame.joint_command.size(); j++){
      outFile << frame.joint_command[j].joint_data.vel << ",";
    }
    for(int j = 0; j < frame.joint_command.size(); j++){
      outFile << frame.joint_command[j].joint_data.tor << ",";
    }
        for(int j = 0; j < frame.joint_data.size(); j++){
      outFile << frame.joint_command[j].control_word<< ",";
    }
    outFile << frame.Arm_imu_data.acc_x << ","
            << frame.Arm_imu_data.acc_y << ","
            << frame.Arm_imu_data.acc_z << ","
            << frame.Arm_imu_data.roll << ","
            << frame.Arm_imu_data.pitch << ","
            << frame.Arm_imu_data.yaw << ","
            << frame.Arm_imu_data.omega_x << ","
            << frame.Arm_imu_data.omega_y << ","
            << frame.Arm_imu_data.omega_z << ",";
    outFile << frame.Leg_imu_data.acc_x << ","
            << frame.Leg_imu_data.acc_y << ","
            << frame.Leg_imu_data.acc_z << ","
            << frame.Leg_imu_data.roll << ","
            << frame.Leg_imu_data.pitch << ","
            << frame.Leg_imu_data.yaw << ","
            << frame.Leg_imu_data.omega_x << ","
            << frame.Leg_imu_data.omega_y << ","
            << frame.Leg_imu_data.omega_z << ",";
    for(int j = 0; j < sizeof(frame.steer_data)/sizeof(float); j++){
      outFile << frame.steer_data.data[j] << ",";
    }
    outFile << frame.motion_state[0] << "," << frame.motion_state[1] << "," << frame.motion_gait[0] << "," << frame.motion_gait[1];
    outFile << std::endl;
  }
  outFile.close();
  is_writing_to_file_ = false;
  logger_->info("Write complete");
}


void Snapshot::initializeDDSChannels() {
  logger_ = new DrLogger("Snapshot");
  std::string model = GetRobotModel();
  std::map<std::uint8_t, XYZ> motorMap = RobotConfig().getMapForModel(model);
  memset(&snapshot_frame_.Arm_imu_data, 0, sizeof(snapshot_frame_.Arm_imu_data));
  memset(&snapshot_frame_.Leg_imu_data, 0, sizeof(snapshot_frame_.Leg_imu_data));

  memset(&snapshot_frame_.steer_data, 0, sizeof(snapshot_frame_.steer_data));
  memset(&snapshot_frame_.motion_state, 0, sizeof(snapshot_frame_.motion_state));
  memset(&snapshot_frame_.motion_gait, 0, sizeof(snapshot_frame_.motion_gait));

  snapshot_frame_.joint_command.resize(motorMap.size());
  snapshot_frame_.joint_data.resize(motorMap.size());
  
  snapshot_vector_ = new CircularQueue<SnapshotFrame>(record_time_*200);
  logger_->info("The snapshot recording time window is %d seconds ", record_time_);
  ch_steer_ = new ChannelSteer(std::bind(&Snapshot::DDSSteerDataHandler, this, std::placeholders::_1), "/STEER", 0);
  ch_motion_state_ = new ChannelMotionState(std::bind(&Snapshot::DDSMotionStateHandler, this, std::placeholders::_1), "/MOTION_STATE", 0);
  ch_gait_ = new ChannelGait(std::bind(&Snapshot::DDSMotionGait, this, std::placeholders::_1), "/GAIT", 0);

}

/*  */
void Snapshot::DDSJointCmdHandler(const drdds::msg::JointsCmd *data, uint32_t size) {
  for(int i = 0; i < size; i++){
    snapshot_frame_.joint_command[i].joint_data.tor = data->data().at(i).torque();
    snapshot_frame_.joint_command[i].joint_data.pos = data->data().at(i).position();  
    snapshot_frame_.joint_command[i].joint_data.vel = data->data().at(i).velocity();  
    snapshot_frame_.joint_command[i].joint_data.kp = data->data().at(i).kp();   
    snapshot_frame_.joint_command[i].joint_data.kd = data->data().at(i).kd(); 
    snapshot_frame_.joint_command[i].control_word = data->data().at(i).control_word();
  }
}
void Snapshot::DDSJointRawDataHandler(const drdds::msg::Joints* data, uint32_t size)
{
  for(int i = 0; i < size; i++)
  {
    snapshot_frame_.joint_data[i].joint_data.tor = data->data().at(i).torque();
    snapshot_frame_.joint_data[i].joint_data.pos = data->data().at(i).position();  
    snapshot_frame_.joint_data[i].joint_data.vel = data->data().at(i).velocity();  
    snapshot_frame_.joint_data[i].joint_data.motor_temp = data->data().at(i).motion_temp();   
    snapshot_frame_.joint_data[i].joint_data.driver_temp = data->data().at(i).driver_temp(); 
    snapshot_frame_.joint_data[i].control_word =data->data().at(i).status_word();
  }
}   
void Snapshot::DDSArmImuDataHandler(const sensor_msgs::msg::Imu *data)
{
  if (!data) return;
  // 从四元数反算 RPY（ZYX 顺序），保持与原接口一致以度为单位写入
  const double qx = data->orientation().x();
  const double qy = data->orientation().y();
  const double qz = data->orientation().z();
  const double qw = data->orientation().w();
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  const double kRad2Deg = 180.0 / M_PI;
  snapshot_frame_.Arm_imu_data.acc_x = data->linear_acceleration().x();
  snapshot_frame_.Arm_imu_data.acc_y = data->linear_acceleration().y();
  snapshot_frame_.Arm_imu_data.acc_z = data->linear_acceleration().z();
  snapshot_frame_.Arm_imu_data.roll = roll * kRad2Deg;
  snapshot_frame_.Arm_imu_data.pitch = pitch * kRad2Deg;
  snapshot_frame_.Arm_imu_data.yaw = yaw * kRad2Deg;
  snapshot_frame_.Arm_imu_data.omega_x = data->angular_velocity().x();
  snapshot_frame_.Arm_imu_data.omega_y = data->angular_velocity().y();
  snapshot_frame_.Arm_imu_data.omega_z = data->angular_velocity().z();
}
void Snapshot::DDSLegImuDataHandler(const drdds::msg::ImuData *data)
{
  snapshot_frame_.Leg_imu_data.acc_x = data->data().acc_x();
  snapshot_frame_.Leg_imu_data.acc_y = data->data().acc_y();
  snapshot_frame_.Leg_imu_data.acc_z = data->data().acc_z();
  snapshot_frame_.Leg_imu_data.roll = data->data().roll();
  snapshot_frame_.Leg_imu_data.pitch = data->data().pitch();
  snapshot_frame_.Leg_imu_data.yaw = data->data().yaw();
  snapshot_frame_.Leg_imu_data.omega_x = data->data().omega_x();
  snapshot_frame_.Leg_imu_data.omega_y = data->data().omega_y();
  snapshot_frame_.Leg_imu_data.omega_z = data->data().omega_z();
}
void Snapshot::DDSBatteryDataHandler(const BatteryInfo& battery_info, uint32_t index)
{
    if(index > 1)
    {
      return;
    }
    uint32_t i = index;
    battery_info_[i].voltage = battery_info[i].voltage;
    battery_info_[i].current = battery_info[i].current;
    battery_info_[i].remaining_capacity = battery_info[i].remaining_capacity;
    battery_info_[i].nominal_capacity = battery_info[i].nominal_capacity;
    battery_info_[i].cycles = battery_info[i].cycles;
    battery_info_[i].production_date = battery_info[i].production_date;
    battery_info_[i].balanced_low = battery_info[i].balanced_low;
    battery_info_[i].balanced_high = battery_info[i].balanced_high;
    battery_info_[i].protected_state = battery_info[i].protected_state;
    battery_info_[i].software_version = battery_info[i].software_version;
    battery_info_[i].battery_level = battery_info[i].battery_level;
    battery_info_[i].mos_state = battery_info[i].mos_state;
    battery_info_[i].battery_quantity = battery_info[i].battery_quantity;
    battery_info_[i].battery_ntc = battery_info[i].battery_ntc;
    for(int j = 0; j < sizeof(battery_info_[i].battery_temperature)/sizeof(float); j++){
      battery_info_[i].battery_temperature[j] = battery_info[i].battery_temperature[j];
    }
}
void Snapshot::DDSSteerDataHandler(const drdds::msg::Steer* data){
  snapshot_frame_.steer_data.x = data->data().x();
  snapshot_frame_.steer_data.y = data->data().y();
  snapshot_frame_.steer_data.z = data->data().z();
  snapshot_frame_.steer_data.roll = data->data().roll();
  snapshot_frame_.steer_data.pitch = data->data().pitch();
  snapshot_frame_.steer_data.yaw = data->data().yaw();
}


void Snapshot::DDSMotionSattusHandler(const drdds::msg::MotionInfo* data){
  static int32_t last_state = 0;
  if(data->data().motion_state().state() == 2 && last_state != 2){
    is_writing_to_file_ = true;
  }
  last_state = data->data().motion_state().state();
  snapshot_frame_.motion_state[1] = data->data().motion_state().state();
  snapshot_frame_.motion_gait[1] = data->data().gait_state().gait();

}


void Snapshot::DDSMotionStateHandler(const drdds::msg::MotionState* data){
  snapshot_frame_.motion_state[0] = data->data().state();
}
void Snapshot::DDSMotionGait(const drdds::msg::Gait* data){
  snapshot_frame_.motion_gait[0] = data->data().gait();
  
}

void Snapshot::Write(){
    is_writing_to_file_ = true;
}
