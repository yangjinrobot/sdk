/**
 * @file data_recorder.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <time.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
 
#include "data_recorder.h"
#include "robot_model.hpp"


using namespace ecan;
using namespace std;

std::string battery_csv_name[] = {
      "Voltage V",
      "Current A",
      "Percentage %",
      "Cycles",
      "Protected State",
      "Mos State",
      "Battery Temperature_1 ",
      "Battery Temperature_2 ",
      "Battery Temperature_3 ",
      "Battery Temperature_4 "
    };
std::string cpu_csv_name[2] = {
      "Cpu Temperature ",
      "Cpu Frequency "
};
DataRecorder::DataRecorder() {

}

DataRecorder::DataRecorder(int thread_priority):BaseActor(thread_priority) {
  std::string model = GetRobotModel();
  std::map<std::uint8_t, XYZ> motorMap = RobotConfig().getMapForModel(model);
  DataMapInit(motorMap);
}

DataRecorder::~DataRecorder(){

}
int DataRecorder::DataMapInit(std::map<uint8_t, XYZ> InitMap){
  int i = 0;

  for (const auto& pair : InitMap) {
      int key = pair.first;
      if(key >= kJointCount){
        std::cout << "Key is not defined. Please check the enum for JointID." << std::endl;
        break;
      }
      XYZ value = pair.second;
      raw_data_index_[raw_joint_num_++] = key;
    }
  return 0;
}

void DataRecorder::WriteHeader(void){
  outFile << "Time s" << "," ;
  for(string it : battery_csv_name){
    outFile << "Battery L " << it << ",";		// 解引用打印对应的元素
  }
  for(string it : battery_csv_name){
    outFile << "Battery R " << it << ",";		// 解引用打印对应的元素
  }  
  // for(string it : cpu_csv_name){
  //   outFile << it << ",";		// 解引用打印对应的元素
  // }
  if(isFirefly3588()){
    for(int i = 0; i < 3; i++){
      outFile << cpu_csv_name[0] << i << ",";
    }
    for(int i = 0; i < 4; i++){
      outFile << cpu_csv_name[1] << i <<" MHz "<< ",";
    }  
  }

  for(int i = 0; i < raw_joint_num_; i++){
    outFile << "Joint " << raw_data_index_[i] << " Motor Temperature ,";
  }
  for(int i = 0; i < raw_joint_num_; i++){
    outFile << "Joint " << raw_data_index_[i] << " Driver Temperature ,";
  }
  outFile << std::endl;
}

bool DataRecorder::isFirefly3588(void) {
    std::ifstream model_file("/proc/device-tree/model");
    std::string model;
    if (model_file) {
        std::getline(model_file, model);
        is_firefly_3588_ = model.find("3588") != std::string::npos;
        return is_firefly_3588_;
    }
    is_firefly_3588_ = false;
    return false;
}
std::vector<int> DataRecorder::getCPUFrequencies() {
    std::vector<int> cpu_frequencies;
    for (int i = 4; i <= 7; i++) {
        std::ifstream freq_file("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq");
        if (freq_file) {
            int frequency;
            freq_file >> frequency;
            cpu_frequencies.push_back(frequency);
        } else {
            cpu_frequencies.push_back(0);  // 如果无法读取，使用0表示
        }
    }
    return cpu_frequencies;
}

std::vector<int> DataRecorder::getTemperatures() {
  std::vector<int> temperatures;
  for (int i = 0; i <= 2; i++) {
    std::ifstream temp_file("/sys/class/thermal/thermal_zone" + std::to_string(i) + "/temp");
    if (temp_file) {
      int temp;
      temp_file >> temp;
      temperatures.push_back(temp / 1000);  // 温度单位是毫度，转换为摄氏度
    } else {
      temperatures.push_back(0);  // 如果无法读取，使用0表示
    }
  }

  return temperatures;
}
void DataRecorder::WriteBattery(void){
  
  outFile << (float)(*battery_info_)[0].voltage*0.01 << "," << (float)(*battery_info_)[0].current*0.01 << "," << (int)(*battery_info_)[0].battery_level << ",";
  outFile << (int)(*battery_info_)[0].cycles << "," << (int)(*battery_info_)[0].protected_state << "," << (int)(*battery_info_)[0].mos_state << ",";
  for(float it : (*battery_info_)[0].battery_temperature){
    outFile << it << ",";
  }
  outFile << (float)(*battery_info_)[1].voltage*0.01 << "," << (float)(*battery_info_)[1].current*0.01 << "," << (int)(*battery_info_)[1].battery_level << ",";
  outFile << (*battery_info_)[1].cycles << "," << (*battery_info_)[1].protected_state << "," << (int)(*battery_info_)[1].mos_state << ",";
  for(float it : (*battery_info_)[1].battery_temperature){
    outFile << it << ",";
  }
}

void DataRecorder::WriteMotorTemperature(void){
  for(int i = 0; i < raw_joint_num_; i++)
  {
    outFile << (float)(*raw_data_vector_)[i].joint_data.motor_temp << ",";
  }
}
void DataRecorder::WriteDriverTemperature(void){
 
  for(int i = 0; i < raw_joint_num_; i++){
    outFile << (float)(*raw_data_vector_)[i].joint_data.driver_temp << ",";
  }
}

void DataRecorder::WriteCPUInfo(void){
  if(!is_firefly_3588_){
    return;
  }
  std::vector<int> cpu_frequencies = getCPUFrequencies();
  std::vector<int> temperatures = getTemperatures();
  uint32_t cpu_temperature_error = 0;
  int cpu_temperature_max = 0;
  bool cpu_frequencies_error = false;

  for (size_t i = 0; i < temperatures.size(); ++i) 
  {
    outFile  << temperatures[i] << ",";
  }
  for (size_t i = 0; i < cpu_frequencies.size(); ++i) {
    outFile << cpu_frequencies[i] / 1000 << ",";  // 转换为 MHz
  }

}

void DataRecorder::WriteTime(void){
    outFile <<  GetTime() << ",";
}
string DataRecorder::GetTime(void){
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

void* DataRecorder::RunRoutine(void*){

  int tfd;    //定时器描述符
  int efd;    //epoll描述符
  int fds, ret;
  uint64_t value;
  struct epoll_event ev, *evptr;
  struct itimerspec time_intv; //用来存储时间

  tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
  if(tfd == -1) {
  }

  time_intv.it_value.tv_sec = 2; //设定2s超时
  time_intv.it_value.tv_nsec = 0;
  time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
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
  outFile.open("/var/opt/robot/log/system/"+GetTime()+".csv", std::ios::out);

  pthread_barrier_wait(&barrier_);
  WriteHeader();
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
    WriteTime();
    WriteBattery();
    WriteCPUInfo();
    WriteMotorTemperature();
    WriteDriverTemperature();
    outFile << std::endl;
  }
}
