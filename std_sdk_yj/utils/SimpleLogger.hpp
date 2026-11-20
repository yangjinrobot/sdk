//
// Created by willw on 23-7-31.
//

#ifndef SIMPLELOGGER_H
#define SIMPLELOGGER_H

#include <fstream>
#include <iostream>
#include <cmath>
#include <memory>
#include <list>

class SimpleLogger{

public:
  SimpleLogger(const std::string& fileName, const size_t len):
          numLogCols(len){

    char _dateStr[100] = "";
    time_t curr_time;
    tm *curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    strftime(_dateStr, 100, "_%y%m%d_%H%M%S.csv", curr_tm);
    _logFileName = fileName + _dateStr;

    isLogOccupied = (bool*)malloc(numLogCols*sizeof(bool));
    logDataCol = (float*)malloc(numLogCols*sizeof(float));
    memset(isLogOccupied,false,numLogCols * sizeof(bool));
    memset(logDataCol,0,numLogCols * sizeof(float));
  }

  void addNewLine(){
    if (_log_buffer.size()>=numLogCapacity) _log_buffer.pop_front();
    auto newData = (float*)malloc(numLogCols*sizeof(float));
    memset(newData,0,numLogCols * sizeof(float));
    _log_buffer.push_back(newData);
  }

  void record(float* x_ptr, size_t mountPoint, size_t length){
    if (mountPoint >= numLogCols) return;
    if (mountPoint + length > numLogCols) length = numLogCols - mountPoint;
    memcpy(_log_buffer.back()+mountPoint,x_ptr,length * sizeof(float));
    if (_maxRecords < mountPoint + length)_maxRecords = mountPoint + length;
  }

  void record(int* x_ptr, size_t mountPoint, size_t length) {
    if (mountPoint >= numLogCols) return;
    if (mountPoint + length > numLogCols) length = numLogCols - mountPoint;
    for (size_t i = 0; i < length; ++i) {
        _log_buffer.back()[mountPoint + i] = static_cast<float>(x_ptr[i]);
    }
    if (_maxRecords < mountPoint + length) {
        _maxRecords = mountPoint + length;
    }
  }

  void record(const double* x_ptr, size_t mountPoint, size_t length){
    if (mountPoint >= numLogCols) return;
    if (mountPoint + length > numLogCols) length = numLogCols - mountPoint;
    for (int i=0;i<length;i++) _log_buffer.back()[mountPoint+i] = float(x_ptr[i]);
    if (_maxRecords < mountPoint + length)_maxRecords = mountPoint + length;
  }

  void write(){
      printf("[Logger.write] Preparing to write to %s.\n", _logFileName.c_str());
    if (_maxRecords == 0) {
      printf("[Logger.write] _maxRecords is zero, no data to write.\n");
      return;
    }
    FILE* _fp = fopen(_logFileName.c_str(),"w+");
    if (_fp== nullptr){
      int errNum = errno;
      printf("[Logger.write] File %s open failed\n", _logFileName.c_str());
      printf("[Logger.write] Err code: %d, reason: %s\n", errNum, strerror(errNum));
      return;
    }
    for (auto line: _log_buffer){
      for (int i=0;i<_maxRecords;i++){
        if (abs(line[i])<1e-8) fprintf(_fp, "0,");
        else fprintf(_fp, "%.5f,",line[i]);
      }
      fprintf(_fp, "0\n");
    }
    printf("[Logger.write] File %s written.\n", _logFileName.c_str());
    fclose(_fp);
  }

private:
  size_t _maxRecords=0;
  std::string _logFileName;
  float* logDataCol;
  bool* isLogOccupied;

  FILE* _fp{};
  std::list<float*> _log_buffer{};
  static const size_t numLogCapacity{16384};
  size_t numLogCols{120};

};

#endif //SIMPLELOGGER_H
