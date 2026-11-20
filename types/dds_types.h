#ifndef DDS_TYPES_H_
#define DDS_TYPES_H_
#include "common_types.h"

namespace types{
    typedef struct{      // BatterySensorData  BatteryBaseInfoTag
        uint16_t voltage;             ///<总电压            单位10mV
        int16_t  current;             ///<总电流            单位10mA
        uint16_t remaining_capacity;  ///<剩余容量          单位10mAH
        uint16_t nominal_capacity;    ///<标称容量          单位10mAH
        uint16_t cycles;              ///<循环次数
        uint16_t production_date;     ///<生产日期
        uint16_t balanced_low;        ///<均衡低
        uint16_t balanced_high;       ///<均衡高
        uint16_t protected_state;     ///<保护状态

        uint8_t software_version;  ///<软件版本
        uint8_t battery_level;     ///<剩余容量百分比
        uint8_t mos_state;         ///<Mos状态
        uint8_t battery_quantity;  ///<电池串数
        uint8_t battery_ntc;       ///<NTC个数

        float battery_temperature[4];
    }BatteryInfo_dds;

    typedef struct {
        float pos, vel, tor, kp, kd;
        float motor_temperature, driver_temperature;
        uint16_t status_word, control_word, data_id;  
    } JointData_dds;

    struct ImuData_dds{
        float roll, pitch, yaw;
        float omega_x, omega_y, omega_z;
        float acc_x, acc_y, acc_z;
        uint32_t timestamp;
    };

}
#endif