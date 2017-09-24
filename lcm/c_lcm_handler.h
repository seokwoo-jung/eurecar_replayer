#ifndef CLCM_HANDLER_H
#define CLCM_HANDLER_H

#include "eurecar_lcmtypes/eurecar/cam.hpp"
#include "eurecar_lcmtypes/eurecar/can_t.hpp"
#include "eurecar_lcmtypes/eurecar/velo_raw.hpp"
#include "eurecar_lcmtypes/eurecar/vlp_16_pt.hpp"
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "sensor/c_vlp_16_hr.h"

class C_LCM_CAM
{
public:
    ~C_LCM_CAM () {}

    int64_t utime_cam[8];

    void handleMessage_cam_1(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_2(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_3(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_4(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_5(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_6(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_7(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
    void handleMessage_cam_8(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);

};

class C_LCM_CAN_T
{
public:
    ~C_LCM_CAN_T () {}

    int64_t    utime;
    double     time;
    double     yaw_rate;
    double     mdps_torque;
    double     mdps_str_ang;
    double     VS_CAN;
    double     lat_accel;
    double     mcp;
    double     accel_pedal_value;
    double     tps;
    double     odometer;
    double     battery_voltage;
    double     WHL_SPD_RR;
    double     WHL_SPD_RL;
    double     WHL_SPD_FR;
    double     WHL_SPD_FL;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::can_t * msg);
};


class C_LCM_VELO_RAW
{
public:
    ~C_LCM_VELO_RAW () {}

    int64_t utime = 0;
    uint8_t raw[1206];

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::velo_raw * msg);

};

class C_LCM_VLP_16_PT
{
public:
    ~C_LCM_VLP_16_PT () {}

    int64_t    utime = 0;

    double     x_data_arr[32][2220] = {{0}};

    double     y_data_arr[32][2220] = {{0}};

    double     z_data_arr[32][2220] = {{0}};

    int32_t    intensity_data_arr[32][2220] = {{0}};

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::vlp_16_pt * msg);

};



#endif // CLCM_HANDLER_H
