#include "c_lcm_handler.h"

void C_LCM_CAM::handleMessage_cam_1(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[0] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_2(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[1] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_3(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[2] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_4(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[3] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_5(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[4] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_6(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[5] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_7(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[6] = msg->timestamp;
}

void C_LCM_CAM::handleMessage_cam_8(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime_cam[7] = msg->timestamp;
}


void C_LCM_CAN_T::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::can_t * msg)
{
    utime = msg->utime;
    time= msg->time;
    yaw_rate = msg->yaw_rate;
    mdps_str_ang = msg->mdps_str_ang;
    VS_CAN = msg->VS_CAN;
    lat_accel = msg->lat_accel;
    mcp = msg->mcp;
    accel_pedal_value = msg->accel_pedal_value;
    tps = msg->tps;
    odometer = msg->odometer;
    battery_voltage = msg->battery_voltage;
    WHL_SPD_RR = msg->WHL_SPD_RR;
    WHL_SPD_RL = msg->WHL_SPD_RL;
    WHL_SPD_FR = msg->WHL_SPD_FR;
    WHL_SPD_FL = msg->WHL_SPD_FL;
}

void C_LCM_VELO_RAW::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string  &chan, const eurecar::velo_raw *msg)
{
    utime = msg->utime;
    memcpy(raw,msg->raw,1206);
}

void C_LCM_VLP_16_PT::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const eurecar::vlp_16_pt *msg)
{
    utime = msg->utime;
    memcpy(x_data_arr,msg->x_data_arr,sizeof(double)*VLP_16_HR_TOTAL_PACKET_NUMBER*VLP_16_HR_BOLCKS_NUM*VLP_16_HR_LASERS_NUM);
    memcpy(y_data_arr,msg->y_data_arr,sizeof(double)*VLP_16_HR_TOTAL_PACKET_NUMBER*VLP_16_HR_BOLCKS_NUM*VLP_16_HR_LASERS_NUM);
    memcpy(z_data_arr,msg->z_data_arr,sizeof(double)*VLP_16_HR_TOTAL_PACKET_NUMBER*VLP_16_HR_BOLCKS_NUM*VLP_16_HR_LASERS_NUM);
    memcpy(intensity_data_arr,msg->intensity_data_arr,sizeof(int)*VLP_16_HR_TOTAL_PACKET_NUMBER*VLP_16_HR_BOLCKS_NUM*VLP_16_HR_LASERS_NUM);
}
