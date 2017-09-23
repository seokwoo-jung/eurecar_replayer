#include "c_t_monitor_lidar.h"

void C_T_MONITOR_LIDAR::run()
{
    while(true)
    {
        emit SIG_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR();
        QThread::usleep(1);
    }
    return;
}
