#ifndef C_T_MONITOR_LIDAR_H
#define C_T_MONITOR_LIDAR_H

#include "thread_common_header.h"

class C_T_MONITOR_LIDAR : public QThread {
    Q_OBJECT

public:
    C_T_MONITOR_LIDAR() {}
    ~C_T_MONITOR_LIDAR() {}

private:
    void run();

signals:
    void SIG_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR();
};

#endif // C_T_MONITOR_LIDAR_H
