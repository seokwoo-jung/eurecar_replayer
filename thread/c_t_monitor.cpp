#include "c_t_monitor.h"

void C_T_MONITOR::run()
{
    while(true)
    {
        emit SIG_C_T_MONITOR_2_MAIN();
        QThread::msleep(10);
    }
    return;
}
