#ifndef C_MONITOR_H
#define C_MONITOR_H

#include "thread_common_header.h"

class C_T_MONITOR : public QThread {
    Q_OBJECT

public:
    C_T_MONITOR() {}
    ~C_T_MONITOR() {}

private:
    void run();

signals:
    void SIG_C_T_MONITOR_2_MAIN();
};

#endif // C_MONITOR_H

