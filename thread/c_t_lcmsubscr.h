#ifndef C_LCMSUBSCR_H
#define C_LCMSUBSCR_H

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "thread_common_header.h"

class C_T_LCMSUBSCR : public QThread {
    Q_OBJECT

public:
    C_T_LCMSUBSCR() {}
    ~C_T_LCMSUBSCR() {}

    lcm::LCM *m_lcm_obj;
    void LCMSubscribe(lcm::LCM *_lcm_obj);
    void LCMHandle();


private:
    void run();
    QMutex mtx_subc;
};

#endif // C_LCMSUBSCR_H

