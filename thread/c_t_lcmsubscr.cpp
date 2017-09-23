#include "c_t_lcmsubscr.h"

void C_T_LCMSUBSCR::LCMSubscribe(lcm::LCM *_lcm_obj)
{
    if(!mtx_subc.tryLock())
        return;
    m_lcm_obj = _lcm_obj;
    this->start();
    mtx_subc.unlock();
}

void C_T_LCMSUBSCR::LCMHandle()
{
    m_lcm_obj->handleTimeout(0);
}

void C_T_LCMSUBSCR::run()
{
    LCMHandle();
    return;
}


