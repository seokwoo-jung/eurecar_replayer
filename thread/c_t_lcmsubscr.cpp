#include "c_t_lcmsubscr.h"
void C_T_LCMSUBSCR::SetLCMObj(lcm::LCM *_lcm_obj)
{
    m_lcm_obj = _lcm_obj;
}

void C_T_LCMSUBSCR::LCMSubscribe()
{
    if(!mtx_subc.tryLock())
        return;

    this->start();
    mtx_subc.unlock();
}

void C_T_LCMSUBSCR::LCMHandle()
{
    m_lcm_obj->handleTimeout(1);
}

void C_T_LCMSUBSCR::run()
{
    LCMHandle();
    return;
}


