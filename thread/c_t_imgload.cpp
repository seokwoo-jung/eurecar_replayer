#include "c_t_imgload.h"
void C_T_IMGLOAD::SLOT_MAIN_2_C_T_IMGLOAD(string _load_path)
{
    if(!mtx_imgload.tryLock())
        return;

    m_loadpath_img = _load_path;
    this->start();
    mtx_imgload.unlock();
}

void C_T_IMGLOAD::ImgLoad()
{
    mtx_imgload.lock();

    m_img =  cv::imread(m_loadpath_img);

    if(!m_img.empty())
        emit SIG_C_T_IMGLOAD_2_MAIN_IMG(m_img);

    mtx_imgload.unlock();
}


void C_T_IMGLOAD::run()
{
    ImgLoad();
    return;
}
