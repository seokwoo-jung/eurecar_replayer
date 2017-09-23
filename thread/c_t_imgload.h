#ifndef C_IMGLOAD_H
#define C_IMGLOAD_H

#include "thread_common_header.h"

using namespace std;

class C_T_IMGLOAD : public QThread {
    Q_OBJECT
public:
    C_T_IMGLOAD() {}
    ~C_T_IMGLOAD() {}

private:
    void run();
    void ImgLoad();
    QMutex mtx_imgload;

private:
    cv::Mat m_img;
    string m_loadpath_img;

    bool m_pause_status = false;

public slots:
    void SLOT_MAIN_2_C_T_IMGLOAD(string _load_path);

signals:
    void SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat);
};
#endif // C_IMGLOAD_H
