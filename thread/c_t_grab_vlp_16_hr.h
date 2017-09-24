#ifndef C_T_GRAB_VLP_16_HR_H
#define C_T_GRAB_VLP_16_HR_H

// Qt header
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QMetaType>
#include <QStandardItemModel>
#include <QProcess>

// stl header
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <error.h>
#include <algorithm>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <boost/lexical_cast.hpp>

// lcm
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

// local
#include "3d_view/c_3d_viewer.h"
#include "lcm/c_lcm_handler.h"
#include "lcm/eurecar_lcmtypes/eurecar/velo_raw.hpp"
#include "thread/c_t_monitor_lidar.h"
#include "thread/c_t_lcmsubscr.h"

using namespace std;

class C_T_GRAB_VLP_16_HR : public QThread {
    Q_OBJECT
public:
    C_T_GRAB_VLP_16_HR(){
    }

    C_T_GRAB_VLP_16_HR(C_3D_VIEWER* _c_3d_viewer) {
        c_3d_viewer_obj = _c_3d_viewer;
        connect(c_t_monitor_lidar,SIGNAL(SIG_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR()),this,SLOT(SLOT_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR()));
//        m_lcm_obj.subscribe("VELO_RAW",&C_LCM_VELO_RAW::handleMessage,&c_lcm_velo_raw_obj_rec);
        m_lcm_obj.subscribe("VLP_16_PT",&C_LCM_VLP_16_PT::handleMessage,&c_lcm_vlp_16_pt_obj_rec);
        c_t_lcmsubscr_lidar->SetLCMObj(&m_lcm_obj);
    }
    QMutex mtx_vlp_16_hr;
    QMutex mtx_update_pointcloud;
    QMutex mtx_monitor_lidar;

    bool GetPauseStatus();

protected:
    void run();

// thread
public:
    C_T_MONITOR_LIDAR* c_t_monitor_lidar = new C_T_MONITOR_LIDAR;
    C_T_LCMSUBSCR* c_t_lcmsubscr_lidar = new C_T_LCMSUBSCR;


private:
    bool m_pause_status = true;
    bool m_init_status = false;
    bool m_initial_angle_set = false;
    bool m_rotation_finish = false;
    double m_initial_angle = 0.0;

   C_3D_VIEWER* c_3d_viewer_obj;

   UINT count = 0;
   WORD rotation = 0;
   WORD m_rotation_rec_past = 0;
   WORD m_rotation_rec_current = 0;
   double prev_deg = 0.0;
   double deg = 0.0;

   VLP_16_HR_DATA buffer[VLP_16_HR_TOTAL_PACKET_NUMBER];
   VLP_16_HR_DATA buffer_past[VLP_16_HR_TOTAL_PACKET_NUMBER];

   VLP_16_HR_DATA m_buffer_rec;
   bool GrabData(VLP_16_HR_DATA buffer_block);
   void GrabData_pt();

   lcm::LCM m_lcm_obj;
   C_LCM_VELO_RAW c_lcm_velo_raw_obj_rec;
   C_LCM_VLP_16_PT c_lcm_vlp_16_pt_obj_rec;
   eurecar::velo_raw velo_raw_data;



signals:
   void SIG_C_T_GRAB_VLP_16_HR_2_MAIN(PointCloudT);

public slots:
   void SLOT_MAIN_2_C_T_GRAB_VLP_16_HR_PAUSE(bool _pause_status);
   void SLOT_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR();


};



#endif // C_T_GRAB_VLP_16_HR_H
