#include "c_t_grab_vlp_16_hr.h"

void C_T_GRAB_VLP_16_HR::SLOT_MAIN_2_C_T_GRAB_VLP_16_HR_PAUSE(bool _pause_status)
{
    m_pause_status = _pause_status;
    if(!m_pause_status)
        this->start();
}

void C_T_GRAB_VLP_16_HR::SLOT_C_T_MONITOR_LIDAR_2_C_T_GRAB_VLP_16_HR()
{
    if(!mtx_monitor_lidar.tryLock())
        return;

    c_t_lcmsubscr_lidar->LCMSubscribe(&m_lcm_obj);
    VLP_16_HR_DATA buffer_rec;

    memcpy(&buffer_rec,c_lcm_velo_raw_obj_rec.raw,VLP_16_HR_DATA_SIZE);

    m_buffer_rec = buffer_rec;

    m_rotation_rec_past = m_rotation_rec_current;
    m_rotation_rec_current = buffer_rec.firing_data[0].rotation;

    if(m_rotation_rec_past != m_rotation_rec_current)
    {
        this->start();
    }
    mtx_monitor_lidar.unlock();
}


void C_T_GRAB_VLP_16_HR::run()
{
    mtx_monitor_lidar.lock();
    if(!m_rotation_finish)
    {
        GrabData(m_buffer_rec);
    }
    else
    {
        memset(buffer,0,sizeof(VLP_16_HR_DATA)*VLP_16_HR_TOTAL_PACKET_NUMBER);
        m_rotation_finish = false;
        GrabData(m_buffer_rec);
    }
    mtx_monitor_lidar.unlock();
}

bool C_T_GRAB_VLP_16_HR::GrabData(VLP_16_HR_DATA buffer_block)
{

    memcpy((char *)&buffer[count],&buffer_block,VLP_16_HR_DATA_SIZE);

    rotation = (buffer[count].firing_data[0].rotation);
    prev_deg = deg;
    deg = (double)(rotation / 100.0);

    if(m_initial_angle_set == false)
    {
        m_initial_angle = deg;
        m_initial_angle_set = true;
    }

    if(prev_deg < deg)
    {
        if((m_initial_angle - prev_deg)*(m_initial_angle - deg) < 0)
        {
            m_rotation_finish = true;
            m_initial_angle = deg;
        }
    }
    else
    {
        if(m_initial_angle > prev_deg)
        {
            m_rotation_finish = true;
            m_initial_angle = deg;
        }
        else if(m_initial_angle < deg)
        {
            m_rotation_finish = true;
            m_initial_angle = deg;
        }
    }

    if(count >= VLP_16_HR_TOTAL_PACKET_NUMBER)
    {
        m_rotation_finish = true;
        m_initial_angle = deg;
    }


    if(m_rotation_finish){
        mtx_update_pointcloud.lock();

        PointCloudT cloud_velodyne;

        memcpy(c_3d_viewer_obj->m_vlp_16_hr_data_ary, buffer, sizeof(VLP_16_HR_DATA)*VLP_16_HR_TOTAL_PACKET_NUMBER);
        c_3d_viewer_obj->SetVelodyneData();

        long pt_count = 0;
        for(int k = 0;k < VLP_16_HR_LASERS_NUM;k++){
            for(int i=0; i< VLP_16_HR_TOTAL_PACKET_NUMBER;i++){
                for(int j=0; j < VLP_16_HR_BOLCKS_NUM;j++){

                    pcl::PointXYZRGBA velodyne_pt;

                    velodyne_pt.x = c_3d_viewer_obj->m_x_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)]*0.001;
                    velodyne_pt.y = c_3d_viewer_obj->m_y_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)]*0.001;
                    velodyne_pt.z = c_3d_viewer_obj->m_z_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)]*0.001;

                    double pt_dist = sqrt(velodyne_pt.x*velodyne_pt.x + velodyne_pt.y*velodyne_pt.y + velodyne_pt.z*velodyne_pt.z);

                    velodyne_pt.r = cv::saturate_cast<uint8_t>(220 - 255*(pt_dist/5.0));
                    velodyne_pt.g = cv::saturate_cast<uint8_t>(50);
                    velodyne_pt.b = cv::saturate_cast<uint8_t>(70*(1+(pt_dist/5.0)));

                    if((velodyne_pt.x == 0) && (velodyne_pt.y == 0) && (velodyne_pt.z == 0))
                    {
                        continue;
                    }
                    pt_count++;
                    cloud_velodyne.points.push_back(velodyne_pt);
                }
            }
        }
        emit SIG_C_T_GRAB_VLP_16_HR_2_MAIN(cloud_velodyne);
        count = 0;
        mtx_update_pointcloud.unlock();
        QThread::usleep(10);
        return true;
    }
    else
    {
        count ++;
    }
    return true;
}

bool C_T_GRAB_VLP_16_HR::GetPauseStatus()
{
    return m_pause_status;
}

