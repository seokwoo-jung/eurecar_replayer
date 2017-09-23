#include "c_3d_viewer.h"

void C_3D_VIEWER::init()
{
    cloud.reset(new PointCloudT);
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer",false));
}

void C_3D_VIEWER::SetVelodyneData()
{
    for (int k = 0; k < 16; k++){
        for (int i = 0; i < VLP_16_HR_TOTAL_PACKET_NUMBER; i++){
            for (int j = 0; j < VLP_16_HR_BOLCKS_NUM; j++){

                double z_projected = 0.0;
                double xy_deg = 0.0;

                unsigned int distance = (m_vlp_16_hr_data_ary[i].firing_data[j].laser_data[k].distance) * 2;//mm
                unsigned short rotation = (m_vlp_16_hr_data_ary[i].firing_data[j].rotation);
                if(distance != 0)
                    xy_deg = rotation / 100.0;
                z_projected = distance * cos(D2R*(vlp_16_hr_firing_vertical_angle[k]));

                m_x_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)] =
                    z_projected*sin(D2R*xy_deg);

                m_y_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)] =
                    z_projected*cos(D2R*xy_deg);

                m_z_data_arr[k][(j + i*VLP_16_HR_BOLCKS_NUM)] =
                    distance * sin(D2R*(vlp_16_hr_firing_vertical_angle[k]));
            }
        }
    }
}
