#include "c_t_sensorfusion.h"

void C_T_SENSORFUSION::UpdateCalibrationData(string _calib_data_path)
{
    m_calib_data_path_str = _calib_data_path;

    cv::FileStorage fs(m_calib_data_path_str,cv::FileStorage::READ);
    fs["transform_mat"] >> m_transform_mat;
    fs.release();
}

void C_T_SENSORFUSION::SLOT_MAIN_2_C_T_SENSORFUSION(cv::Mat _recv_img, PointCloudT _cloud)
{
    if(!mtx_sensor_fusion.tryLock())
        return;
    _recv_img.copyTo(m_input_img);
    m_cloud = _cloud;
    this->start();
    mtx_sensor_fusion.unlock();
}

void C_T_SENSORFUSION::run()
{
    Calibration();
    QThread::msleep(10);
    return;
}

void C_T_SENSORFUSION::Calibration()
{
    mtx_sensor_fusion.lock();
    cv::Size size_load_to_real;
    size_load_to_real.width = 1920;
    size_load_to_real.height = 1208;

    cv::Mat real_img;
    cv::resize(m_input_img,real_img,size_load_to_real);

    arma::mat M(3,4);

    for(int i = 0; i < 3;i++)
    {
        for(int j = 0; j < 4;j++)
        {
            M(i,j) = m_transform_mat(i,j);
        }
    }


    uint u_past = UINT_MAX;
    uint v_past = UINT_MAX;

    vector<cv::Point> img_point_list;
    vector<cv::Point3f> lidar_point_list;

    for(int i= (int)m_cloud.points.size()-1;i >= 0;i--)
    {


        float x = m_cloud.points[i].x;
        float y = m_cloud.points[i].y;
        float z = m_cloud.points[i].z;

        float angle = atan2(x,y);

        if(!((angle > -3.141592/2.0) && (angle < 3.141592/2.0)))
            continue;

        uint u = (M(0,0)*x + M(0,1)*y + M(0,2)*z + M(0,3))/(M(2,0)*x + M(2,1)*y + M(2,2)*z + M(2,3));
        uint v = (M(1,0)*x + M(1,1)*y + M(1,2)*z + M(1,3))/(M(2,0)*x + M(2,1)*y + M(2,2)*z + M(2,3));


        float dist = sqrt(x*x+y*y+z*z);

        if(dist < 1.0)
        {
            continue;
        }
        float color_scale = cv::saturate_cast<uchar>(dist*10);
        float color_scale2 = cv::saturate_cast<uchar>(200 - dist*10);
        float color_scale3 = cv::saturate_cast<uchar>(dist*5);
        if((u >= 0) && (u < real_img.cols) && (v >= 0) && (v < real_img.rows))
        {
//            if( (u < u_past) || (abs((int)v_past- (int)v) > 10))
//            {
                cv::circle(real_img,cv::Point(u,v),2,cv::Scalar(color_scale3,color_scale2,color_scale),2);
                u_past = u;

                cv::Point img_point;
                img_point.x = u;
                img_point.y = v;

                cv::Point3f lidar_point;
                lidar_point.x = x;
                lidar_point.y = y;
                lidar_point.z = z;

                img_point_list.push_back(img_point);
                lidar_point_list.push_back(lidar_point);
//            }

            v_past = v;
        }
    }

    cv::resize(real_img,m_output_img,cv::Size(960,604));

    emit SIG_C_T_SENSORFUSION_2_MAIN(m_output_img,img_point_list,lidar_point_list);
    mtx_sensor_fusion.unlock();
}
