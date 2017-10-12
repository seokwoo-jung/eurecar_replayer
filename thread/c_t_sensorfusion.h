#ifndef C_T_SENSORFUSION_H
#define C_T_SENSORFUSION_H

#include <armadillo>

#include "thread_common_header.h"

#include "3d_view/c_3d_viewer.h"

using namespace std;

class C_T_SENSORFUSION : public QThread {
    Q_OBJECT

public:
    C_T_SENSORFUSION() {}
    ~C_T_SENSORFUSION() {}

    void UpdateCalibrationData(string _calib_data_path);

protected:
    void run();

private:
    void Calibration();

private:
    QMutex mtx_sensor_fusion;

    string m_calib_data_path_str;

    // Calibration data matrix
    cv::Mat1f  m_transform_mat;

    cv::Mat m_input_img;
    cv::Mat m_output_img;
    PointCloudT m_cloud;

public slots:
    void SLOT_MAIN_2_C_T_SENSORFUSION(cv::Mat _recv_img, PointCloudT _cloud);

signals:
    void SIG_C_T_SENSORFUSION_2_MAIN(cv::Mat, vector<cv::Point>, vector<cv::Point3f>);
};

#endif // C_T_SENSORFUSION_H
