#ifndef CUTIL_FUNCS_H
#define CUTIL_FUNCS_H

#include <vector>

// Qt header
#include <QMainWindow>
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QPixmap>
#include <QGraphicsScene>

// opencv header
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#undef Bool

typedef uint8_t byte;

struct cv_line_element{
    cv::Point start_pt;
    cv::Point end_pt;
};

struct vehicle_detection_info_img{
    int mean_x;
    int mean_y;
    int var_x;
    int var_y;
};

using namespace std;

#define IMG_CENTER_X 379.0
#define IMG_CENTER_Y 409.0

QImage Mat2QImage(cv::Mat src);

byte* Mat2Bytes(cv::Mat src);

cv::Mat Bytes2Mat(byte* bytes, int width, int height);

cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size);

vector<double> Img2Real_Coordinate_Convert(vector<int> _img_coord);
vector<int> Real2Img_Coordinate_Convert(vector<double> _real_coord);

#endif
