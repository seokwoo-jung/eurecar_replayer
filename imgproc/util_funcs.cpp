#include "util_funcs.h"

QImage Mat2QImage(cv::Mat src)
{
    cv::Mat temp;
    cv::cvtColor(src,temp,CV_BGR2RGB);
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

byte* Mat2Bytes(cv::Mat src)
{
    int size = src.total() * src.elemSize();
    byte* bytes = new byte[size];
    std::memcpy(bytes,src.data,size * sizeof(byte));
    return bytes;
}

cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size)
{
    cv::Mat output_img;
    double width_ratio = (double)_ori_img.cols/(double)_output_size.width;
    double height_ratio = (double)_ori_img.rows/(double)_output_size.height;

    if(width_ratio > height_ratio) // resize by width ratio
    {
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/width_ratio,(1.0)/width_ratio);
    }
    else
    {
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/height_ratio,(1.0)/height_ratio);
    }

    return output_img;
}

vector<double> Img2Real_Coordinate_Convert(vector<int> _img_coord)
{
    double real_x = 0;
    double real_y = 0;

    real_x = ((double)_img_coord.at(0) - IMG_CENTER_X)*0.04;
    real_y = -((double)_img_coord.at(1) - IMG_CENTER_Y)*0.04;

    vector<double> return_vec;
    return_vec.push_back(real_x);
    return_vec.push_back(real_y);

    return return_vec;
}

vector<int> Real2Img_Coordinate_Convert(vector<double> _real_coord)
{
    int img_x = 0;
    int img_y = 0;

    img_x = (int)((_real_coord.at(0)/0.04) + (double)IMG_CENTER_X);
    img_y = (int)((-_real_coord.at(1)/0.04) + (double)IMG_CENTER_Y);

    if((img_x < 0) || (img_x > 760) || (img_y < 0) || (img_y > 990))
    {
        img_x = -1;
        img_y = -1;
    }

    vector<int> return_vec;
    return_vec.push_back(img_x);
    return_vec.push_back(img_y);

    return return_vec;
}
