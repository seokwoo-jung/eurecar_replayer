#include "g_main_window.h"
#include "ui_g_main_window.h"

G_MAIN_WINDOW::G_MAIN_WINDOW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::G_MAIN_WINDOW)
{
    VTK_MODULE_INIT(vtkRenderingOpenGL2);

    // Register metatype
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<PointCloudT>("PointCloudT");
    qRegisterMetaType<vector<cv::Point>>("vector<cv::Point>");
    qRegisterMetaType<vector<cv::Point3f>>("vector<cv::Point3f>");

    ui->setupUi(this);

    c_t_grab_vlp_16_hr = new C_T_GRAB_VLP_16_HR(c_3d_viewer_obj);

    m_graphicsview_list[0] = ui->graphicsView_cam_1;
    m_graphicsview_list[1] = ui->graphicsView_cam_2;
    m_graphicsview_list[2] = ui->graphicsView_cam_3;
    m_graphicsview_list[3] = ui->graphicsView_cam_4;
    m_graphicsview_list[4] = ui->graphicsView_cam_5;
    m_graphicsview_list[5] = ui->graphicsView_cam_6;
    m_graphicsview_list[6] = ui->graphicsView_cam_7;
    m_graphicsview_list[7] = ui->graphicsView_cam_8;

    m_disp_size.width = ui->graphicsView_cam_1->geometry().width();
    m_disp_size.height = ui->graphicsView_cam_1->geometry().height();

    connect(c_t_monitor,SIGNAL(SIG_C_T_MONITOR_2_MAIN()),this,SLOT(SLOT_C_T_MONITOR_2_MAIN()));
    connect(c_t_grab_vlp_16_hr,SIGNAL(SIG_C_T_GRAB_VLP_16_HR_2_MAIN(PointCloudT,PointCloudT)),this,SLOT(SLOT_C_T_GRAB_VLP_16_HR_2_MAIN(PointCloudT,PointCloudT)));
    connect(this,SIGNAL(SIG_MAIN_2_C_T_GRAB_VLP_16_HR_PAUSE(bool)),c_t_grab_vlp_16_hr,SLOT(SLOT_MAIN_2_C_T_GRAB_VLP_16_HR_PAUSE(bool)));
    connect(this,SIGNAL(SIG_MAIN_2_C_T_SENSORFUSION(cv::Mat,PointCloudT)),c_t_sensorfusion,SLOT(SLOT_MAIN_2_C_T_SENSORFUSION(cv::Mat,PointCloudT)));
    connect(c_t_sensorfusion,SIGNAL(SIG_C_T_SENSORFUSION_2_MAIN(cv::Mat,vector<cv::Point>,vector<cv::Point3f>)),this,SLOT(SLOT_C_T_SENSORFUSION_2_MAIN(cv::Mat,vector<cv::Point>,vector<cv::Point3f>)));

    if(ui->checkBox_sensor_fusion->isChecked())
    {
        string calib_data_path = "/home/jung/git/eurecar_replayer/sensor/fusion_data/cam3_w_vlp_16_hr.xml";
        c_t_sensorfusion->UpdateCalibrationData(calib_data_path);
    }

    // Seting QVTK widget and cloud
    m_cloud_disp.reset(new PointCloudT);
    ui->qvtkWidget_lidar->SetRenderWindow(c_3d_viewer_obj->viewer->getRenderWindow());
    c_3d_viewer_obj->viewer->setupInteractor(ui->qvtkWidget_lidar->GetInteractor(),ui->qvtkWidget_lidar->GetRenderWindow());
    c_3d_viewer_obj->viewer->setBackgroundColor(0,0,0);
    c_3d_viewer_obj->viewer->addCoordinateSystem(1.0);
//    c_3d_viewer_obj->viewer->addPointCloud(c_3d_viewer_obj->cloud,"cloud");
    c_3d_viewer_obj->viewer->addPointCloud(m_cloud_disp,"cloud_disp");
    c_3d_viewer_obj->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_disp");


    // Import veloster 3ds model
    vtkSmartPointer<vtk3DSImporter> veloster_importer(vtkSmartPointer<vtk3DSImporter>::New());
    veloster_importer->SetRenderWindow(c_3d_viewer_obj->viewer->getRenderWindow());
    veloster_importer->SetFileName("/home/jung/git/eurecar_replayer/3d_view/model/veloster.3ds");
    veloster_importer->Read();
    c_3d_viewer_obj->viewer->getRenderWindow()->AddRenderer(veloster_importer->GetRenderer());

    vtkSmartPointer<vtkLight> light (vtkSmartPointer<vtkLight>::New());
    light->SetLightTypeToHeadlight();
    light->SetColor(1, 1, 1);
    light->SetDiffuseColor(1.0, 1.0, 1.0);
    light->SetIntensity(10);
    light->SetAmbientColor(2.0, 2.0, 2.0);
    veloster_importer->GetRenderer()->AddLight(light);
}

G_MAIN_WINDOW::~G_MAIN_WINDOW()
{
    delete ui;
}

// SLOTS ********************************************************************************************************************************

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_1(cv::Mat _recv_img)
{
    mtx_grab_cam[0].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[0]);
    m_disp_img[0] = ResizeByConst(m_ori_img[0],m_disp_size);
    m_disp_img_q[0] = Mat2QImage(m_disp_img[0]);
    m_disp_img_p[0].convertFromImage(m_disp_img_q[0]);
    m_disp_img_scene[0].clear();
    m_disp_img_scene[0].addPixmap(m_disp_img_p[0]);
    m_graphicsview_list[0]->setScene(&m_disp_img_scene[0]);
    m_graphicsview_list[0]->show();
    mtx_grab_cam[0].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_2(cv::Mat _recv_img)
{
    mtx_grab_cam[1].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[1]);
    m_disp_img[1] = ResizeByConst(m_ori_img[1],m_disp_size);
    m_disp_img_q[1] = Mat2QImage(m_disp_img[1]);
    m_disp_img_p[1].convertFromImage(m_disp_img_q[1]);
    m_disp_img_scene[1].clear();
    m_disp_img_scene[1].addPixmap(m_disp_img_p[1]);
    m_graphicsview_list[1]->setScene(&m_disp_img_scene[1]);
    m_graphicsview_list[1]->show();
    mtx_grab_cam[1].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_3(cv::Mat _recv_img)
{
    mtx_grab_cam[2].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[2]);
    m_disp_img[2] = ResizeByConst(m_ori_img[2],m_disp_size);
    m_disp_img_q[2] = Mat2QImage(m_disp_img[2]);
    m_disp_img_p[2].convertFromImage(m_disp_img_q[2]);
    m_disp_img_scene[2].clear();
    m_disp_img_scene[2].addPixmap(m_disp_img_p[2]);
    m_graphicsview_list[2]->setScene(&m_disp_img_scene[2]);
    m_graphicsview_list[2]->show();

    mtx_grab_cam[2].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_4(cv::Mat _recv_img)
{
    mtx_grab_cam[3].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[3]);
    m_disp_img[3] = ResizeByConst(m_ori_img[3],m_disp_size);
    m_disp_img_q[3] = Mat2QImage(m_disp_img[3]);
    m_disp_img_p[3].convertFromImage(m_disp_img_q[3]);
    m_disp_img_scene[3].clear();
    m_disp_img_scene[3].addPixmap(m_disp_img_p[3]);
    m_graphicsview_list[3]->setScene(&m_disp_img_scene[3]);
    m_graphicsview_list[3]->show();
    mtx_grab_cam[3].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_5(cv::Mat _recv_img)
{
    mtx_grab_cam[4].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[4]);
    m_disp_img[4] = ResizeByConst(m_ori_img[4],m_disp_size);
    m_disp_img_q[4] = Mat2QImage(m_disp_img[4]);
    m_disp_img_p[4].convertFromImage(m_disp_img_q[4]);
    m_disp_img_scene[4].clear();
    m_disp_img_scene[4].addPixmap(m_disp_img_p[4]);
    m_graphicsview_list[4]->setScene(&m_disp_img_scene[4]);
    m_graphicsview_list[4]->show();
    mtx_grab_cam[4].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_6(cv::Mat _recv_img)
{
    mtx_grab_cam[5].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[5]);
    m_disp_img[5] = ResizeByConst(m_ori_img[5],m_disp_size);
    m_disp_img_q[5] = Mat2QImage(m_disp_img[5]);
    m_disp_img_p[5].convertFromImage(m_disp_img_q[5]);
    m_disp_img_scene[5].clear();
    m_disp_img_scene[5].addPixmap(m_disp_img_p[5]);
    m_graphicsview_list[5]->setScene(&m_disp_img_scene[5]);
    m_graphicsview_list[5]->show();
    mtx_grab_cam[5].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_7(cv::Mat _recv_img)
{
    mtx_grab_cam[6].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[6]);
    m_disp_img[6] = ResizeByConst(m_ori_img[6],m_disp_size);
    m_disp_img_q[6] = Mat2QImage(m_disp_img[6]);
    m_disp_img_p[6].convertFromImage(m_disp_img_q[6]);
    m_disp_img_scene[6].clear();
    m_disp_img_scene[6].addPixmap(m_disp_img_p[6]);
    m_graphicsview_list[6]->setScene(&m_disp_img_scene[6]);
    m_graphicsview_list[6]->show();
    mtx_grab_cam[6].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_IMGLOAD_2_MAIN_8(cv::Mat _recv_img)
{
    mtx_grab_cam[7].lock();
    if(_recv_img.empty())
        return;
    _recv_img.copyTo(m_ori_img[7]);
    m_disp_img[7] = ResizeByConst(m_ori_img[7],m_disp_size);
    m_disp_img_q[7] = Mat2QImage(m_disp_img[7]);
    m_disp_img_p[7].convertFromImage(m_disp_img_q[7]);
    m_disp_img_scene[7].clear();
    m_disp_img_scene[7].addPixmap(m_disp_img_p[7]);
    m_graphicsview_list[7]->setScene(&m_disp_img_scene[7]);
    m_graphicsview_list[7]->show();
    mtx_grab_cam[7].unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_MONITOR_2_MAIN()
{
    if(!mtx_monitor.tryLock())
        return;

    c_t_lcmsubscr_cam->LCMSubscribe();

    m_timestamp = m_lcm_cam_obj_recv.utime_cam[0];

    for(uint cam_ind = 0;cam_ind < m_cam_num;cam_ind++)
    {
        string img_file_path = m_img_loadpath_str + "/CAM_" + std::to_string(cam_ind + 1) + "/" + std::to_string(m_lcm_cam_obj_recv.utime_cam[cam_ind]) + ".jpg";
        switch(cam_ind)
        {
        case 0:
            emit SIG_MAIN_2_C_T_IMGLOAD_1(img_file_path);
            break;
        case 1:
            emit SIG_MAIN_2_C_T_IMGLOAD_2(img_file_path);
            break;
        case 2:
            emit SIG_MAIN_2_C_T_IMGLOAD_3(img_file_path);
            break;
        case 3:
            emit SIG_MAIN_2_C_T_IMGLOAD_4(img_file_path);
            break;
        case 4:
            emit SIG_MAIN_2_C_T_IMGLOAD_5(img_file_path);
            break;
        case 5:
            emit SIG_MAIN_2_C_T_IMGLOAD_6(img_file_path);
            break;
        case 6:
            emit SIG_MAIN_2_C_T_IMGLOAD_7(img_file_path);
            break;
        case 7:
            emit SIG_MAIN_2_C_T_IMGLOAD_8(img_file_path);
            break;
        }
    }

    mtx_monitor.unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_GRAB_VLP_16_HR_2_MAIN(PointCloudT _cloud, PointCloudT _cloud_disp)
{
    mtx_grab_lidar.lock();
    c_t_grab_vlp_16_hr->mtx_update_pointcloud.lock();
    *(c_3d_viewer_obj->cloud) = _cloud;
    m_cloud_for_sensor_fusion = *(c_3d_viewer_obj->cloud);
    *m_cloud_disp = _cloud_disp;
    c_3d_viewer_obj->viewer->updatePointCloud(m_cloud_disp,"cloud_disp");
    ui->qvtkWidget_lidar->update();

    if(ui->checkBox_sensor_fusion->isChecked())
    {
        if(!m_ori_img[2].empty())
        {
            emit SIG_MAIN_2_C_T_SENSORFUSION(m_ori_img[2],m_cloud_for_sensor_fusion);
        }
    }

    c_t_grab_vlp_16_hr->mtx_update_pointcloud.unlock();
    mtx_grab_lidar.unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_SENSORFUSION_2_MAIN(cv::Mat _recv_img, vector<cv::Point> _img_coord, vector<cv::Point3f> _real_coord)
{
    mtx_sensor_fusion.lock();
    _recv_img.copyTo(m_sensor_fusion_img);
    m_fusion_img_coord_list = _img_coord;
    m_fusion_real_coord_list = _real_coord;

    m_sensor_fusion_img_q = Mat2QImage(m_sensor_fusion_img);
    m_sensor_fusion_img_p.convertFromImage(m_sensor_fusion_img_q);
    m_sensor_fusion_img_scene->clear();
    m_sensor_fusion_img_scene->addPixmap(m_sensor_fusion_img_p);

    ui->graphicsView_sensor_fusion->setScene(m_sensor_fusion_img_scene);
    ui->graphicsView_sensor_fusion->show();

    mtx_sensor_fusion.unlock();
}

void G_MAIN_WINDOW::on_pushButton_start_load_clicked()
{
    m_cam_num = ui->lineEdit_cam_num->text().toInt();

    c_t_imgload = new C_T_IMGLOAD[m_cam_num];

    for(uint cam_ind = 0;cam_ind < m_cam_num;cam_ind++)
    {
        switch(cam_ind)
        {
        case 0:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_1(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_1(cv::Mat)));
            m_lcm_obj.subscribe("CAM_1",&C_LCM_CAM::handleMessage_cam_1,&m_lcm_cam_obj_recv);
            break;
        case 1:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_2(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_2(cv::Mat)));
            m_lcm_obj.subscribe("CAM_2",&C_LCM_CAM::handleMessage_cam_2,&m_lcm_cam_obj_recv);
            break;
        case 2:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_3(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_3(cv::Mat)));
            m_lcm_obj.subscribe("CAM_3",&C_LCM_CAM::handleMessage_cam_3,&m_lcm_cam_obj_recv);
            break;
        case 3:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_4(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_4(cv::Mat)));
            m_lcm_obj.subscribe("CAM_4",&C_LCM_CAM::handleMessage_cam_4,&m_lcm_cam_obj_recv);
            break;
        case 4:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_5(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_5(cv::Mat)));
            m_lcm_obj.subscribe("CAM_5",&C_LCM_CAM::handleMessage_cam_5,&m_lcm_cam_obj_recv);
            break;
        case 5:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_6(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_6(cv::Mat)));
            m_lcm_obj.subscribe("CAM_6",&C_LCM_CAM::handleMessage_cam_6,&m_lcm_cam_obj_recv);
            break;
        case 6:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_7(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_7(cv::Mat)));
            m_lcm_obj.subscribe("CAM_7",&C_LCM_CAM::handleMessage_cam_7,&m_lcm_cam_obj_recv);
            break;
        case 7:
            connect(this,SIGNAL(SIG_MAIN_2_C_T_IMGLOAD_8(string)),&(c_t_imgload[cam_ind]),SLOT(SLOT_MAIN_2_C_T_IMGLOAD(string)));
            connect(&(c_t_imgload[cam_ind]),SIGNAL(SIG_C_T_IMGLOAD_2_MAIN_IMG(cv::Mat)),this,SLOT(SLOT_C_T_IMGLOAD_2_MAIN_8(cv::Mat)));
            m_lcm_obj.subscribe("CAM_8",&C_LCM_CAM::handleMessage_cam_8,&m_lcm_cam_obj_recv);
            break;
        }
    }

    c_t_lcmsubscr_cam->SetLCMObj(&m_lcm_obj);
    c_t_monitor->start();
    c_t_grab_vlp_16_hr->c_t_monitor_lidar->start();
}

void G_MAIN_WINDOW::on_pushButton_set_img_loadpath_clicked()
{
    QString directory;
    directory = QFileDialog::getExistingDirectory(this,"Select load folder",QDir::currentPath());

    if (!directory.isEmpty())
    {
        ui->lineEdit_img_loadpath->setText(directory);
        m_img_loadpath_str = directory.toStdString();
    }
}

void G_MAIN_WINDOW::on_pushButton_set_img_savepath_clicked()
{
    QString directory;
    directory = QFileDialog::getExistingDirectory(this,"Select save Folder",QDir::currentPath());

    if (!directory.isEmpty())
    {
        ui->lineEdit_img_savepath->setText(directory);
        m_img_savepath_str = directory.toStdString();
        for(uint cam_ind = 0;cam_ind < m_cam_num;cam_ind++)
        {
            string savefolder = m_img_savepath_str + "/CAM_" + std::to_string(cam_ind + 1);
            boost::filesystem::create_directories(savefolder);
        }
    }
}

void G_MAIN_WINDOW::on_pushButton_save_img_clicked()
{
    for(uint cam_ind = 0;cam_ind < m_cam_num;cam_ind++)
    {
        string savepath = m_img_savepath_str + "/CAM_" + std::to_string(cam_ind + 1) + "/" + std::to_string(m_timestamp) + ".jpg";
        cv::imwrite(savepath,m_ori_img[cam_ind]);
    }
}

void G_MAIN_WINDOW::on_pushButton_save_lidar_data_clicked()
{
    string savepath = m_img_savepath_str + "/vlp_16_" + std::to_string(m_timestamp) + ".xml";

    vector<double> save_pt_data_x;
    vector<double> save_pt_data_y;
    vector<double> save_pt_data_z;
    vector<uint8_t> save_pt_data_r;
    vector<uint8_t> save_pt_data_g;
    vector<uint8_t> save_pt_data_b;

    for(unsigned int pt_ind = 0;pt_ind < c_3d_viewer_obj->cloud->points.size();pt_ind++)
    {
        double data_x;
        double data_y;
        double data_z;
        uint8_t data_r;
        uint8_t data_g;
        uint8_t data_b;
        data_x = c_3d_viewer_obj->cloud->points.at(pt_ind).x;
        data_y = c_3d_viewer_obj->cloud->points.at(pt_ind).y;
        data_z = c_3d_viewer_obj->cloud->points.at(pt_ind).z;
        data_r = c_3d_viewer_obj->cloud->points.at(pt_ind).r;
        data_g = c_3d_viewer_obj->cloud->points.at(pt_ind).g;
        data_b = c_3d_viewer_obj->cloud->points.at(pt_ind).b;

        save_pt_data_x.push_back(data_x);
        save_pt_data_y.push_back(data_y);
        save_pt_data_z.push_back(data_z);
        save_pt_data_r.push_back(data_r);
        save_pt_data_g.push_back(data_g);
        save_pt_data_b.push_back(data_b);
    }


    cv::FileStorage fs(savepath,cv::FileStorage::WRITE);
    fs << "pt_data_x" << save_pt_data_x;
    fs << "pt_data_y" << save_pt_data_y;
    fs << "pt_data_z" << save_pt_data_z;
    fs << "pt_data_r" << save_pt_data_r;
    fs << "pt_data_g" << save_pt_data_g;
    fs << "pt_data_b" << save_pt_data_b;
    fs.release();
}

void G_MAIN_WINDOW::on_pushButton_init_drivenet_clicked()
{
    dwStatus status;
    dwStatus result = DW_SUCCESS;

    initSdk(&gSdk);

    // Setting virtual camera ----------------------------------------------------
    dwSALHandle_t sal = DW_NULL_HANDLE;
    dwSensorHandle_t camera = DW_NULL_HANDLE;
    dwRawPipelineHandle_t gRawPipeline              = DW_NULL_HANDLE;

    // init Drivenet
    DriveNet drivenet(gSdk);

    // Load sample image
    cv::Mat ori_img = cv::imread("/home/jung/Documents/1507702514030235.jpg");

    cv::cvtColor(ori_img,ori_img,CV_BGR2RGB);

    dwImageCPU  rawImageCPU;
    dwImageCUDA gRGBCudaImage;
    dwImageCUDA gRCBImage{};
    dwImageCUDA gRGBAImage{};


    dwImageProperties rawImageCPU_prop;

    rawImageCPU_prop.type = DW_IMAGE_CPU;
    rawImageCPU_prop.pxlFormat = DW_IMAGE_RGB;
    rawImageCPU_prop.planeCount = 1;
    rawImageCPU_prop.width = (uint32_t)ori_img.cols;
    rawImageCPU_prop.height = (uint32_t)ori_img.rows;
    rawImageCPU_prop.pxlType = DW_TYPE_UINT8;

    dwImageProperties cudaImageRGB_prop;

    cudaImageRGB_prop.type = DW_IMAGE_CUDA;
    cudaImageRGB_prop.pxlFormat = DW_IMAGE_RGB;
    cudaImageRGB_prop.planeCount = 1;
    cudaImageRGB_prop.width = (uint32_t)ori_img.cols;
    cudaImageRGB_prop.height = (uint32_t)ori_img.rows;
    cudaImageRGB_prop.pxlType = DW_TYPE_UINT8;

    dwImageProperties cudaImage_prop{};

    cudaImage_prop.type = DW_IMAGE_CUDA;
    cudaImage_prop.pxlFormat = DW_IMAGE_RCB;
    cudaImage_prop.planeCount = 3;
    cudaImage_prop.width = (uint32_t)ori_img.cols;
    cudaImage_prop.height = (uint32_t)ori_img.rows;
    cudaImage_prop.pxlType = DW_TYPE_UINT8;

    dwImageProperties rgbaImage_prop{};

    rgbaImage_prop.type = DW_IMAGE_CUDA;
    rgbaImage_prop.pxlFormat = DW_IMAGE_RGBA;
    rgbaImage_prop.planeCount = 1;
    rgbaImage_prop.width = (uint32_t)ori_img.cols;
    rgbaImage_prop.height = (uint32_t)ori_img.rows;
    rgbaImage_prop.pxlType = DW_TYPE_UINT8;

    dwVector2ui cam_resol;
    cam_resol.x = 960;
    cam_resol.y = 604;
    dwCameraProperties camera_props;
    camera_props.cameraType = DW_CAMERA_GENERIC;
    camera_props.framerate = 30;
    camera_props.siblings = 0;
    camera_props.resolution = cam_resol;
    camera_props.outputTypes = DW_CAMERA_RAW_IMAGE;

    dwSensorHandle_t gCameraSensor                  = DW_NULL_HANDLE;
    

    bool sensorsInitialized = initSensors(&sal, &gCameraSensor, &rawImageCPU_prop, &camera_props, gSdk);

    status = dwRawPipeline_initialize(&gRawPipeline, rawImageCPU_prop, camera_props, gSdk);
    if(status != DW_SUCCESS)
    {
        cout << "dw Raw Pipeline initialize failed" << dwGetStatusName(status) << endl;
    }
    else{
        cout << "dw Raw Pipeline initialize success" << endl;
    }

    status = dwImageCPU_create(&rawImageCPU,(const dwImageProperties*)&rawImageCPU_prop);

    if(status != DW_SUCCESS)
    {
        cout << "dw ImageCPU create error" << endl;
    }
    else{
        cout << "dw ImageCPU create success" << endl;
    }


    status = dwImageCUDA_create(&gRCBImage,(const dwImageProperties*)&cudaImage_prop,DW_IMAGE_CUDA_PITCH);

    if(status != DW_SUCCESS)
    {
        cout << "dw Image CUDA create error" << endl;
    }
    else{
        cout << "dw Image CUDA create success" << endl;
    }

    status = dwImageCUDA_create(&gRGBAImage,(const dwImageProperties*)&rgbaImage_prop,DW_IMAGE_CUDA_PITCH);

    if(status != DW_SUCCESS)
    {
        cout << "dw Image RGBA create error" << endl;
    }
    else{
        cout << "dw Image RGBA create success" << endl;
    }


    memcpy(rawImageCPU.data[0],ori_img.data, ori_img.total()*ori_img.channels());

    dwImageStreamerHandle_t gInput2cuda             = DW_NULL_HANDLE;
    dwImageStreamerHandle_t gCuda2gl                = DW_NULL_HANDLE;

    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gInput2cuda, &rawImageCPU_prop, DW_IMAGE_CUDA, gSdk);
    result = result != DW_SUCCESS ? result : dwImageStreamer_initialize(&gCuda2gl, &rgbaImage_prop, DW_IMAGE_GL, gSdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Image streamer initialization failed: " << dwGetStatusName(result) << std::endl;
        return;
    }

    dwImageFormatConverterHandle_t gConvert2RGBA    = DW_NULL_HANDLE;
    dwImageFormatConverterHandle_t gConvert2CUDA    = DW_NULL_HANDLE;

    // init format converter to convert from RCB->RGBA
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvert2RGBA, &cudaImage_prop, &rgbaImage_prop, gSdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Image format converter initialization failed: " << dwGetStatusName(result) << std::endl;
        return;
    }

    // init format converter to convert from RGB CUDA ->RCB
    result = result != DW_SUCCESS ? result : dwImageFormatConverter_initialize(&gConvert2CUDA, &cudaImageRGB_prop, &cudaImage_prop, gSdk);
    if (result != DW_SUCCESS) {
        std::cerr << "Image format converter initialization failed(CUDA): " << dwGetStatusName(result) << std::endl;
        return;
    }


    result = dwImageStreamer_postCPU(&rawImageCPU, gInput2cuda);
    if (result != DW_SUCCESS) {
        std::cout << "Cannot post raw image: " << dwGetStatusName(result) << std::endl;
        return;
    }

    dwImageCUDA *rawImageCUDA;

    result = dwImageStreamer_receiveCUDA(&rawImageCUDA, 10000, gInput2cuda);
    if (result != DW_SUCCESS) {
        std::cout << "Cannot receive CUDA Image " << dwGetStatusName(result) << std::endl;
        return;
    }

    cudaStream_t g_cudaStream  = 0;

    dwImageCUDA *rcbImageCUDA;
    
    status = dwImageFormatConverter_copyConvertCUDA(rcbImageCUDA,rawImageCUDA,gConvert2CUDA,g_cudaStream);

    if(status != DW_SUCCESS)
    {
        cout << "dw CUDA format convert error" << endl;
    }
    else{
        cout << "dw CUDA format convert success" << endl;
    }

}
