#ifndef C_3D_VIEWER_H
#define C_3D_VIEWER_H
//-------------------------------------------------
// VTK - pck header
//-------------------------------------------------
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCubeSource.h>
#include <vtkVRMLImporter.h>
#include <vtk3DSImporter.h>
#include <vtkSmartPointer.h>
#include <vtkMapper.h>
#include <vtkLight.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
//for projection
#include <pcl/filters/project_inliers.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sensor/c_vlp_16_hr.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef struct THREE_DIMENSIONAL_DATA_TYPE{
    double x;
    double y;
    double z;

}DATA_TYPE_3D;

using namespace std;

class C_3D_VIEWER
{
public:
    C_3D_VIEWER() {
        init();
    }

    void init();
    void SetVelodyneData();

    VLP_16_HR_DATA m_vlp_16_hr_data_ary[VLP_16_HR_TOTAL_PACKET_NUMBER];//360 deg Data

    double m_x_data_arr[VLP_16_HR_LASERS_NUM][VLP_16_HR_BOLCKS_NUM*VLP_16_HR_TOTAL_PACKET_NUMBER] = {{0}};
    double m_y_data_arr[VLP_16_HR_LASERS_NUM][VLP_16_HR_BOLCKS_NUM*VLP_16_HR_TOTAL_PACKET_NUMBER] = {{0}};
    double m_z_data_arr[VLP_16_HR_LASERS_NUM][VLP_16_HR_BOLCKS_NUM*VLP_16_HR_TOTAL_PACKET_NUMBER] = {{0}};
    int m_intensity_data_arr[VLP_16_HR_LASERS_NUM][VLP_16_HR_BOLCKS_NUM*VLP_16_HR_TOTAL_PACKET_NUMBER] = {{0}};

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    PointCloudT::Ptr cloud;
};


#endif // C_3D_VIEWER_H
