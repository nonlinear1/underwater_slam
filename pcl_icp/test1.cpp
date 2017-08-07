#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/transforms.h>

using namespace pcl;
using namespace std;


int main()
{
    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr final_cloud(new PointCloud<PointXYZ>);

    io::loadPCDFile<pcl::PointXYZ>("test1/test4.pcd", *cloud1);
    io::loadPCDFile<pcl::PointXYZ>("test1/test5.pcd", *cloud2);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 1.0, 0.0, 0.0;

    pcl::transformPointCloud(*cloud1, *final_cloud, transform);

    boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoint_color_handler (cloud1, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (final_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, cloud_color_handler, "cloud2");
    viewer->addPointCloud<pcl::PointXYZ> (final_cloud, keypoint_color_handler, "point1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point1");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
        pcl_sleep(0.01);
    }
}

