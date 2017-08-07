#include<iostream>
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

using namespace pcl;
using namespace std;


int main()
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    io::loadPCDFile<pcl::PointXYZ>("test1.pcd", *cloud);

    const float min_scale = 0.01f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(0.2);
    ne.compute(*cloud_normals);

    for(size_t i = 0; i<cloud_normals->points.size(); ++i)
    {
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;
    }

    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(result);

    std::cout << "Number of SIFT points in the result is " << result.points.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);

    vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
    for(it = cloud_temp->points.begin(); it != cloud_temp->points.end(); ++it)
    {
        cout << *it << endl;
    }

    boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoint_color_handler (cloud_temp, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_color_handler, "final cloud");
    viewer->addPointCloud<pcl::PointXYZ> (cloud_temp, keypoint_color_handler, "key point");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "final cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "key point");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
        pcl_sleep(0.01);
    }
}