#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include <pcl/common/transforms.h>

#ifdef DEBUG
#define  OUT(str) std::cout << str << endl;
#else
#define OUT(str)
#endif

using namespace pcl;

search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>());
PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
PointCloud<PointXYZ>::Ptr cloud_filter(new PointCloud<PointXYZ>);


PointCloud<PointXYZ>::Ptr Sample(PointCloud<PointXYZ>::Ptr cloud)
{
    PointCloud<PointXYZ>::Ptr cloud_output(new PointCloud<PointXYZ>);

    VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filter);

    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud(cloud_filter);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1f);
    ne.compute(*cloud_normals);

    NormalSpaceSampling<PointXYZ, Normal> ns;
    ns.setInputCloud(cloud_filter);
    ns.setNormals(cloud_normals);
    ns.setSample(2000);
    ns.setSeed(0);
    ns.setBins(5,5,5);
    ns.filter(*cloud_output);

    return cloud_output;
}

int main()
{
    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr final(new PointCloud<PointXYZ>);

    io::loadPCDFile<pcl::PointXYZ>("test1.pcd", *cloud1);
    io::loadPCDFile<pcl::PointXYZ>("test2.pcd", *cloud2);

    PointCloud<PointXYZ>::ConstPtr cloud1_out = Sample(cloud1);
    PointCloud<PointXYZ>::ConstPtr cloud2_out = Sample(cloud2);

    registration::CorrespondenceRejectorDistance::Ptr rejector_dis(new registration::CorrespondenceRejectorDistance);
    rejector_dis->setMaximumDistance(0.2);

    registration::CorrespondenceRejector::Ptr rejector_boundy(new registration::CorrespondenceRejectionOrganizedBoundary);

    IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2_out);
    icp.setInputTarget(cloud1_out);
    //icp.addCorrespondenceRejector(rejector_dis);
    //icp.addCorrespondenceRejector(rejector_boundy);
    icp.align(*final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    transformPointCloud(*cloud2, *cloud2, icp.getFinalTransformation());

    boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud1_out, "final cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "final cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud2, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color, "cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}
