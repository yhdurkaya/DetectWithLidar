//
// Created by yhd on 11.06.2020.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_files/sdc.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer("Original viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sourceCloudColorHandler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, sourceCloudColorHandler, "Original Cloud");

    std::cout << cloud->size() << "\n";

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.5, 0.5, 0.5);
    voxelGrid.filter(*cloud);

    pcl::visualization::PCLVisualizer viewer2("Downsampled viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsampledCloudColorHandler (cloud, 255, 255, 255); // Red
    viewer2.addPointCloud (cloud, downsampledCloudColorHandler, "Downsampled Cloud");

    std::cout << cloud->size() << "\n";


    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }

    while(!viewer2.wasStopped()){
        viewer.spinOnce();
    }

}
