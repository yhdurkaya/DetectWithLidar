//
// Created by yhd on 11.06.2020.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_files/sdc.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer("Original viewer");

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.5, 0.5, 0.5);
    voxelGrid.filter(*cloud);

    std::vector<int> inliers;
    std::vector<int> outliers;

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelPlane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (modelPlane);
    ransac.setDistanceThreshold (0.25);
    ransac.computeModel();
    ransac.setMaxIterations(150);
    ransac.getInliers(inliers);

    pcl::copyPointCloud (*cloud, inliers, *inlierCloud);

    for(int i =0; i<cloud->size(); ++i){
        if(std::find(inliers.begin(), inliers.end(), i) != inliers.end()){

        }
        else{
            outliers.push_back(i);
        }
    }
    pcl::copyPointCloud(*cloud, outliers, *outlierCloud);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlierCloudColorHandler (inlierCloud, 0, 255, 0); // Red
    viewer.addPointCloud (inlierCloud, inlierCloudColorHandler, "Inlier Cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outlierCloudColorHandler (outlierCloud, 255, 0, 0); // Red
    viewer.addPointCloud (outlierCloud, outlierCloudColorHandler, "Outlier Cloud");

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }

    std::cout << cloud->size() << "\n";
    std::cout << inlierCloud->size() << "\n";
    std::cout << outlierCloud->size() << std::endl;
}
