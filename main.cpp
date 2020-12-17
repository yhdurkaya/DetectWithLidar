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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>


int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_files/sdc.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewerInitial("Initial viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudHandler (cloud, 0, 255, 0); // Red
    viewerInitial.addPointCloud (cloud, cloudHandler, "Initial Cloud");

    while(!viewerInitial.wasStopped()){
        viewerInitial.spinOnce();
    }

/*
    DOWNSAMPLING THE CLOUD
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.25, 0.25, 0.25);
    voxelGrid.filter(*cloud);

    pcl::visualization::PCLVisualizer viewerDownsampled("Downsampled viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> downsampledCloudHandler (cloud, 0, 255, 0); // Red
    viewerDownsampled.addPointCloud (cloud, downsampledCloudHandler, "Downsampled Cloud");

    while(!viewerDownsampled.wasStopped()){
        viewerDownsampled.spinOnce();
    }
*/

    std::vector<int> inliers;
    std::vector<int> outliers;

    //GETTING THE INLIERS AND OUTLIERS
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

    //Binary Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (outlierCloud);

    //Euclidean Clustering
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanCluster;
    euclideanCluster.setClusterTolerance (0.22);
    euclideanCluster.setMinClusterSize (30);
    euclideanCluster.setMaxClusterSize (1000);
    euclideanCluster.setSearchMethod (tree);
    euclideanCluster.setInputCloud (outlierCloud);
    euclideanCluster.extract (clusterIndices);

    //TODO
    // NEED TO TRY WITH DBSCAN, EUCLIDEAN CLUSTERING IS NOT PROMISING
    std::cout << cloud->size() << "\n";
    std::cout << inlierCloud->size() << "\n";
    std::cout << outlierCloud->size() << "\n";
    std::cout << clusterIndices.size() << "\n";

    pcl::visualization::PCLVisualizer viewer("Original viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlierCloudColorHandler (inlierCloud, 0, 255, 0); // Red
    viewer.addPointCloud (inlierCloud, inlierCloudColorHandler, "Inlier Cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outlierCloudColorHandler (outlierCloud, 255, 0, 0); // Red
    viewer.addPointCloud (outlierCloud, outlierCloudColorHandler, "Outlier Cloud");

    pcl::visualization::PCLVisualizer viewerRANSAC("RANSAC viewer");
    viewerRANSAC.addPointCloud (inlierCloud, inlierCloudColorHandler, "Inlier Cloud");
    viewerRANSAC.addPointCloud (outlierCloud, outlierCloudColorHandler, "Outlier Cloud");

    while(!viewerRANSAC.wasStopped()){
        viewerRANSAC.spinOnce();
    }

    int j = 0;
    for (auto it = clusterIndices.begin (); it != clusterIndices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloudCluster->points.push_back(outlierCloud->points[*pit]); //*
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudClusterColorHandler (cloudCluster, 255, 255, 255); // Red
        viewer.addPointCloud (cloudCluster, cloudClusterColorHandler, "Cloud Cluster" + std::to_string(j));


        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloudCluster, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cloudCluster, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloudCluster, *cloudPointsProjected, projectionTransform);

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        //viewer.addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "cube" + std::to_string(j));
        ++j;
    }

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }
}
