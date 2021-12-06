//
// Edited by Dominic Carrillo on 24/10/21.
//

// Sourced from:
//  https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html#compiling-and-running-the-program

#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "normal_distributions_transform.h"

using namespace std::chrono_literals;

int NDT::ndt () {
    std::string targetFile;
    std::string inputFile;

    targetFile = "./ptCloudSrc.pcd";
    inputFile = "./ptCloudRef.pcd";

    // Loading first scan.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (targetFile, *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read target file! Make sure it is a pcd file.\n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size () << " data points from " << targetFile << std::endl;

    // Loading second scan.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (inputFile, *input_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read input file! Make sure it is a pcd file. \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size () << " data points from " << inputFile << std::endl;

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size ()
              << " data points from file 2 pcd" << std::endl;

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // OS time start
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    ndt.align (*output_cloud, init_guess);

    // OS time end
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    // output time
    std::cout << "Normal Distribution Transform Time Computation: "
	    << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
	    << " milliseconds."
            << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

    // Saving transformed input cloud.
    pcl::io::savePCDFileASCII ("transformedCloud.pcd", *output_cloud);

    return (0);
}

