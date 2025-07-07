// PCL lib Functions for processing point clouds
#include "processPointClouds.h"
#include "quiz/ransac/ransac.hpp"
#include "quiz/cluster/kdtree.h"
#include "quiz/cluster/clustering.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region_extractor(true);
    region_extractor.setInputCloud(cloud);
    region_extractor.setMin(minPoint);
    region_extractor.setMax(maxPoint);
    region_extractor.filter(*cloud_region);


    std::vector<int> indices;
    typename pcl::PointCloud<PointT>::Ptr cloud_region_without_cars_roof(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> car_roof_extractor(true);
    car_roof_extractor.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    car_roof_extractor.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    car_roof_extractor.setInputCloud(cloud_region);
    car_roof_extractor.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);


    // points in inliers are extracted from the cloud and removed
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region_without_cars_roof);

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_region_without_cars_roof);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());
    for (auto inlier : inliers->indices) {
        road->points.push_back(cloud->points[inlier]);
    }


    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    auto inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index)) {
			plane->points.push_back(point);
        }
		else {
			obstacles->points.push_back(point);
        }
	}

    if (inliers.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return std::make_pair(obstacles, plane);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    KdTree tree;

    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); ++i) {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(point);
        tree.insert(points[i], i);
    }

    auto cluster_indices = euclideanCluster(points, &tree, clusterTolerance);
    for (const auto& cluster : cluster_indices)
    {
        if (cluster.size() <= maxSize && cluster.size() >= minSize) {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (const auto& idx : cluster) {
                cloud_cluster->push_back((*cloud)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end());

    return paths;

}