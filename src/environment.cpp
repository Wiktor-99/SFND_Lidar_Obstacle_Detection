/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    auto lidar = std::make_unique<Lidar>(cars, 0);
    auto scan = lidar->scan();
    // renderRays(viewer, lidar->position, scan);

    // renderPointCloud(viewer, scan, "first_frame");
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    auto segmentCloud = pointProcessor.SegmentPlane(scan, 10000, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0.5), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),colors[clusterId]);
        auto boundingBox = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, boundingBox, clusterId, colors[clusterId]);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    auto filterCloud = pointProcessor.FilterCloud(inputCloud, .5 , Eigen::Vector4f(-15, -6.0, -2, 1), Eigen::Vector4f(15, 6, 1, 1));
    auto segmentCloud = pointProcessor.SegmentPlane(filterCloud, 50, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.6, 8, 400);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0.5), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),colors[clusterId % colors.size()]);
        auto boundingBox = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, boundingBox, clusterId, colors[clusterId % colors.size() ]);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
    auto filterCloud = pointProcessor.FilterCloud(inputCloud, .5 , Eigen::Vector4f(-15, -6.0, -2, 1), Eigen::Vector4f(15, 6, 1, 1));
    // renderPointCloud(viewer, filterCloud, "inputCloud");


    auto segmentCloud = pointProcessor.SegmentPlane(filterCloud, 50, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.6, 8, 400);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0.5), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),colors[clusterId % colors.size()]);
        auto boundingBox = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, boundingBox, clusterId, colors[clusterId % colors.size() ]);
        ++clusterId;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<std::filesystem::path> stream = pointProcessorI.streamPcd("src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
}