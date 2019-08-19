/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool renderCluster = true;
    bool renderBoxes = true;
    bool renderSegmentation = false;
    bool renderRawCloud = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    // Create lidar sensor
    auto myLidarSensor = new Lidar(cars, 0.0);
    //renderRays(viewer,myLidarSensor->position,myLidarSensor->scan());
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = myLidarSensor->scan();
    if (renderSegmentation)
    {
        renderPointCloud(viewer, inputCloud, "test", Color(0, 0.5, 0));
    }
    // Create point processor
    auto myProcessPointClouds = new ProcessPointClouds<pcl::PointXYZ>();
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = myProcessPointClouds->SegmentPlane(inputCloud,1000,0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = myProcessPointClouds->SegmentPlaneOwn(inputCloud, 1000, 0.2);
    if (renderSegmentation)
    {
        renderPointCloud(viewer, segResult.first, "obstacle", Color(1, 0, 0));
        renderPointCloud(viewer, segResult.second, "plane", Color(0, 1, 0));
    }

    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = myProcessPointClouds->Clustering(segResult.first, 1.0, 3, 30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = myProcessPointClouds->ClusteringOwn(segResult.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (renderCluster)
        {
            std::cout << "cluster size ";
            myProcessPointClouds->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }
        if (renderBoxes)
        {
            //  Box box = myProcessPointClouds->BoundingBox(cluster);
            BoxQ box = myProcessPointClouds->BoundingBoxQ(cluster);
            renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);
        }
        ++clusterId;
    }
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderRawCloud = false;
    bool renderFilteredCloud = false;
    bool renderSegmentation = true;
    bool renderCluster = true;
    bool renderBoxes = false;
    bool renderBoxesQ = true;

    //ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    if (renderRawCloud)
    {
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
    if (renderRawCloud)
    {
        renderPointCloud(viewer, filterCloud, "filterCloud");
    }
    //Obstacle detection pipline
    //1. Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segCloud = pointProcessorI->SegmentPlaneOwn(filterCloud, 25, 0.3);
    if (renderSegmentation)
    {
      //  renderPointCloud(viewer, segCloud.first, "obstacle", Color(1, 0, 0));
        renderPointCloud(viewer, segCloud.second, "roadPlane", Color(0, 1, 0));
    }
    //2. Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringOwn(segCloud.first, 0.53, 10, 500);
    //3. BoundingBoxes
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(0.2, 0, 1), Color(0.2, 0.6, 0.3)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (renderCluster)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }
        if (renderBoxes)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);
        }
                if (renderBoxesQ)
        {
            BoxQ box = pointProcessorI->BoundingBoxQ(cluster);
            renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);
        }
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}();
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //simpleHighway(viewer);

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}
