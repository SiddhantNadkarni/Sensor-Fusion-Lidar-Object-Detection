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
    Lidar* lidar(new Lidar(cars, 0)); //By instatinating on the heap, we have more memory to work with than the 2MB on the stack
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud = lidar->scan();
    // renderRays(viewer, lidar->position, lidarCloud);
    // renderPointCloud(viewer, lidarCloud, "Lidar Cloud");

    ProcessPointClouds<pcl::PointXYZ>* ppc(new ProcessPointClouds<pcl::PointXYZ>());
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc->SegmentPlane(lidarCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "Obstacle Point Cloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.first, "Road Cloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ppc->Clustering(segmentCloud.second, 1.0, 3, 30);
    int clusterID(0);
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for (int i = 0; i < cloudClusters.size(); ++i)
    {
        std::cout << "Cluster size: ";
        ppc->numPoints(cloudClusters[i]);
        renderPointCloud(viewer, cloudClusters[i], "Cluster " + std::to_string(clusterID), colors[i]);


        Box box = ppc->BoundingBox(cloudClusters[i]);
        renderBox(viewer, box, clusterID);        
        ++clusterID;
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* ppc, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = ppc->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-20, -6, -3, 1), Eigen::Vector4f ( 30, 7, 2, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = ppc->RansacPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "Road Cloud", Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = ppc->euclideanCluster(segmentCloud.first, 0.53, 10, 500);
    int clusterID(0);
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (int i = 0; i < cloudClusters.size(); ++i)
    {
        std::cout << "Cluster size: ";
        ppc->numPoints(cloudClusters[i]);
        renderPointCloud(viewer, cloudClusters[i], "Cluster " + std::to_string(i), colors[i]);


        Box box = ppc->BoundingBox(cloudClusters[i]);
        renderBox(viewer, box, i);        
    }
    // Box box;
    // box.x_min = -1.5;
    // box.y_min = -1.7;
    // box.z_min = -1;
    // box.x_max = 2.6;
    // box.y_max = 1.7;
    // box.z_max = -0.4;
    // renderBox(viewer, box, clusterID++, Color(1,1,1));  
    // renderPointCloud(viewer, filteredCloud, "Filter Cloud");
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* ppc(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = ppc->streamPcd("../src/sensors/data/pcd/data_1");
    auto iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = ppc->loadPcd((*iterator).string());
        cityBlock(viewer, ppc, inputCloud);

        iterator++;

        if(iterator == stream.end())
            iterator = stream.begin();
        viewer->spinOnce (); //Calls the interactor and updates the screen once
    } 
}