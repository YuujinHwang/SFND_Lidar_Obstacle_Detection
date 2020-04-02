/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
// #include "render/render.h"
// #include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "processPointClouds.cpp"
// #include "quiz/cluster/kdtree3.h"
#include "quiz/ransac/ransac3d.h"
#include "quiz/cluster/cluster.cpp"

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
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //dec on stack
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //dec on heap
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // renderRays(viewer, lidar->position,cloud);
    // renderPointCloud(viewer, cloud, "cloud");
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloud,100,0.2);
    // renderPointCloud(viewer, segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 0, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,0.2,Eigen::Vector4f(-15,-7,-5,1), Eigen::Vector4f(30,7,0,1));
    
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,100,0.5);
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 10000);
    
    std::unordered_set<int> inliers = Ransac3d(filterCloud, 50, 0.3);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudObstacle(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < filterCloud->points.size(); index++)
    {
        pcl::PointXYZI point = filterCloud->points[index];
        if(inliers.count(index))
            cloudPlane->points.push_back(point);
        else
            cloudObstacle->points.push_back(point);
    }

    KdTree* tree = new KdTree;
    int obstsize = cloudObstacle->points.size();
    for(int i = 0; i<obstsize; i++)
        // std::vector<float> opoint = {cloudObstacle->points[i][0], }
        tree->insert(cloudObstacle->points[i],i);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(cloudObstacle, tree, 0.5);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        // for (indice: cluster)
        //     clusterCloud->points.push_back(cloudObstacle->points[indice]);
        renderPointCloud(viewer, cluster, "obstCLoud"+std::to_string(clusterId));
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }
    
    renderPointCloud(viewer, cloudObstacle,"obstCloud", Color(0,0,1));
    renderPointCloud(viewer, cloudPlane,"planeCloud",Color(0,1,0));

    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    // for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "cluster size";
    //     pointProcessorI->numPoints(cluster);
    //     renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

    //     Box box = pointProcessorI->BoundingBox(cluster);
    //     renderBox(viewer,box,clusterId);

    //     ++clusterId;
    // }
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer,pointProcessorI, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}