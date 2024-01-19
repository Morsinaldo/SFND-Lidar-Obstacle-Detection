#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// templates for processPointClouds defined in .cpp file
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

/************ use simpleHighway to test simulated Lidar data *******/
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // DONE:: Create lidar sensor 
    Lidar* lidar_sensor = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcd = lidar_sensor ->scan();
    // renderRays(viewer, lidar_sensor->position, lidar_pcd);
    renderPointCloud(viewer, lidar_pcd, "lidar_pcd");

    // DONE:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    // segment plane for road plane isolation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(lidar_pcd, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // clutering on non-road points, to mark objects on road
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    // render clusters
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(cluster_id),colors[cluster_id]);

          // add bounding boxes
          if (render_box) {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,cluster_id);
          }
          ++cluster_id;
    }
}

/*****************************************************************/
/************ use cityBlock to test on real Lidar data *******/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // hyperparameters
  // filter params
  float filter_res = 0.4;
  Eigen::Vector4f minPoint(-10, -6.5, -2, 1);
  Eigen::Vector4f maxPoint(30, 6.5, 1, 1);
  // segment params
  int max_iter = 40;
  float distance_threshold = 0.3;
  // cluster params
  float cluster_tolerance = 0.5;
  int min_cluster_size = 10;
  int max_cluster_size = 140;

  // Filter cloud, to reduce omputational cost
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filter_res, minPoint, maxPoint);

  // Segment the filtered cloud into road and obstacle points
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlaneSegment(filterCloud, max_iter, distance_threshold);

  // Render the segmented road and obstacle points
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  // Cluster the obstacle points using Euclidean Clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, cluster_tolerance, min_cluster_size, max_cluster_size);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(cluster_id),colors[cluster_id]);

        // Step 3. Find bounding boxes for the clusters
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,cluster_id);

        ++cluster_id;
  }

}
/*****************************************************************/


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

/********************** main function ************************/
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // std::vector<std::__fs::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1"); // for Mac
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
      {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // stream
        streamIterator++;
        if(streamIterator == stream.end())
          streamIterator = stream.begin();
        viewer->spinOnce ();
      }
}