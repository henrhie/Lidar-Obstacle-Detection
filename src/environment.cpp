/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "customRansac/customRansac.h"
#include "customRansac/customRansac.cpp"
#include "customClustering/EuclideanClustering.h"
#include "customClustering/EuclideanClustering.cpp"

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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor

    // TODO:: Create point processor

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
  ProcessPointClouds<pcl::PointXYZI>* processor) {
  processor->FilterCloud(cloud, 0.3, Eigen::Vector4f(-18,-5,-3,1), Eigen::Vector4f(18,7,9,1));
  Ransac<pcl::PointXYZI>* ransac = new Ransac<pcl::PointXYZI>();
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = ransac->SegmentPlane(cloud, 100, .2);

  renderPointCloud(viewer, segResult.first, "pl", Color(0,1,0));
  //renderPointCloud(viewer, segResult.second, "obs", Color(1,0,0));
  KdTree<pcl::PointXYZI> tree {};
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = tree.Clustering(segResult.second, 0.5, 10, 2500);
  std::vector<Color> colors {Color(0,0,1), Color(1,0,1), Color(0,1,1)};
  int idx {0};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud : clusters) {
    Box box = processor->BoundingBox(clusterCloud);
    renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(idx), colors[idx%3]);
    renderBox(viewer, box, idx, Color(1,0,0),0.7);
    idx++;
  }
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* processor {new ProcessPointClouds<pcl::PointXYZI>()};
    std::vector<boost::filesystem::path> paths = processor->streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path>::iterator it = paths.begin();


    while (!viewer->wasStopped ())
    {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = processor->loadPcd((*it).string());
      cityBlock(viewer, cloud, processor);
      ++it;
      if (it == paths.end())
        it = paths.begin();
      viewer->spinOnce ();
    }
}
