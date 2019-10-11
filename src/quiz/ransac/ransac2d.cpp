/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../customRansac/customRansac.h"
#include "../../customRansac/customRansac.cpp"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"



pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
// 	std::unordered_set<int> inliersResult;
// 	srand(time(NULL));
//
// 	// TODO: Fill in this function
//
// 	// For max iterations
//
// 	// Randomly sample subset and fit line
//
// 	// Measure distance between every point and fitted line
// 	// If distance is smaller than threshold count it as inlier
//
// 	// Return indicies of inliers from fitted line with most inliers
// 	uint data_size = cloud->points.size();
// 	for (size_t i{0}; i < maxIterations; i++) {
// 		std::unordered_set<int> inliers;
// 		while (inliers.size() < 3) {
// 			inliers.insert(rand() % data_size);
// 		}
// 		auto it = inliers.begin();
// 		float x1 = cloud->points[*it].x;
// 		float y1 = cloud->points[*it].y;
// 		float z1 = cloud->points[*it].z;
// 		++it;
// 		float x2 = cloud->points[*it].x;
// 		float y2 = cloud->points[*it].y;
// 		float z2 = cloud->points[*it].z;
// 		++it;
// 		float x3 = cloud->points[*it].x;
// 		float y3 = cloud->points[*it].y;
// 		float z3 = cloud->points[*it].z;
//
// 		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
// 		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
// 		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
// 		float d = -(a*x1 + b*y1 + c*z1);
//
// 		for (size_t idx{0}; idx<data_size; idx++) {
// 			if (inliers.count(idx)>0) {
// 				continue;
// 			}
// 			float x = cloud->points[idx].x;
// 			float y = cloud->points[idx].y;
// 			float z = cloud->points[idx].z;
// 			float distance = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
//
// 			if (distance < distanceTol) {
// 				inliers.insert(idx);
// 			}
// 		}
// 		if (inliers.size() > inliersResult.size()) {
// 			inliersResult = inliers;
// 		}
// 	}
// 	return inliersResult;
//
// }

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// Ransac<pcl::PointXYZ> ransac {};
	// std::unordered_set<int> inliers = ransac.Ransac2D(cloud, 100, 0.1);
	//
	// pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	//
	// for(int index = 0; index < cloud->points.size(); index++)
	// {
	// 	pcl::PointXYZ point = cloud->points[index];
	// 	if(inliers.count(index))
	// 		cloudInliers->points.push_back(point);
	// 	else
	// 		cloudOutliers->points.push_back(point);
	// }
	//

	// Render 2D point cloud with inliers and outliers
	// if(inliers.size())
	// {
	// 	renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  // 		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	// }
  // 	else
  // 	{
  // 		renderPointCloud(viewer,cloud,"data");
  // 	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
