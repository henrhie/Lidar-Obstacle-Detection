#ifndef RANSAC_H
#define  RANSAC_H
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


template<typename PointT>
class Ransac {

public:
  Ransac();

  ~Ransac();

  std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, float maxIterations, float distanceTol);

  std::unordered_set<int> Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, float maxIterations, float distanceTol);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);


};


#endif
