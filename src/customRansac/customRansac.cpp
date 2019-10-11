#include "customRansac.h"
#include <unordered_set>

template<typename PointT>
Ransac<PointT>::Ransac(){};

template<typename PointT>
Ransac<PointT>::~Ransac(){};

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, float maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  uint data_size = cloud->points.size();
  for (size_t i{0}; i < maxIterations; i++) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 2) {
      inliers.insert(rand() %  data_size);
    }

    auto it = inliers.begin();
    float x1 = cloud->points[*it].x;
    float y1 = cloud->points[*it].y;
    ++it;
    float x2 = cloud->points[*it].x;
    float y2 = cloud->points[*it].y;
    float a = y1-y2;
    float b = x2-x1;
    float c = (x1*y2 - x2*y1);

    for (size_t idx{0}; idx < data_size; ++idx) {
      if (inliers.count(idx)>0) {
        continue;
      }
      float x = cloud->points[idx].x;
      float y = cloud->points[idx].y;
      float distance = fabs(a*x+b*y+c)/sqrt(a*a+b*b);

      if (distance < distanceTol) {
        inliers.insert(idx);
      }
    }
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }
  return inliersResult;
}

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, float maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));


	uint data_size = cloud->points.size();
	for (size_t i{0}; i < maxIterations; i++) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand() % data_size);
		}
		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		float z1 = cloud->points[*it].z;
		++it;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;
		float z2 = cloud->points[*it].z;
		++it;
		float x3 = cloud->points[*it].x;
		float y3 = cloud->points[*it].y;
		float z3 = cloud->points[*it].z;

		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float d = -(a*x1 + b*y1 + c*z1);

		for (size_t idx{0}; idx<data_size; idx++) {
			if (inliers.count(idx)>0) {
				continue;
			}
			float x = cloud->points[idx].x;
			float y = cloud->points[idx].y;
			float z = cloud->points[idx].z;
			float distance = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);

			if (distance < distanceTol) {
				inliers.insert(idx);
			}
		}
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{

    typename pcl::PointCloud<PointT>::Ptr obsCloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};
    for (size_t i{0}; i<cloud->points.size(); i++) {
      if (inliers.count(i)>0) {
        obsCloud->points.push_back(cloud->points[i]);
      }
      else {
        planeCloud->points.push_back(cloud->points[i]);
      }
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}
