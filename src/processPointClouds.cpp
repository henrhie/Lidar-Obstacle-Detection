// PCL lib Functions for processing point clouds

#include "processPointClouds.h"



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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered {new pcl::PointCloud<pcl::PointXYZI>};
    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(filterRes, filterRes, filterRes);
    grid.filter(*cloud);

    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*cloud);

    std::vector<int> indices;
    pcl::CropBox<PointT> roofCrop {true};
    roofCrop.setInputCloud(cloud);
    roofCrop.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roofCrop.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roofCrop.filter(indices); //indices of box region we want to remove

    pcl::PointIndices::Ptr point_indices {new pcl::PointIndices};
    for (auto idx : indices) {
      point_indices->indices.push_back(idx);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(point_indices);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
// {
//
//   // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
//     typename pcl::PointCloud<PointT>::Ptr obsCloud {new pcl::PointCloud<PointT>};
//     typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};
//     for (size_t i{0}; i<cloud->points.size(); i++) {
//       if (inliers.count(i)>0) {
//         obsCloud->points.push_back(cloud->points[i]);
//       }
//       else {
//         planeCloud->points.push_back(cloud->points[i]);
//       }
//     }
//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);
//     return segResult;
// }
//
//
// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
// 	  //pcl::PointIndices::Ptr inliers;
//
//     // TODO:: Fill in this function to find inliers for the cloud.
//     Ransac<PointT> ransac {};
//     std::unordered_set<int> inliers = ransac.Ransac3D(cloud, maxIterations, distanceThreshold);
//
//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
//
//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
//     return segResult;
// }

// template <typename PointT>
// void ProcessPointClouds<PointT>::clusterHelper(int idx, std::vector<int>& cluster, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, std::vector<bool>& processed, KdTree<PointT>* tree) {
//   processed[idx] = true;
//   cluster.push_back(idx);
//   std::vector<int> ids = tree->search(cloud->points[idx], distanceTol);
//
//   for (auto id : ids) {
//     if (!processed[id]) {
//       clusterHelper(id, cluster, cloud, distanceTol, processed, tree);
//     }
//   }
// }

// template<typename PointT>
// std::vector<std::vector<int>> ProcessPointClouds<PointT>::cluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
// {
//   std::vector<bool> processed {cloud->points.size(), false};
//   std::vector<std::vector<int>> clusters;
//   int idx{0};
//   while(idx < cloud->points.size())
//   {
//     if (processed.at(idx)){
//       idx++;
//       continue;
//     }
//     std::vector<int> cluster;
//     clusterHelper(idx, cluster, cloud, distanceTol, processed, tree);
//     clusters.push_back(cluster);
//     idx++;
//
//   }
//   return clusters;
// }
//
// // template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {
//     KdTree<PointT>* tree {new KdTree<PointT>()};
//     for (size_t i{0}; i<cloud->points.size(); ++i) {
//       tree->insert(cloud->points[i], i);
//     }
//
//
//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();
//
//     std::vector<typename pcl::PointCloud<PointT>::Ptr> _clusters;
//     std::vector<std::vector<int>> clusters = cluster(cloud, tree, clusterTolerance);
//     for (std::vector<int> cluster : clusters) {
//       typename pcl::PointCloud<PointT>::Ptr clusterCloud {new pcl::PointCloud<PointT>};
//       for (int idx : cluster) {
//         clusterCloud->points.push_back(cloud->points[idx]);
//       }
//       if (clusterCloud->points.size() > minSize && clusterCloud->points.size() < maxSize){
//         _clusters.push_back(clusterCloud);
//       }
//
//     }
//
//     // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
//
//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
//
//     return _clusters;
// }


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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
