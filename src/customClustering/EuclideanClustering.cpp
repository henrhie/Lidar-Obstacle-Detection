#include <boost/assign/list_of.hpp>
#include "EuclideanClustering.h"

struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;
  Node(std::vector<float> _point, int _id) : point{_point}, id{_id}, left(NULL), right(NULL)
  {}
};

template<typename PointT>
struct KdTree{
  Node* root;
  KdTree() : root{NULL}{}

  void insertHelper(Node** node, PointT pcl_point, uint depth, int id) {
    std::vector<float> point = boost::assign::list_of(pcl_point.x)(pcl_point.y)(pcl_point.z);

    if (*node == NULL) {
      *node = new Node(point, id);
    }
    else {
      int axis = depth % 3;

      if (point.at(axis) < (*node)->point[axis]) {
        insertHelper(&((*node)->left), pcl_point, depth+1, id);
      }
      else {
        insertHelper(&((*node)->right), pcl_point, depth+1, id);
      }
    }
  }

  void insert(PointT point, int id) {
    insertHelper(&root, point, 0, id);
  }

  void searchHelper(Node* node, PointT _target, uint depth, std::vector<int>& ids, float distanceTol) {

    std::vector<float> target = boost::assign::list_of(_target.x)(_target.y)(_target.z);

    if (node != NULL) {

      if (distanceTol>=(target.at(0)-node->point.at(0))&&-distanceTol<=(target.at(0)-node->point.at(0))&&
          distanceTol>=(target.at(1)-node->point.at(1))&&-distanceTol<=(target.at(1)-node->point.at(1))&&
          distanceTol>=(target.at(2)-node->point.at(2))&&-distanceTol<=(target.at(2)-node->point.at(2)))
          {

            float distance = sqrt(std::pow(target.at(0)-node->point.at(0),2) +
            std::pow(target.at(1)-node->point.at(1),2) +
            std::pow(target.at(2)-node->point.at(2),2));

            if (distance <= distanceTol) {
              ids.push_back(node->id);
            }
          }

        if ((target.at(depth%3)-distanceTol)<node->point.at(depth%3)) {
          searchHelper(node->left, _target, depth+1, ids, distanceTol);
        }

        if ((target.at(depth%3)+distanceTol)>node->point.at(depth%3)) {
          searchHelper(node->right, _target, depth+1, ids, distanceTol);
        }
    }
  }


  std::vector<int> search(PointT target, float distanceTol)
  {
    std::vector<int> ids;
    searchHelper(root, target, 0, ids, distanceTol);

    return ids;
  }

  void clusterHelper(int idx, std::vector<int>& cluster, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, std::vector<bool>& processed) {
    processed[idx] = true;
    cluster.push_back(idx);
    std::vector<int> ids = search(cloud->points[idx], distanceTol);


    for (auto id : ids) {
      if (!processed[id]) {
        clusterHelper(id, cluster, cloud, distanceTol, processed);

      }
    }
  }

  std::vector<std::vector<int>> cluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol)
  {
    std::vector<bool> processed (cloud->points.size(), false);
    std::vector<std::vector<int>> clusters;
    int idx{ 0 };
    while(idx < cloud->points.size())
    {
      if (processed.at(idx)){
        idx++;
        continue;
      }
      std::vector<int> cluster;
      clusterHelper(idx, cluster, cloud, distanceTol, processed);
      clusters.push_back(cluster);
      idx++;

    }

    return clusters;
  }

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
  {
      // Time clustering process

      auto startTime = std::chrono::steady_clock::now();
      for (size_t i{0}; i<cloud->points.size(); ++i) {
        insert(cloud->points[i], i);
      }

      std::vector<typename pcl::PointCloud<PointT>::Ptr> _clusters;
      std::vector<std::vector<int>> clusters = cluster(cloud, clusterTolerance);
      for (std::vector<int> cluster : clusters) {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud {new pcl::PointCloud<PointT>};
        for (int idx : cluster) {
          clusterCloud->points.push_back(cloud->points[idx]);
        }
        if (clusterCloud->points.size() > minSize && clusterCloud->points.size() < maxSize){
          _clusters.push_back(clusterCloud);
        }

      }

      auto endTime = std::chrono::steady_clock::now();
      auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
      std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

      return _clusters;
  }

};
