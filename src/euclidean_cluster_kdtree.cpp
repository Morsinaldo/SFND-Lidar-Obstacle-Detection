#include <chrono>
#include <string>
#include "kdtree_node.h"

/**
 * @brief Implementation of Cluster_KDTree class.
 * 
 * This class implements the Euclidean clustering algorithm using KD-Tree.
 *
 * Author: Morsinaldo Medeiros
 * Date: 2023-12-28
 * 
 * Implementation reference: https://github.com/MohamedHussein736/Lidar-Obstacle-Detection/blob/master/src/cluster_kdtree.cpp
 * Github Copilot was used to help with the implementation.
 * 
 */

template<typename PointT>
class Cluster_KDTree {
private:
  int num_points;                  // Total number of points in the point cloud
  float cluster_tolerance;         // Distance tolerance for clustering
  int min_cluster_size;            // Minimum size of a valid cluster
  int max_cluster_size;            // Maximum size of a valid cluster
  std::vector<bool> processed;     // Keeps track of processed points during clustering
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;  // Stores the resulting clusters

public:
  /**
   * @brief Constructor for Cluster_KDTree class.
   *
   * @param n_pts Number of points in the point cloud.
   * @param clust_tol Distance tolerance for clustering.
   * @param min_size Minimum size of a valid cluster.
   * @param max_size Maximum size of a valid cluster.
   */
  Cluster_KDTree(int n_pts, float clust_tol, int min_size, int max_size) :
      num_points(n_pts), cluster_tolerance(clust_tol), min_cluster_size(min_size), max_cluster_size(max_size) {
        processed.assign(num_points, false);
      }

  /**
   * @brief Destructor for Cluster_KDTree class.
   */
  ~Cluster_KDTree();

  /**
   * @brief Recursively finds neighboring points and adds them to the cluster.
   *
   * @param cloud Point cloud to be clustered.
   * @param tree KD-Tree structure for efficient point searching.
   * @param cluster Vector storing indices of points belonging to the cluster.
   * @param index Index of the current point being processed.
   */
  void cluster_neighbors(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int index);

  /**
   * @brief Performs Euclidean clustering on a point cloud.
   *
   * @param cloud Input point cloud to be clustered.
   * @return Vector of point cloud pointers representing individual clusters.
   */
  std::vector<typename pcl::PointCloud<PointT>::Ptr> euclidean_cluster(typename pcl::PointCloud<PointT>::Ptr cloud);
};

/**********************************************************************/
// Destructor
template<typename PointT>
Cluster_KDTree<PointT>::~Cluster_KDTree() {}

/**********************************************************************/
// Recursively find and add neighboring points to a cluster
template<typename PointT>
void Cluster_KDTree<PointT>::cluster_neighbors(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int index) {

  // Mark the point as processed
  processed[index] = true;
  cluster.push_back(index);

  // Find the neighboring points
  std::vector<int> neighbor_points = tree->search(cloud->points[index], cluster_tolerance);

  // Iterate through the neighboring points
  for(int neighbor_id: neighbor_points) {
    // Check if the point has already been processed
    if (!processed[neighbor_id]) {
      // Recursively find points in the cluster
      cluster_neighbors(cloud, tree, cluster, neighbor_id);
    } else {
      continue;
    }
  }
}

/**********************************************************************/
// Main function for Euclidean clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Cluster_KDTree<PointT>::euclidean_cluster(typename pcl::PointCloud<PointT>::Ptr cloud) {
  KdTree* tree = new KdTree;

  // Build KD-Tree
  for (int i=0; i<num_points; i++)
    tree->insert(cloud->points[i], i);

  // Iterate through points to form clusters
  for (int i=0; i<num_points; i++) {

    // Check if the point has already been processed
    if (processed[i]) {
      i++;
      continue;
    }

    std::vector<int> cluster_index;
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    
    // Recursively find points in the cluster
    cluster_neighbors(cloud, tree, cluster_index, i);

    int cluster_size = cluster_index.size();

    // Check if the cluster size is within the specified limits
    if (cluster_size >= min_cluster_size && cluster_size <= max_cluster_size) {
      // Add points to the cluster
      for (int j=0; j<cluster_size; j++) {
        cloud_cluster->points.push_back(cloud->points[cluster_index[j]]);
      }

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;

      // Store the cluster
      clusters.push_back(cloud_cluster);
    }
  }

  // Return the clusters
  return clusters;
}