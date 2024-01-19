#include <unordered_set>
#include <pcl/common/common.h>

/**
 * @brief Implementation of RANSAC for detecting inliers in 3D point clouds.
 * 
 * This class uses the RANSAC method to identify inliers belonging to a
 * plane in a 3D point cloud.
 *
 * Author: Morsinaldo Medeiros
 * Date: 2023-12-28
 * 
 * Implementation reference: https://github.com/MohamedHussein736/Lidar-Obstacle-Detection/blob/master/src/ransac.cpp
 * Github Copilot was used to help with the implementation.
 *
 * @tparam PointT Type of points in the point cloud.
 */
template<typename PointT>
class Ransac {
private:
  int max_iterations;  ///< Maximum number of RANSAC iterations.
  float distance_tol;  ///< Distance tolerance to consider a point as an inlier.
  int num_points;      ///< Total number of points in the point cloud.


  /**
   * @brief Calculates the coefficients of the plane equation.
   * 
   * @param cloud 3D point cloud.
   * @param inliers Set of indices representing the inliers.
   * @param a Coefficient a of the plane equation.
   * @param b Coefficient b of the plane equation.
   * @param c Coefficient c of the plane equation.
   * @param d Coefficient d of the plane equation.
   * @param sqrt_abc Square root of the sum of the squares of coefficients a, b and c.
   * 
   * @return void
  */
  void calculatePlaneCoefficients(typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int> inliers, float *a, float *b, float *c, float *d, float *sqrt_abc){
    // Extract coordinates of the 3 chosen points
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto index = inliers.begin();
    x1 = cloud->points[*index].x;
    y1 = cloud->points[*index].y;
    z1 = cloud->points[*index].z;
    index++;
    x2 = cloud->points[*index].x;
    y2 = cloud->points[*index].y;
    z2 = cloud->points[*index].z;
    index++;
    x3 = cloud->points[*index].x;
    y3 = cloud->points[*index].y;
    z3 = cloud->points[*index].z;

    // Plane equation coefficients
    *a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    *b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    *c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    *d = -(*a * x1 + *b * y1 + *c * z1);
    *sqrt_abc = sqrt(*a * *a + *b * *b + *c * *c);
  }

public:
  /**
   * @brief Constructor of the Ransac class.
   * 
   * @param max_iter Maximum number of RANSAC iterations.
   * @param dist_tol Distance tolerance to consider a point as an inlier.
   * @param n_pts Total number of points in the point cloud.
   */
  Ransac(int max_iter, float dist_tol, int n_pts) : max_iterations(max_iter), distance_tol(dist_tol), num_points(n_pts) {}

  /**
   * @brief Destructor of the Ransac class.
   */
  ~Ransac();

  /**
   * @brief Executes the RANSAC algorithm to find inliers in a 3D point cloud.
   * 
   * @param cloud 3D point cloud.
   * @return A set of indices representing the found inliers.
   */
  std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud);
};

/**********************************************************************/
template<typename PointT>
Ransac<PointT>::~Ransac() {} // Destructor

/**********************************************************************/
/**
 * @brief Executes the RANSAC algorithm to find inliers in a 3D point cloud.
 * 
 * @param cloud 3D point cloud.
 * 
 * @return A set of indices representing the found inliers.
*/
template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::unordered_set<int> inliers_result;

  auto all_points = cloud->points;

  // Iterate over all RANSAC iterations
  while (max_iterations--) {
    std::unordered_set<int> inliers;

    // Randomly choose 3 points to define a plane
    while (inliers.size() < 3) {
      inliers.insert(rand() % num_points);
    }

    // Plane equation coefficients
    float a, b, c, d, sqrt_abc;
    calculatePlaneCoefficients(cloud, inliers, &a, &b, &c, &d, &sqrt_abc);

    // Check the distance from all points to the plane
    for (int i = 0; i < num_points; i++) {
      if (inliers.count(i) > 0) {
        continue;
      }
      PointT point_t = all_points[i];
      float distance = fabs(a * point_t.x + b * point_t.y + c * point_t.z + d) / sqrt_abc;

      if (distance <= distance_tol) {
        inliers.insert(i);
      }

      // Update inliers set if necessary
      if (inliers.size() > inliers_result.size()) {
        inliers_result = inliers;
      }
    }
  }

  return inliers_result;
}