#include <pcl/common/common.h>
#include <vector>

/**
 * @brief Implementation of KD-Tree for 3D point clouds.
 * 
 * This class uses the KD-Tree method to search for the nearest neighbors of a
 *
 * Author: Morsinaldo Medeiros
 * Date: 2023-12-28
 * 
 * Implementation reference: https://github.com/MohamedHussein736/Lidar-Obstacle-Detection/blob/master/src/kdtree_pcl.h
 * Github Copilot was used to help with the implementation.
 * 
 */

// Node structure
struct Node {
    int id; 				// Id of the point
    pcl::PointXYZI point;	// Point
    Node* left;				// Left child
    Node* right;			// Right child

    // Constructor for Node
    Node(pcl::PointXYZI& array, int set_id)
        : point(array), id(set_id), left(nullptr), right(nullptr) {}
};

// KD-Tree structure
struct KdTree
{
    // constructor
	Node* root;
	KdTree() : root(nullptr){}

    /**
     * @brief Inserts a new point into the tree.
     * 
     * @param point Point to be inserted.
     * @param id Id of the point to be inserted.
     * 
     * @return void
     *
     * @details This method creates a new node and places it correctly within the root.
    */
	void insert_node(Node **node, pcl::PointXYZI point, int id, int depth) {
		if (*node != nullptr) {

            // Choose the next node based on the comparison result
			if (depth % 3 == 0) {
				if (point.x < (*node)->point.x){
					insert_node(&((*node)->left), point, id, depth+1);
				} else {
					insert_node(&((*node)->right), point, id, depth+1);
				}
			}
			else if (depth % 3 == 1) {
				if (point.y < (*node)->point.y){
					insert_node(&((*node)->left), point, id, depth+1);
				} else {
					insert_node(&((*node)->right), point, id, depth+1);
				}
			}
			else {
				if (point.z < (*node)->point.z) {
					insert_node(&((*node)->left), point, id, depth+1);
				} else {
					insert_node(&((*node)->right), point, id, depth+1);
				}
			}
		}
		else {
			// Create a new node if the current node is null
			*node = new Node(point, id);
		}
	}

    /************************************************/
    /**
     * @brief Inserts a new point into the tree.
     * 
     * @param point Point to be inserted.
     * @param id Id of the point to be inserted.
     * 
     * @return void
    */
	void insert(pcl::PointXYZI point, int id)
	{
		int depth = 0;
		// the function should create a new node and place correctly with in the root
		insert_node(&root, point, id, depth);
	}

	/************************************************/
    /**
     * @brief Searches for the nearest neighbors of a given point.
     * 
     * @param ids Vector of ids of the nearest neighbors.
     * @param target Point to be searched.
     * @param distance_tolerance Distance tolerance.
     * @param parent_node Current node.
     * @param depth Current depth.
     * 
     * @return void
     *
     * @details This method searches for the nearest neighbors of a given point.
    */
	void search_neighbors(std::vector<int> &ids, pcl::PointXYZI target,
				float distance_tolerance, Node* parent_node, int depth) {
        
        // Check if the current node is null
		if (parent_node != nullptr) {
			float delta_x = parent_node->point.x - target.x;
			float delta_y = parent_node->point.y - target.y;
			float delta_z = parent_node->point.z - target.z;
            
            // Check if the current node is within the box
			if ((-distance_tolerance <= delta_x && distance_tolerance >= delta_x) &&
								(-distance_tolerance <= delta_y && distance_tolerance >= delta_y) &&
									(-distance_tolerance <= delta_z && distance_tolerance >= delta_z)) {
				float distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
				// check distance
				if (distance <= distance_tolerance) {
					ids.push_back(parent_node->id);
				}
			}

            // Check across boundary
			if (depth % 3 == 0) // 3 dim kd-tree - x-axis
			{
				// check across boundary
				if (-distance_tolerance < delta_x){
					search_neighbors(ids, target, distance_tolerance, parent_node->left, depth+1);
				}
				if (distance_tolerance > delta_x) {
					search_neighbors(ids, target, distance_tolerance, parent_node->right, depth+1);
				}
			}
			else if (depth % 3 == 1) // y-axis
			{
				// check across boundary
				if (-distance_tolerance < delta_y) {
					search_neighbors(ids, target, distance_tolerance, parent_node->left, depth+1);
				}
				if (distance_tolerance > delta_y) {
					search_neighbors(ids, target, distance_tolerance, parent_node->right, depth+1);
				}
			}
			else // z-axis
			{
				if (-distance_tolerance < delta_z) {
					search_neighbors(ids, target, distance_tolerance, parent_node->left, depth+1);
				}
				if (distance_tolerance > delta_z) {
					search_neighbors(ids, target, distance_tolerance, parent_node->right, depth+1);
				}
			}
		} else {
			// if the current node is null,
			return;
		}
	}
	/************************************************/

	/**
     * @brief Searches for the nearest neighbors of a given point.
     * 
     * @param target Point to be searched.
     * @param distance_tolerance Distance tolerance.
     * 
     * @return Vector of ids of the nearest neighbors.
    */
	std::vector<int> search(pcl::PointXYZI target, float distance_tolerance) {
		// return a list of point ids in the tree that are within distance of target
		std::vector<int> point_ids;
		search_neighbors(point_ids, target, distance_tolerance, root, 0);
		return point_ids;
	}
};