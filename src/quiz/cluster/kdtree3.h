/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);

	}

	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if(*node==NULL)
			*node = new Node(point,id);
		else
		{
			uint cd = depth % 3;
            std::vector<float> npoint = {(*node)->point.x, (*node)->point.y, (*node)->point.z};
            std::vector<float> tpoint = {point.x, point.y, point.z};
			if(tpoint[cd] < (npoint[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
			
	}
	// return a list of point ids in the tree that are within distance of target

	void searchHelper(pcl::PointXYZI target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if ((node->point.x>=target.x-distanceTol)&&(node->point.x<=target.x+distanceTol)&&(node->point.y>=target.y-distanceTol)&&(node->point.y<=target.y+distanceTol)&&(node->point.z>=target.z-distanceTol)&&(node->point.z<=target.z+distanceTol))
			{
				float distance = sqrt((node->point.x-target.x)*(node->point.x-target.x)+(node->point.y-target.y)*(node->point.y-target.y)+(node->point.z-target.z)*(node->point.z-target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
            std::vector<float> npoint = {node->point.x, node->point.y, node->point.z};
            std::vector<float> tpoint = {target.x, target.y, target.z};
			if (tpoint[depth%3]-distanceTol<npoint[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if (tpoint[depth%3]+distanceTol>npoint[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




