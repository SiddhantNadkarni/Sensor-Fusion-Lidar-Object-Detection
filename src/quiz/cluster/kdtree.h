/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

// Structure to represent node of kd tree

template<typename PointT>
class Node
{
public:
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

template<typename PointT>
class KdTree
{
public:
	Node<PointT>* root;

	KdTree()
	: root(nullptr)
	{}

	void insertHelper(Node<PointT>** node, int depth, PointT point, int id)
	{
		if(*node == nullptr)
		{
			*node = new Node<PointT>(point, id);
		}

		else
		{
			uint cd = depth%3;
			if (cd == 0)
			{
				if(point.x < (*node)->point.x)
					insertHelper(&(*node)->left, depth + 1, point, id);
				else
					insertHelper(&(*node)->right, depth + 1, point, id);
			}
			else
			{
				if(point.y < (*node)->point.y)
					insertHelper(&((*node)->left), depth+1, point, id);
			    else
			    	insertHelper(&((*node)->right), depth+1, point, id);
			}
			
		}
	}

	void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	// void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	// {
	// 	if(node!=nullptr)
	// 	{
	// 		if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)))
	// 		{
	// 			float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
	// 			if(distanceTol >= distance)
	// 				ids.push_back(node->id);
	// 		}
		

	// 		//if we want to flow left or right by checking box boundary
	// 		if ((target[depth%2] - distanceTol) < node->point[depth%2])
	// 		 {
	// 		 	searchHelper(target, node->left, depth+1, distanceTol, ids);
	// 		 } 

	// 		if ((target[depth%2] + distanceTol) > node->point[depth%2])
	// 		{
	// 			searchHelper(target, node->right, depth + 1, distanceTol, ids);
	// 		}
	// 	}
	// }

	void searchHelper(PointT pivot, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if ((node->point.x >= (pivot.x - distanceTol) && (node->point.x <= (pivot.x + distanceTol))) && (node->point.y >= (pivot.y - distanceTol) && (node->point.y <= (pivot.y + distanceTol))))
			{
				float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));

				if (distance <= distanceTol) 
					ids.push_back(node->id);
			}
			if (depth % 3 == 0) // 3 dim kd-tree
			{
				if ((pivot.x - distanceTol) < node->point.x) 
					searchHelper(pivot, node->left, depth + 1, distanceTol, ids);

				if ((pivot.x + distanceTol) > node->point.x) 
					searchHelper(pivot, node->right, depth + 1, distanceTol, ids);
			}
			else 
			{
				if ((pivot.y - distanceTol) < node->point.y) 
					searchHelper(pivot, node->left, depth + 1, distanceTol, ids);
				if ((pivot.y + distanceTol) > node->point.y) 
					searchHelper(pivot, node->right, depth + 1, distanceTol, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




