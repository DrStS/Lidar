/* \author Stefan Sicklinger */
// 3d Tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void insert(Node *&node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			uint branch = depth % 3;
			if (point[branch] < node->point[branch])
			{
				insert(node->left, branch + 1, point, id);
			}
			else
			{
				insert(node->right, branch + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insert(root, 0, point, id);
	}

	void search(Node *&node, uint depth, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{

			float leftXBound  = target[0] - distanceTol;
			float rightXBound = target[0] + distanceTol;
			float upperYBound = target[1] + distanceTol;
			float lowerYBound = target[1] - distanceTol;
			float frontZBound = target[2] + distanceTol;
			float rearZBound  = target[2] - distanceTol;

			if ((node->point[0] > leftXBound && node->point[0] < rightXBound) && (node->point[1] > lowerYBound && node->point[1] < upperYBound) && (node->point[2] > rearZBound && node->point[2] < frontZBound))
			{
				//check for point within circle
				float distance = sqrt((target[0] - node->point[0]) * (target[0] - node->point[0]) + (target[1] - node->point[1]) * (target[1] - node->point[1])+ (target[2] - node->point[2]) * (target[2] - node->point[2]));

				if (distanceTol > distance)
				{ // we have a match
					ids.push_back(node->id);
					std::cout << "Node "<< node->id << "added" << std::endl;
				}
			}

			//leverage the tree
			if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
			{
				search(node->left, depth + 1, target, distanceTol, ids);
			}
			if ((target[depth % 3] + distanceTol) > node->point[depth % 3])
			{
				search(node->right, depth + 1, target, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, 0, target, distanceTol, ids);
		return ids;
	}
};
