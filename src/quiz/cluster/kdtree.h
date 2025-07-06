/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left{};
	Node* right{};

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(Node*& current, int depth, std::vector<float> point, int id) {
		if (not current) {
			current = new Node(point, id);
			return;
		}

		if (depth % 3 == 0) {
			if (point[0] < current->point[0]) {
				insert(current->left, depth +1, point, id);
			} else {
				insert(current->right, depth +1, point, id);
			}

		} else if (depth % 3 == 1) {
			if (point[1] < current->point[1]) {
				insert(current->left, depth +1, point, id);
			} else {
				insert(current->right, depth +1, point, id);
			}
		} else {
			if (point[2] < current->point[2]) {
				insert(current->left, depth +1, point, id);
			} else {
				insert(current->right, depth +1, point, id);
			}
		}

	}

	void insert(std::vector<float> point, int id)
	{
		insert(root, 0, point, id);
	}



	void search(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids) {
		if (node == nullptr) {
			return;
		}

		if (node->point[0] < target[0] + distanceTol &&  node->point[0] > target[0] - distanceTol &&
			node->point[1] < target[1] + distanceTol &&  node->point[1] > target[1] - distanceTol &&
			node->point[2] < target[2] + distanceTol &&  node->point[2] > target[2] - distanceTol) {
			if ( std::sqrt(std::pow(node->point[0] - target[0] , 2) + std::pow(node->point[1] - target[1] , 2) + std::pow(node->point[2] - target[2] , 2)) < distanceTol) {
				ids.push_back(node->id);
			}

		}

		if (target[depth % 3] - distanceTol < node->point[depth % 3]) {
			search(node->left, depth + 1, target, distanceTol, ids);
		}

		if (target[depth % 3] + distanceTol > node->point[depth % 3] ) {
			search(node->right, depth + 1, target, distanceTol, ids);
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




