#include "clustering.h"

#include "kdtree.h"


void proximity(const std::vector<std::vector<float>>& points,
                      int id,
                      std::vector<int>& cluster,
                      std::vector<bool>& processed,
                      KdTree* tree,
                      float distanceTol)
{
    processed[id] = true;
    cluster.push_back(id);

    const auto nearby = tree->search(points[id], distanceTol);
    for (const int id : nearby) {
		if (processed[id]) {
			continue;
		}
		proximity(points, id, cluster, processed, tree, distanceTol);
	}
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	for (int i{0}; i < points.size(); ++i) {
		if (processed[i]) {
			continue;
		}

		std::vector<int> cluster;
		proximity(points, i, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}

	return clusters;
}

