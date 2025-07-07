#pragma once

#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <random>

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, cloud->points.size());
	for (int i = 0; i < maxIterations; ++i) {

		const auto x_1 = cloud->points[distrib(gen)];
		const auto x_2 = cloud->points[distrib(gen)];
		const auto x_3 = cloud->points[distrib(gen)];

		const auto a = (x_2.y - x_1.y) * (x_3.z - x_1.z) - (x_2.z - x_1.z) * (x_3.y - x_1.y);
		const auto b = (x_2.z - x_1.z) * (x_3.x - x_1.x) - (x_2.x - x_1.x) * (x_3.z - x_1.z);
		const auto c = (x_2.x - x_1.x) * (x_3.y - x_1.y) - (x_2.y - x_1.y) * (x_3.x - x_1.x);
		const auto d = -(a*x_1.x + b*x_1.y + c*x_1.z);
		const auto base = std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));

		std::unordered_set<int> inliersTmp;
		for (int j = 0; j < cloud->points.size(); ++j) {
			auto distance = std::fabs(a * cloud->points[j].x + b * cloud->points[j].y + c * cloud->points[j].z + d) / base;
			if (distance < distanceTol) {
				inliersTmp.insert(j);
			}
		}

		if (inliersTmp.size() > inliersResult.size()) {
			inliersResult = inliersTmp;
		}

	}
	return inliersResult;
}