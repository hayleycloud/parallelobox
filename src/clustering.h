#pragma once

#include "cgals.h"

struct Cluster
{
	K::Point_3 centroid;
	std::vector<int> vertices;
};

[[nodiscard]] std::vector<Cluster> getClusters(int k, const Mesh& mesh);

