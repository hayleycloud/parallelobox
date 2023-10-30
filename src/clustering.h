#pragma once

#include "cgals.h"

struct Cluster
{
	K::Point_3 centroid;
	std::vector<Mesh::Vertex_index> vertices;
};

[[nodiscard]] std::vector<Cluster> getClusters(
	int k, const Mesh& mesh, bool useKMeansPP = true);

