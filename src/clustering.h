#pragma once

#include "cgals.h"

struct Cluster
{
	K::Point_3 centroid;
	std::vector<Mesh::Vertex_index> vertices;
};

[[nodiscard]] std::vector<Cluster> getClusters(
	int k, const Mesh& mesh, bool useKMeansPP = true);

void verifyClusters(
	const std::vector<Cluster>& clusters,
	const K::Point_3& min, const K::Point_3& max);

[[nodiscard]]
std::vector<Cluster> filterVerifiedClusters(
	const std::vector<Cluster>& clusters,
	const K::Point_3& min, const K::Point_3& max);

 
