#include "clustering.h"
#include "random.h"

void printClusters(const std::vector<Cluster>& clusters)
{
	for(const Cluster& cluster: clusters)
		std::cout << '\t' << cluster.centroid << std::endl;
}

bool indexAlreadyExists(unsigned int index, const std::vector<unsigned int>& indices)
{
	return std::find(indices.begin(), indices.end(), index) != indices.end();
}

[[nodiscard]] double l2NormFast(const K::Point_3& p1, const K::Point_3& p2)
{
	const K::Vector_3 dp(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
	return (dp.x() * dp.x()) + (dp.y() * dp.y()) + (dp.z() * dp.z());
}

[[nodiscard]] double l2Norm(const K::Point_3& p1, const K::Point_3& p2)
{
	const K::Vector_3 dp(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
	return std::sqrt((dp.x() * dp.x()) + (dp.y() * dp.y()) + (dp.z() * dp.z()));
}

[[nodiscard]] std::vector<Cluster> getInitialCentroids(unsigned int k, const Mesh& mesh)
{
	std::vector<Cluster> clusters;
	clusters.reserve(k);

	std::vector<unsigned int> chosenIndices;
	RNGInt<unsigned int> rng = makeRNGInt<unsigned int>(0, mesh.number_of_vertices());

	for(int i = 0; i < k; )
	{
		unsigned int randomIndex = fetchRandom(rng);

		if(indexAlreadyExists(randomIndex, chosenIndices))
			continue;
		chosenIndices.push_back(randomIndex);

		clusters.push_back((Cluster) {
			mesh.point(Mesh::Vertex_index(randomIndex)), {} });

		++i;
	}

	return clusters;
}

[[nodiscard]]
std::vector<Cluster> getInitialCentroidsKMeansPP(unsigned int k, const Mesh& mesh)
{
	RNGInt<unsigned int> rngInt = makeRNGInt<unsigned int>(0, k);

    std::vector<K::Point_3> centroids;
	centroids.reserve(k);

	unsigned int initialVertex = fetchRandom(rngInt);
    centroids.push_back(mesh.point(Mesh::Vertex_index(initialVertex)));

    while(centroids.size() < k)
    {
        std::vector<double> distances;
		distances.reserve(mesh.number_of_vertices());

	    for(Mesh::Vertex_index vIndex: mesh.vertices())
	    {
		    const K::Point_3& p = mesh.point(vIndex);
			double minDistance = std::numeric_limits<double>::max();

			for(const K::Point_3& centroid: centroids)
			{
				double d = l2Norm(p, centroid);
				minDistance = std::min(minDistance, d);
			}

			distances.push_back(minDistance);
		}

        // Pick the next centroid proportional to the squared distance
        std::discrete_distribution<unsigned int>
            distribution(distances.begin(), distances.end());
        unsigned int index = distribution(randEng);
        centroids.push_back(mesh.point(Mesh::Vertex_index(index)));
    }

	std::vector<Cluster> clusters;
	clusters.reserve(k);
	for(const K::Point_3& centroid: centroids)
		clusters.push_back((Cluster) { centroid, {} });

    return clusters;
}

void assignVerticesToClusters(const Mesh& mesh, std::vector<Cluster>& clusters)
{
	for(Mesh::Vertex_index vIndex: mesh.vertices())
	{
		const K::Point_3& p = mesh.point(vIndex);

		double bestL2Norm = std::numeric_limits<double>::max();
		Cluster* bestCluster = nullptr;

		for(Cluster& cluster: clusters)
		{
			double l2n = l2NormFast(p, cluster.centroid);
			if(l2n < bestL2Norm)
			{
				bestL2Norm = l2n;
				bestCluster = std::addressof(cluster);
			}
		}

		assert(bestCluster);
		bestCluster->vertices.push_back(vIndex);
	}
}

[[nodiscard]] 
K::Point_3 getClusterMeanPos(const Mesh& mesh, const Cluster& cluster)
{
	const double numIndices = static_cast<double>(cluster.vertices.size());

	double xSum = 0.0, ySum = 0.0, zSum = 0.0;
	for(Mesh::Vertex_index index: cluster.vertices)
	{
		const K::Point_3& p = mesh.point(index);
		xSum += p.x();
		ySum += p.y();
		zSum += p.z();
	}

	return K::Point_3(xSum / numIndices, ySum / numIndices, zSum / numIndices);
}

[[nodiscard]] std::vector<K::Point_3> 
getClusterMeanPositions(const Mesh& mesh, const std::vector<Cluster>& clusters)
{
	std::vector<K::Point_3> means;

	for(const Cluster& cluster: clusters)
		means.push_back(getClusterMeanPos(mesh, cluster));

	return means;
}

// True = identical, false = there are changes
[[nodiscard]] 
bool compareClusterSets(
	const std::vector<Cluster>& clusters1, 
	const std::vector<Cluster>& clusters2)
{
	// Mesh doesn't change ergo indices are guaranteed linear ordering
	for(int i = 0; i < clusters1.size(); ++i)
	{
		const Cluster& cluster1 = clusters1.at(i);
		const Cluster& cluster2 = clusters2.at(i);
		for(int j = 0; j < cluster1.vertices.size(); ++j)
		{
			if(cluster1.vertices.at(j) != cluster2.vertices.at(j))
				return false;
		}
	}

	return true;
}

std::vector<Cluster> getClusters(int k, const Mesh& mesh, bool useKMeansPP)
{
	std::vector<Cluster> clusters = useKMeansPP
		? getInitialCentroidsKMeansPP(k, mesh)
		: getInitialCentroids(k, mesh);
	assignVerticesToClusters(mesh, clusters);

	std::cout << "Initial cluster centroids ";
	std::cout << '(' << (useKMeansPP ? "k-means++" : "random") << "):";
	std::cout << std::endl;
	printClusters(clusters);

	int itrNum = 1;
	bool converged = false;
	while(!converged)
	{
		//std::cout << "k-Means Iteration " << itrNum << std::endl;
		//printClusters(clusters);

		std::vector<K::Point_3> meanPositions = 
			getClusterMeanPositions(mesh, clusters);

		std::vector<Cluster> newClusters;
		for(const K::Point_3& mean: meanPositions)
			newClusters.emplace_back((Cluster) { mean, {} });

		assignVerticesToClusters(mesh, newClusters);

		converged = compareClusterSets(clusters, newClusters);
		clusters = newClusters;

		++itrNum;
	}

	std::cout << "k-Means Iteration " << itrNum+1 << std::endl;
	printClusters(clusters);

	return clusters;
}

