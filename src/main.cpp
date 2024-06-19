#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "symmetry.h"
#include "align.h"
//#include "blockmerge.h"
#include "clustering.h"
#include "regiongrowth.h"
#include "objective.h"
#include "resolve.h"
#include "multivec.h"
#include "random.h"

#include <sstream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

std::string toText(Direction direction)
{
	switch(direction)
	{
		case Direction::Left:
			return "Left";
		case Direction::Right:
			return "Right";
		case Direction::Up:
			return "Up";
		case Direction::Down:
			return "Down";
		case Direction::In:
			return "In";
		case Direction::Out:
			return "Out";
	}

	return "";
}

std::string toTextSide(Direction direction)
{
	switch(direction)
	{
		case Direction::Left:
			return "Left";
		case Direction::Right:
			return "Right";
		case Direction::Up:
			return "Top";
		case Direction::Down:
			return "Bottom";
		case Direction::In:
			return "Front";
		case Direction::Out:
			return "Back";
	}

	return "";
}

GridCell* getNearestBoundaryTo(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	const Cluster& cluster)
{
	K::Vector_3 d = cluster.centroid - grid.getOrigin();

	int offsetX = std::floor(d.x() / grid.getElementSize());
	int offsetY = std::floor(d.y() / grid.getElementSize());
	int offsetZ = std::floor(d.z() / grid.getElementSize());

	GridCell& targetCell = mv::get(gridCells, offsetX, offsetY, offsetZ);

	if(targetCell.type == GridCell::ContentType::Boundary)
		return std::addressof(targetCell);

	constexpr int MAX_SEARCH_RADIUS = 2;
	for(int radius = 1; radius < MAX_SEARCH_RADIUS; ++radius)
	{
		int xStart = std::max(offsetX - radius, 0);
		int xEnd = std::min(offsetX + radius, (int)grid.getNumBoxesX());
		for(int x = xStart; x < xEnd; ++x)
		{
			int yStart = std::max(offsetY - radius, 0);
			int yEnd = std::min(offsetY + radius, (int)grid.getNumBoxesY());
			for(int y = yStart; y < yEnd; ++y)
			{
				int zStart = std::max(offsetZ - radius, 0);
				int zEnd = std::min(offsetZ + radius, (int)grid.getNumBoxesZ());
				for(int z = zStart; z < zEnd; ++z)
				{
					GridCell& alterCell = mv::get(gridCells, x, y, z);

					if(alterCell.type == GridCell::ContentType::Boundary)
					{
						return std::addressof(alterCell);
					}
				}
			}
		}
	}

	return nullptr;
}

std::vector<std::unique_ptr<MeshBox>> getSourceMeshBoxesFrom(
	const std::vector<Cluster>& clusters, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells)
{
	std::vector<std::unique_ptr<MeshBox>> sourceMeshBoxes;

	for(const Cluster& cluster: clusters)
	{
		GridCell *result = getNearestBoundaryTo(grid, gridCells, cluster);
		if(result)
		{
			sourceMeshBoxes.emplace_back(std::make_unique<MeshBox>((MeshBox) {
				result->mesh,
				Cuboid(result->position, Vector3D(1, 1, 1)),
				{result}
			}));
			result->parents.push_back(sourceMeshBoxes.back().get());
		}
	}

	return sourceMeshBoxes;
}

void verifyClusters(
	const std::vector<Cluster>& clusters,
	const K::Point_3& min, const K::Point_3& max)
{
	for(const Cluster& cluster: clusters)
	{
		const K::Point_3& centroid = cluster.centroid;
		if(    centroid.x() > min.x() && centroid.x() < max.x()
			&& centroid.y() > min.y() && centroid.y() < max.y()
			&& centroid.z() > min.z() && centroid.z() < max.z())
		{}
		else 
			//throw std::runtime_error("Cluster centroid outside of mesh!");
		{
			std::cerr << "Cluster centroid (" << centroid << ") "
				      << "outside of mesh! (min: " << min << ", max: " << max << ")";
		}
	}
}

[[nodiscard]]
std::vector<Cluster> filterVerifiedClusters(
	const std::vector<Cluster>& clusters,
	const K::Point_3& min, const K::Point_3& max)
{
	std::vector<Cluster> filtered;

	for(const Cluster& cluster: clusters)
	{
		const K::Point_3& centroid = cluster.centroid;
		if(    centroid.x() > min.x() && centroid.x() < max.x()
			&& centroid.y() > min.y() && centroid.y() < max.y()
			&& centroid.z() > min.z() && centroid.z() < max.z())
		{
			filtered.push_back(cluster);
		}
	}

	return filtered;
}

void saveMeshes(const std::string& directory, const std::vector<const Mesh*>& meshes)
{
	unsigned int meshIndex = 0;
	for(const Mesh* mesh: meshes)
	{
		assert(mesh);

		std::stringstream ss("");
		ss << meshIndex << ".stl";

		if(CGAL::IO::write_STL(directory + "/" + ss.str(), *mesh))
			std::cout << "Saved " << ss.str() << std::endl;
		else
			std::cerr << "Failed to write " << ss.str() << "!" << std::endl;

		++meshIndex;
	}
}

void saveMeshes(const std::string& directory, const std::vector<Mesh>& meshes)
{
	std::vector<const Mesh*> meshPtrs;
	for(const Mesh& mesh: meshes)
		meshPtrs.push_back(std::addressof(mesh));
	saveMeshes(directory, meshPtrs);
}

void processSubMesh(
	const Config& config, 
	int subIndex,
	Mesh& mesh, 
	std::vector<Mesh>& out)
{
	std::stringstream strstrm("");
	strstrm << subIndex;
	std::string parentDirectory = config.outputDir + "/" + strstrm.str();

	recenter(mesh);

	std::cout << "Sub-mesh re-centered." << std::endl;
	
	alignMeshToGrid(mesh);

	K::Point_3 min, max;
	bounds(mesh, min, max);

	std::cout << "Min: " << min << ", Max: " << max << std::endl;

	min = min - K::Vector_3(1.0, 1.0, 1.0);
	max = max + K::Vector_3(1.0, 1.0, 1.0);

	auto fnormals = mesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(mesh, fnormals);

	std::cout << "Computed normals." << std::endl;

	Grid grid(min, max, config.granularityScale);

	mv::vector3<GridCell> gridCells;
    getSurfaceBoxes(mesh, grid, gridCells, config.numPrinters);

	std::vector<double> itrScores;

	std::vector<std::unique_ptr<MeshBox>> meshBoxes;
	for(unsigned int i = config.numPrinters; i > 1; --i)
	{
		double score = 0.0;
		std::vector<double> subitrScores;
		std::vector<int> emptyCounts;

		for(unsigned int j = 0; j < config.sampleTries; ++j)
		{
			double subscore = 0.0;

			resetGridCells(gridCells);

			std::vector<Cluster> clusters = getClusters(i, mesh);
			clusters = filterVerifiedClusters(clusters, min, max);
			meshBoxes = getSourceMeshBoxesFrom(clusters, grid, gridCells);

			enumerateConflicts(meshBoxes, gridCells);

			regionGrowth(config, mesh, meshBoxes, gridCells, grid);

			//resolveConflicts(mesh, meshBoxes, gridCells, grid);

			enumerateConflicts(meshBoxes, gridCells);

			size_t conflicts = 0;
			conflicts = mv::reduce<GridCell, size_t>([](size_t &acc, GridCell &cell) {
				if(cell.type == GridCell::ContentType::Boundary && cell.parents.size() > 1)
					++acc;
			}, gridCells, conflicts);

			std::vector<Cuboid> emptyRegions;
			size_t numEmptyRegions = getDiscreteEmptyRegions(grid, gridCells, emptyRegions);
			emptyCounts.push_back(numEmptyRegions);
			std::cout << numEmptyRegions << " empty regions." << std::endl;

			if(numEmptyRegions > (config.numPrinters - meshBoxes.size()))
			{
				std::cout << "\tInvalid!" << std::endl;
				subscore = -1.0;
			}
			else
			{
				std::cout << "\tSuccess!" << std::endl;
			}

			std::cout << "Final Conflict Count: " << conflicts << std::endl;

			std::vector<const Mesh*> mbPtrs;
			for(auto& mb: meshBoxes)
				mbPtrs.push_back(&mb->mesh);

			std::stringstream sis("");
			sis << i << "-" << j;
			std::string dirName = parentDirectory + "/itr" + sis.str();
			fs::create_directory(dirName);
			saveMeshes(dirName, mbPtrs);

			if(subscore >= 0.0)
				subscore = fullScore(config, mbPtrs);
			subitrScores.push_back(subscore);

			std::vector<MeshBox> emptyMeshBoxes;
			for(const Cuboid& region: emptyRegions)
			{
				emptyMeshBoxes.emplace_back(MeshBox(region));
				clipFromMesh(grid, mesh, emptyMeshBoxes.back());
				std::cout << "\t" << region << std::endl;
			}

			std::vector<const Mesh*> embPtrs;
			for(const MeshBox& mb: emptyMeshBoxes)
				embPtrs.push_back(&mb.mesh);

			std::string dirNameEmpties = dirName + "/empties";
			fs::create_directory(dirNameEmpties);
			saveMeshes(dirNameEmpties, embPtrs);
		}

		int bestIndex = -1;
		int bestEmptyCount = std::numeric_limits<int>::max();
		score = std::numeric_limits<int>::max();
		for(int index = 0; index < config.sampleTries; ++index) 
		{
			if(emptyCounts[index] < bestEmptyCount)
			{
				bestIndex = index;
				bestEmptyCount = emptyCounts[index];
				score = subitrScores[index];
			}
			else if(emptyCounts[index] == bestEmptyCount &&
					subitrScores[index] < score)
			{
				bestIndex = index;
				score = subitrScores[index];
			}
		}

		std::stringstream sis2("");
		sis2 << i << "-" << bestIndex;
		std::string srcDirName = parentDirectory + "/itr" + sis2.str();
		std::stringstream sis3("");
		sis3 << i;
		std::string targetDirName = parentDirectory + "/itr" + sis3.str();
		fs::rename(srcDirName, targetDirName);

		for(int index = 0; index < config.sampleTries; ++index)
		{
			std::stringstream ss("");
			ss << index;
			fs::remove_all(parentDirectory + "/itr" + ss.str());
		}

		itrScores.push_back(score);
	}
	
	int itrIndex = 0, bestIndex = -1;
	double bestScore = std::numeric_limits<double>::max();
	int numPrinters = config.numPrinters, bestNumPrinters;
	for(double score: itrScores)
	{
		std::cout << "Iteration " << itrIndex+1 << ": " << numPrinters 
			      << " printers => ";
		if(score < 0.0)
			std::cout << "Failed";
		else 
		{
			if(score < bestScore)
			{
				bestScore = score;
				bestIndex = itrIndex;
				bestNumPrinters = numPrinters;
			}
			std::cout << "Success [score = " << score << "]!";
		}
		std::cout << std::endl;
		itrIndex++;
		numPrinters--;
	}

	std::cout << "Best Iteration: " << bestIndex+1;
	std::cout << " (# printers: " << bestNumPrinters << ")" << std::endl;

	// Move correct folder to main
	std::stringstream sms("");
	sms << bestNumPrinters;
	std::string bestDir = parentDirectory + "/itr" + sms.str();

	fs::rename(bestDir, parentDirectory + "/best");

	for(auto& meshBox: meshBoxes)
		out.push_back(meshBox->mesh);
}

int run(int argc, const char* argv[])
{
	Arguments args(argc, argv);
	Config config = handleArguments(args);

	std::cout << std::endl;
	printConfig(config);
	std::cout << std::endl;

	initRandom(config.seed);

	std::chrono::time_point<std::chrono::steady_clock> startIval = 
		std::chrono::steady_clock::now();

	Mesh inputMesh;
	if(!PMP::IO::read_polygon_mesh(config.inputFile, inputMesh)) 
		throw std::runtime_error(config.inputFile + " could not be loaded!");

	if(CGAL::Polygon_mesh_processing::does_self_intersect(inputMesh))
		throw std::runtime_error("Mesh self-intersects!");

	std::cout << "Mesh loaded!" << std::endl;

	recenter(inputMesh);

	std::cout << "Mesh re-centered." << std::endl;

	std::vector<Mesh> subMeshes;
	Mesh rightMesh, leftMesh;
	if(!config.symmetrySkip &&
	   symmetrySplit(inputMesh, &rightMesh, &leftMesh))
	{
		subMeshes.push_back(rightMesh);
		subMeshes.push_back(leftMesh);
		std::cout << "Applied preemptive optimisation: symmetric partition." << std::endl;
	} else {
		subMeshes.push_back(inputMesh);
		std::cout << "No preemptive optimisation applied." << std::endl;
	}
	
	std::vector<std::vector<Mesh>> subMeshSubDivs;
	subMeshSubDivs.resize(subMeshes.size());

	fs::remove_all(config.outputDir);
	fs::create_directory(config.outputDir);

	for(unsigned int index = 0; index < subMeshSubDivs.size(); ++index)
	{
		std::stringstream ss("");
		ss << index;

		std::string dirName = config.outputDir + "/" + ss.str();
		fs::create_directory(dirName);

		processSubMesh(config, index, subMeshes[index], subMeshSubDivs[index]);
	}

	std::chrono::time_point<std::chrono::steady_clock> endIval =
		std::chrono::steady_clock::now();
	std::chrono::duration<double, std::milli> duration =
		endIval - startIval;

	std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;

	return EXIT_SUCCESS;
}

int main(int argc, const char* argv[])
{
#ifndef DEBUG
	try
	{
#endif
		return run(argc, argv);
#ifndef DEBUG
	}
	catch(const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
#endif
}

