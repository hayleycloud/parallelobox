#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "symmetry.h"
#include "align.h"
//#include "blockmerge.h"
#include "clustering.h"
#include "regiongrowth.h"
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

void processSubMesh(const Config& config, Mesh& mesh, std::vector<Mesh>& out)
{
	recenter(mesh);

	std::cout << "Sub-mesh re-centered." << std::endl;
	
	alignMeshToGrid(mesh);

	//out.push_back(mesh);

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

	/*unsigned int mIndex = 0;
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary)
		{
			std::stringstream ss("");
			ss << mIndex << ".stl";

			CGAL::IO::write_STL("out/" + ss.str(), cell.mesh);

			++mIndex;
		}
	}, gridCells);*/

	//CGAL::IO::write_STL("out/mesh.stl", mesh);

	std::vector<Cluster> clusters = getClusters(config.numPrinters, mesh);
	verifyClusters(clusters, min, max);
	std::vector<std::unique_ptr<MeshBox>> meshBoxes = 
		getSourceMeshBoxesFrom(clusters, grid, gridCells);

	enumerateConflicts(meshBoxes, gridCells);

	regionGrowth(config, mesh, meshBoxes, gridCells, grid);

	//resolveConflicts(mesh, meshBoxes, gridCells, grid);

	enumerateConflicts(meshBoxes, gridCells);

	size_t conflicts = 0;
	conflicts = mv::reduce<GridCell,size_t>([](size_t& acc, GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.size() > 1)
			++acc;
	}, gridCells, conflicts);

	std::cout << "Final Conflict Count: " << conflicts << std::endl;

	size_t empties = 0;
	empties = mv::reduce<GridCell,size_t>([](size_t& acc, GridCell& cell) {
		if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
			++acc;
	}, gridCells, empties);

	std::vector<const Mesh*> emptyCells;
	emptyCells = mv::reduce<GridCell,std::vector<const Mesh*>>(
		[](std::vector<const Mesh*>& list, GridCell& cell) {
			if(cell.type == GridCell::ContentType::Boundary && cell.parents.empty())
			{
				list.push_back(std::addressof(cell.mesh));
			}
	}, gridCells, emptyCells);

	fs::create_directory(config.outputDir + "/empties");
	saveMeshes(config.outputDir + "/empties", emptyCells);

	std::cout << "Final Empty Boundary Cell Count: " << empties << std::endl;

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
		processSubMesh(config, subMeshes[index], subMeshSubDivs[index]);


	size_t meshIndex = 0;
	for(const auto& subMesh: subMeshSubDivs)
	{
		saveMeshes(config.outputDir, subMesh);
	}

	return EXIT_SUCCESS;

	/*std::vector<Mesh> subdivs;
	grid.clipVolume(inputMesh, subdivs);*/


	//std::ofstream("out.off") << inputMesh;
	//if(CGAL::IO::write_STL(config.outputDir + "/" + "left.stl", leftCube) &&
	//   CGAL::IO::write_STL(config.outputDir + "/" + "right.stl", rightCube))
	//	std::cout << "Mesh re-saved!" << std::endl;
	//else
	//	throw std::runtime_error("Could not save mesh!");

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

