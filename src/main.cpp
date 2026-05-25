#include "reporting.h"
#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "align.h"
#include "symmetry.h"
#include "clustering.h"
#include "regiongrowth.h"
#include "fillempty.h"
#include "resolve.h"
#include "objective.h"
#include "multivec.h"
#include "random.h"

#include <sstream>
#include <fstream>
#include <filesystem>

#define EXPORT_SEED_BLOCKS	false
#define EXPORT_ALL_TRIES	false
#define EXPORT_BEST_TRIES	false
#define RESOLVE_EMPTIES		true

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

K::Vector_3 toVector(Direction direction)
{
	switch(direction)
	{
		case Direction::Left:
			return { -1, 0, 0 };
		case Direction::Right:                                          
			return { 1, 0, 0 };
		case Direction::Down:
			return { 0, -1, 0 };
		case Direction::Up:
			return { 0, 1, 0 };
		case Direction::Out:
			return { 0, 0, -1 };
		case Direction::In:
			return { 0, 0, 1 };
	}

	return { 0, 0, 0 };
}

K::Vector_3 normalize(const K::Vector_3& v)
{
	return v * (1.0 / CGAL::sqrt(v.squared_length()));
}

MeshErrorSet validate(Mesh& mesh, bool throwOnFail)
{
	MeshErrorSet errors = NO_MESH_ERRORS;

	auto fparts = mesh.add_property_map<face_descriptor, std::size_t>("f:part").first;
	int numSeparatedRegions = PMP::connected_components(mesh, fparts);
	mesh.remove_property_map(fparts);
	if(numSeparatedRegions > 1)
	{
		if(throwOnFail)
			throw std::runtime_error("Mesh is discontinuous!");
		SET_MESH_ERROR(errors, MeshErrors::NonManifold);
	}

	if(!PMP::triangulate_faces(mesh))
	{
		if(throwOnFail)
			throw std::runtime_error("Mesh not triangulated!");
		SET_MESH_ERROR(errors, MeshErrors::NonTriangular);
	}

	PMP::autorefine(mesh);

	if(PMP::does_self_intersect(mesh))
	{
		if(throwOnFail)
			throw std::runtime_error("Mesh self-intersects!");
		SET_MESH_ERROR(errors, MeshErrors::SelfIntersects);
	}

	return errors;
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

	constexpr int MAX_SEARCH_RADIUS = 4;
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

void assignMeshBoxesFrom(
	const Mesh& parent, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	const std::vector<Cuboid>& regions, 
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes)
{
	for(const Cuboid& region: regions)
	{
		Mesh mesh;
		if(INVALID(clipFromMesh(grid, parent, region, mesh)))
		{
			std::cerr << "Mesh clip error?" << std::endl;
		}

		std::vector<GridCell*> sampledCells;
		sampleCells(gridCells, region, sampledCells);

		std::unique_ptr<MeshBox> meshBox = 
			std::make_unique<MeshBox>(mesh, region, sampledCells);

		for(GridCell* cell: sampledCells)
		{
			if(cell->parents.size() > 0)
				std::cerr << "Already parented!?" << std::endl;
			cell->parents.push_back(meshBox.get());
		}

		meshBoxes.emplace_back(std::move(meshBox));
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
		{
#if REPORT_VERBOSE
			//std::cout << "Saved " << ss.str() << std::endl;
#endif
		}
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

void saveMeshes(const std::string& directory, const std::vector<const MeshBox*>& meshes)
{
	std::vector<const Mesh*> meshPtrs;
	for(const MeshBox* mesh: meshes)
		meshPtrs.push_back(std::addressof(mesh->mesh));
	saveMeshes(directory, meshPtrs);
}

void saveMeshes(const std::string& directory, const std::vector<std::unique_ptr<MeshBox>>& meshes)
{
	std::vector<const Mesh*> meshPtrs;
	for(const auto& mesh: meshes)
		meshPtrs.push_back(std::addressof(mesh->mesh));
	saveMeshes(directory, meshPtrs);
}

void saveIterationScores(
	const Config& config,
	const std::vector<std::vector<double>>& iterationScores,
	const std::vector<std::vector<std::vector<double>>>& subitrScores)
{
	std::ofstream outFile(config.outputDir + "/iteration_scores.csv");

	outFile << "Iterations:\r\n";

	int submesh = 1;
	for(const auto& iterations: iterationScores)
	{
		outFile << "\r\nSub-Mesh: " << submesh << "\r\n";

		int iteration = 1;
		for(double iterationScore: iterations)
		{
			outFile << iteration << "," << iterationScore << "\r\n";
			++iteration;
		}

		++submesh;
	}

	outFile << "\r\nSub-Iterations:\r\n";

	submesh = 1;
	for(const auto& smSubitrScores: subitrScores)
	{
		outFile << "Sub-Mesh: " << submesh << "\r\n";

		int iteration = 1;
		for(const auto& iterationScores: smSubitrScores)
		{
			outFile << "Iteration: " << iteration << "\r\n";
			int subiteration = 1;
			for(double subitrScore: iterationScores)
			{
				outFile << " ," << subitrScore << "\r\n";
				++subiteration;
			}
			++iteration;
		}

		++submesh;
	}
	
	outFile.close();
}

std::string getTypeAsStr(GridCell::ContentType type)
{
	switch(type)
	{
		case GridCell::ContentType::Boundary:
			return "Boundary";
		case GridCell::ContentType::Internal:
			return "Internal";
		case GridCell::ContentType::Empty:
			return "Empty";
	}
	return "";
}

double processSubMesh(
	const Config& config, 
	const std::string& directory,
	int subIndex, 
	int numPrinters, 
	Mesh& mesh,
	std::vector<double>& itrScores,
	std::vector<std::vector<double>>& outSubitrScores)
{
	std::string parentDirectory = directory;

	recenter(mesh);

#if REPORT_EXTRA
	std::cout << "Sub-mesh re-centered." << std::endl;
#endif
	
	alignMeshToGrid(mesh);

	K::Point_3 min, max;
	bounds(mesh, min, max);

#if REPORT_EXTRA
	std::cout << "Min: " << min << ", Max: " << max << std::endl;
#endif

	min = min - K::Vector_3(1.0, 1.0, 1.0);
	max = max + K::Vector_3(1.0, 1.0, 1.0);

	int minNumPrinters = calcMinNumPrinters(config, mesh);
	if(minNumPrinters > numPrinters)
	{
		std::cout << "Too few printers available for mesh of this size!" << std::endl;
		std::cout << "(" << minNumPrinters << " required, but only " 
				  << numPrinters << " available)" << std::endl;
		return -1.0;
	}

#if REPORT_STANDARD
	std::cout << "Minimum # of printers required: " << minNumPrinters << std::endl;
#endif

	auto fnormals = mesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(mesh, fnormals);

#if REPORT_EXTRA
	std::cout << "Computed normals." << std::endl;
#endif

	Grid grid(min, max, config.granularityScale);

	mv::vector3<GridCell> gridCells;
    getSurfaceBoxes(mesh, grid, gridCells, numPrinters);

	unsigned int numIterations = (numPrinters - minNumPrinters) + 1;
	itrScores.reserve(numIterations);
	outSubitrScores.reserve(numIterations);

	unsigned int skipTo = 0;

	for(unsigned int i = numPrinters; i >= minNumPrinters; --i)
	{
		double score = 0.0;
		std::vector<double> subitrScores;

#if REPORT_BARE
		std::cout << std::endl;

		std::cout << "Running with " << i << " boxes..." << std::endl;
#endif

		for(unsigned int j = 0; j < config.sampleTries; ++j)
		{
#if REPORT_STANDARD
			std::cout << "[Attempt " << j+1 << "...]" << std::endl;
#endif

			resetGridCells(gridCells);

			std::vector<Cluster> clusters = getClusters(i, mesh);
			clusters = filterVerifiedClusters(clusters, min, max);

			if(skipTo > 0 && i > skipTo)
			{
				subitrScores.push_back(-1.0);
#if REPORT_EXTRA
				std::cout << "Skipping." << std::endl;
				std::cout << subitrScores[i].size() << std::endl;
#endif
				continue;
			}

			std::vector<std::unique_ptr<MeshBox>> meshBoxes;
			meshBoxes = getSourceMeshBoxesFrom(clusters, grid, gridCells);

			std::stringstream sis("");
			sis << i << "-" << j;
			std::string dirName = parentDirectory + "/itr" + sis.str();
			fs::create_directory(dirName);

#if EXPORT_SEED_BLOCKS
			std::vector<Mesh> clusterMeshes;
			for(auto& mb: meshBoxes)
				clusterMeshes.emplace_back(mb->mesh);

			std::string clusterDir = dirName + "/seeds";
			fs::create_directory(clusterDir);
			saveMeshes(clusterDir, clusterMeshes);
#endif

			PrintingCostCache printCostCache;

			regionGrowth(config, mesh, meshBoxes, grid, gridCells, printCostCache);

			double parallelCost = parallelPrintingCost(config, grid, meshBoxes, printCostCache);

			//saveMeshes(dirName, mbPtrs);

			//std::cout << "Blocks Grown." << std::endl;
			int freePrinters = numPrinters - meshBoxes.size();

			std::vector<std::unique_ptr<MeshBox>> fillerMeshBoxes;
			bool modelCovered = fillEmptyBoundarySpaces(
				config, mesh, fillerMeshBoxes, parallelCost, grid, gridCells, printCostCache);

			bool noExcessPrinters = !fillerMeshBoxes.empty();

			parallelCost = parallelPrintingCost(config, grid, meshBoxes, printCostCache);


#if REPORT_STANDARD
			std::cout << "After Block Growth Phase: " << std::endl;
			std::cout << "\tParallel Printing Time = " << parallelCost << std::endl;
			std::cout << "\t# meshes: " << meshBoxes.size();
			std::cout << " | # voids: " << fillerMeshBoxes.size() << std::endl;
#endif

#if RESOLVE_EMPTIES == true
			for(auto& mb: fillerMeshBoxes)
				meshBoxes.push_back(std::move(mb));

			noExcessPrinters = resolveMeshExcess(
				config, 
				meshBoxes, 
				numPrinters, 
				parallelCost, false,
				grid, gridCells);

			parallelCost = parallelPrintingCost(config, grid, meshBoxes, printCostCache);
#if REPORT_STANDARD
			std::cout << "After Empty Resolution Phase: " << std::endl;
			std::cout << "\tParallel Printing Time = " << parallelCost << std::endl;
			std::cout << "\t# meshes: " << meshBoxes.size() << std::endl;
#endif
#endif

			saveMeshes(dirName, meshBoxes);

			if(modelCovered && noExcessPrinters)
			{
				subitrScores.push_back(parallelCost);
			}
			else {
				subitrScores.push_back(-parallelCost);
#if REPORT_STANDARD
				std::cout << "Attempt failed." << std::endl;
#endif
			}
		}

		outSubitrScores.push_back(subitrScores);

		int bestIndex = -1;
		double bestFailScore = std::numeric_limits<double>::lowest();
		score = std::numeric_limits<double>::max();
		for(int index = 0; index < config.sampleTries; ++index) 
		{
			if(subitrScores[index] <= 0.0)
			{
				if(subitrScores[index] > bestFailScore)
					bestFailScore = subitrScores[index];
				continue;
			}

			if(subitrScores[index] < score)
			{
				score = subitrScores[index];
				bestIndex = index;
			}
		}

		if(bestIndex >= 0)
		{
			std::stringstream sis2("");
			sis2 << i << "-" << bestIndex;
			std::string srcDirName = parentDirectory + "/itr" + sis2.str();
			std::stringstream sis3("");
			sis3 << i;
			std::string targetDirName = parentDirectory + "/itr" + sis3.str();
			fs::rename(srcDirName, targetDirName);
#if REPORT_STANDARD
			std::cout << "Selected attempt " << bestIndex+1 << " as best";
			std::cout << " (time: " << score << ")." << std::endl;
#endif
		}
#if REPORT_STANDARD	
		else {
			std::cout << "No attempt succeeded." << std::endl;
		}
#endif

#if EXPORT_ALL_TRIES == false
		for(int index = 0; index < config.sampleTries; ++index)
		{
			std::stringstream ss("");
			ss << i << "-" << index;
			fs::remove_all(parentDirectory + "/itr" + ss.str());
		}
#endif

		itrScores.push_back(bestIndex >= 0 ? score : bestFailScore);
	}

#if REPORT_STANDARD
	std::cout << std::endl;
#endif
	
	int itrIndex = 0, bestIndex = -1;
	double bestScore = std::numeric_limits<double>::max();
	int subNumPrinters = numPrinters, bestNumPrinters;
	for(double score: itrScores)
	{
#if REPORT_BARE
		std::cout << "Iteration " << itrIndex+1 << ": " << subNumPrinters 
			      << " printers => ";
#endif
		if(score <= 0.0)
		{
#if REPORT_BARE
			std::cout << "Failed";
#endif
		}
		else 
		{
			if(score < bestScore)
			{
				bestScore = score;
				bestIndex = itrIndex;
				bestNumPrinters = subNumPrinters;
			}
#if REPORT_BARE
			std::cout << "Success [score = " << score << "]!";
#endif
		}
#if REPORT_BARE
		std::cout << std::endl;
#endif
		itrIndex++;
		subNumPrinters--;
	}

	if(bestIndex < 0)
	{
#if REPORT_BARE
		std::cout << "No Valid Results Found!" << std::endl;
#endif
		return -1.0;
	}

#if REPORT_BARE
	std::cout << "Best Iteration: " << bestIndex+1;
	std::cout << " (# printers: " << bestNumPrinters << ")" << std::endl;
	std::cout << "Best Parallel Print Cost: " << bestScore << std::endl;
#endif

	// Move correct folder to main
	std::stringstream sms("");
	sms << bestNumPrinters;
	std::string bestDir = parentDirectory + "/itr" + sms.str();

	fs::rename(bestDir, parentDirectory + "/best");

#if EXPORT_BEST_TRIES == false
	for(int index = 0; index <= numPrinters; ++index)
	{
		std::stringstream ss("");
		ss << index;
		fs::remove_all(parentDirectory + "/itr" + ss.str());
	}
#endif

	return bestScore;
}

int run(int argc, const char* argv[])
{
	Arguments args(argc, argv);
	Config config = handleArguments(args);

#if REPORT_STANDARD
	std::cout << std::endl;
	printConfig(config);

	std::cout << "Seed: " << initRandom(config.seed) << " ";
	std::cout << (config.seed ? "(manual)" : "(automatic)") << std::endl;
	std::cout << std::endl;
#endif

	std::chrono::time_point<std::chrono::steady_clock> startIval = 
		std::chrono::steady_clock::now();

	Mesh inputMesh;
	if(!PMP::IO::read_polygon_mesh(config.inputFile, inputMesh)) 
		throw std::runtime_error(config.inputFile + " could not be loaded!");

	validate(inputMesh, true);

#if REPORT_STANDARD
	std::cout << "Mesh loaded!" << std::endl;
#endif

	recenter(inputMesh);

#if REPORT_EXTRA
	std::cout << "Mesh re-centered." << std::endl;
#endif

	bool symmetrySucceeded = true;
	std::array<int,2> numPrinters = { config.numPrinters, config.numPrinters };
	std::vector<Mesh> subMeshes;
	Mesh rightMesh, leftMesh;
	if(!config.symmetrySkip &&
	   (symmetrySucceeded = symmetrySplit(inputMesh, &rightMesh, &leftMesh)))
	{
		validate(rightMesh, true);
		validate(leftMesh, true);
		subMeshes.push_back(rightMesh);
		subMeshes.push_back(leftMesh);

		int most = (int) std::ceil(config.numPrinters / 2.0);
		int least = (int) std::floor(config.numPrinters / 2.0);
		
		if(PMP::volume(rightMesh) >= PMP::volume(leftMesh))
			numPrinters = { most, least };
		else
			numPrinters = { least, most };

#if REPORT_STANDARD
		std::cout << "Applied preemptive optimization: symmetric partition." << std::endl;
#endif
	} else {
		subMeshes.push_back(inputMesh);
#if REPORT_STANDARD
		if(!symmetrySucceeded)
			std::cout << "Could not find an appropriate symmetric partition for optimization." << std::endl;
		std::cout << "No preemptive optimization applied." << std::endl;
#endif
	}
	
	fs::remove_all(config.outputDir);
	fs::create_directory(config.outputDir);


	std::vector<std::vector<double>> itrScores(subMeshes.size());
	std::vector<std::vector<std::vector<double>>> subitrScores(subMeshes.size());

	double bestScoreTotal = -1.0;
	for(unsigned int index = 0; index < subMeshes.size(); ++index)
	{
		std::string dirName = config.outputDir + "/tries";
		fs::create_directory(dirName);

		if(subMeshes.size() >= 2)
		{
			std::stringstream ss("");
			ss << index;
			dirName += "/" + ss.str();
			fs::create_directory(dirName);
		}

		std::cout << "Processing Mesh " << (index + 1) << "/" << subMeshes.size() << std::endl;
		std::cout << "Number of printers provided " << numPrinters[index] << std::endl;

		double bestScore = processSubMesh(
			config, 
			dirName,
			index, 
			numPrinters[index], 
			subMeshes[index],
			itrScores[index],
			subitrScores[index]);
		if(bestScore <= 0.0)
		{
			bestScoreTotal = -1.0;
#if REPORT_STANDARD
			std::cout << "Sub-mesh pass failed! Dropping out." << std::endl;
#endif
			break;
		}
		
		bestScoreTotal = bestScore > bestScoreTotal ? bestScore : bestScoreTotal;
	}

	std::cout << std::endl;
	if(bestScoreTotal > 0.0)
	{
		std::cout << "Decomposed with parallel print time: " << bestScoreTotal << std::endl;

		if(subMeshes.size() >= 2)
		{
			for(unsigned int index = 0; index < subMeshes.size(); ++index)
			{
				std::string dirName = config.outputDir + "/tries";

				std::string indexDir = "/";
				std::stringstream ss("");
				ss << index;
				indexDir += ss.str();

				fs::rename(
					dirName + indexDir + "/best", 
					config.outputDir + indexDir);
			}

			if(config.cleanupOutDirAfter)
			{
				fs::remove_all(config.outputDir + "/tries");
			}
		}
		else
		{
			fs::rename(
				config.outputDir + "/tries/best", config.outputDir + "/best");

			if(config.cleanupOutDirAfter)
			{
				//fs::remove_all(config.outputDir + "/tries");

				std::string tmpDir = config.outputDir + "_tmp";
				fs::rename(config.outputDir, tmpDir);
				fs::rename(tmpDir + "/best", config.outputDir);

				fs::remove_all(tmpDir);
			}
		}

	}
	else
	{
		std::cout << "Failed to produce a valid decomposition." << std::endl;
	}

	saveIterationScores(config, itrScores, subitrScores);

	std::chrono::time_point<std::chrono::steady_clock> endIval =
		std::chrono::steady_clock::now();
	std::chrono::duration<double, std::milli> duration =
		endIval - startIval;

#if REPORT_STANDARD
	std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
#endif

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

 
