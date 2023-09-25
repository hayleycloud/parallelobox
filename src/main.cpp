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

#include <sstream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

std::vector<std::unique_ptr<MeshBox>> getSourceMeshBoxesFrom(
	const std::vector<Cluster>& clusters, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells)
{
	std::vector<std::unique_ptr<MeshBox>> sourceMeshBoxes;

	for(const Cluster& cluster: clusters)
	{
		K::Vector_3 d = cluster.centroid - grid.getOrigin();

		int offsetX = std::floor(d.x() / grid.getElementSize());
		int offsetY = std::floor(d.y() / grid.getElementSize());
		int offsetZ = std::floor(d.z() / grid.getElementSize());

		GridCell& targetCell = mv::get(gridCells, offsetX, offsetY, offsetZ);
		sourceMeshBoxes.emplace_back(std::make_unique<MeshBox>((MeshBox){
			targetCell.mesh, 
			Cuboid(targetCell.position, Vector3D(1, 1, 1)),
			{ std::addressof(targetCell) }
		}));
		targetCell.parents.push_back(sourceMeshBoxes.back().get());
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

	/*mv::forEach<MeshBox>([](MeshBox& box) {
		std::string type = "";
		switch(box.type)
		{
			case MeshBox::ContentType::Internal:
				type += "Internal";
				break;
			case MeshBox::ContentType::Boundary:
				type += "Boundary";
				break;
			case MeshBox::ContentType::Empty:
				type += "Empty";
				break;
		}

		std::cout << type << ": " << box.dims << std::endl;
	}, meshBoxes);*/

	/*MeshBoxList boundaryMeshBoxes = mv::reduce<MeshBox,MeshBoxList>(
		[](MeshBoxList& out, MeshBox& in) {
			if(in.type == MeshBox::ContentType::Boundary)
				out.push_back(std::addressof(in));
	}, meshBoxes);*/

	/*const std::array<K::Vector_3,6> cardinalVecs = {
		K::Vector_3( 0,  1,  0),
		K::Vector_3( 0, -1,  0),
		K::Vector_3(-1,  0,  0),
		K::Vector_3( 1,  0,  0),
		K::Vector_3( 0,  0,  1),
		K::Vector_3( 0,  0, -1)
	};
	
	for(auto item = meshBoxes.begin(); item != meshBoxes.end(); ++item)
	{
		MeshBox& meshBox = **item;
		Mesh& mesh = meshBox.mesh;

		auto fnormals2 = mesh.add_property_map<face_descriptor, K::Vector_3>(
			"f:normals", CGAL::NULL_VECTOR).first;
		PMP::compute_face_normals(mesh, fnormals2);

		double bestOverhangArea = std::numeric_limits<double>::max();
		K::Vector_3 bestOverhang;
		for(const K::Vector_3& vec: cardinalVecs)
		{
			double ohArea = overhangArea(config, mesh, fnormals2, vec);
			if(ohArea < bestOverhangArea)
			{
				bestOverhangArea = ohArea;
				bestOverhang = vec;
			}
		}

		//std::cout << bestOverhangArea << " of overhang." << std::endl;
	}*/

	std::vector<Cluster> clusters = getClusters(config.numPrinters, mesh);
	verifyClusters(clusters, min, max);
	std::vector<std::unique_ptr<MeshBox>> meshBoxes = 
		getSourceMeshBoxesFrom(clusters, grid, gridCells);

	regionGrowth(mesh, meshBoxes, gridCells, grid);

	resolveConflicts(mesh, meshBoxes, gridCells, grid);

	/*mv::vector3<MeshBox*> meshBoxRefs = mv::map<MeshBox,MeshBox*>(
		[](MeshBox& meshBox) -> MeshBox* {
			if(meshBox.type == MeshBox::ContentType::Empty)
				return nullptr;
			else
				return &meshBox;
	}, meshBoxes);*/

	//mergeIterate(config, mesh, grid, gridCells, meshBoxes);

	//for(auto& meshBox: meshBoxes)
	//	out.push_back(meshBox->mesh);
}

int run(int argc, const char* argv[])
{
	Arguments args(argc, argv);
	Config config = handleArguments(args);

	std::cout << std::endl;
	printConfig(config);
	std::cout << std::endl;

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
	if(symmetrySplit(inputMesh, &rightMesh, &leftMesh))
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
	 
	for(unsigned int index = 0; index < subMeshSubDivs.size(); ++index)
		processSubMesh(config, subMeshes[index], subMeshSubDivs[index]);

	fs::remove_all(config.outputDir);
	fs::create_directory(config.outputDir);

	size_t meshIndex = 0;
	for(const auto& subMesh: subMeshSubDivs)
	{
		const std::vector<Mesh>& subdivs = subMesh;
		for(const Mesh& mesh: subdivs)
		{
			std::stringstream ss("");
			ss << meshIndex << ".stl";

			if(CGAL::IO::write_STL(config.outputDir + "/" + ss.str(), mesh))
				std::cout << "Saved " << ss.str() << std::endl;
			else
				std::cerr << "Failed to write " << ss.str() << "!" << std::endl;

			++meshIndex;
		}
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

