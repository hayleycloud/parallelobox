#include "config.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Iso_cuboid_3.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <omp.h>

#include <limits>
#include <fstream>
#include <filesystem>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace fs = std::filesystem;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;

void bounds(K::Point_3& min, K::Point_3& max, const Mesh& mesh)
{
	constexpr double minDbl = std::numeric_limits<double>::min();
	constexpr double maxDbl = std::numeric_limits<double>::max();
	double minX = maxDbl, minY = maxDbl, minZ = maxDbl;
	double maxX = minDbl, maxY = minDbl, maxZ = minDbl;

	for(Mesh::Vertex_index vIndex: mesh.vertices())
	{
		const K::Point_3& p = mesh.point(vIndex);
		if(p.x() < minX)
			minX = p.x();
		if(p.x() > maxX)
			maxX = p.x();
		if(p.y() < minY)
			minY = p.y();
		if(p.y() > maxY)
			maxY = p.y();
		if(p.z() < minZ)
			minZ = p.z();
		if(p.z() > maxZ)
			maxZ = p.z();
	}

	min = K::Point_3(minX, minY, minZ);
	max = K::Point_3(maxX, maxY, maxZ);
}

int run(int argc, char* argv[])
{
	Config config = handleArguments(argc, argv);

	std::chrono::time_point<std::chrono::steady_clock> startIval = 
		std::chrono::steady_clock::now();

	Mesh inputMesh;
	if(!PMP::IO::read_polygon_mesh(config.inputFile, inputMesh)) 
		throw std::runtime_error(config.inputFile + " could not be loaded!");

	std::cout << "Mesh loaded!" << std::endl;

	K::Point_3 min, max;
	bounds(min, max, inputMesh);

	std::cout << "Min: " << min << ", Max: " << max << std::endl;

	//for(Mesh::Vertex_index vd: inputMesh.vertices())
	//	std::cout << inputMesh.point(vd) << std::endl;

	min = min - K::Vector_3(1.0, 1.0, 1.0);
	max = max + K::Vector_3(1.0, 1.0, 1.0);

	CGAL::Iso_cuboid_3<K> cube1(
		K::Point_3(0, min.y(), min.z()), 
		K::Point_3(max.x(), max.y(), max.z()));
	CGAL::Iso_cuboid_3<K> cube2(
		K::Point_3(min.x(), min.y(), min.z()), 
		K::Point_3(0, max.y(), max.z()));

	Mesh leftCube(inputMesh), rightCube(inputMesh);
	PMP::clip(leftCube, cube1, CGAL::parameters::clip_volume(true));
	PMP::clip(rightCube, cube2, CGAL::parameters::clip_volume(true));

	fs::remove_all(config.outputDir);
	fs::create_directory(config.outputDir);

	//std::ofstream("out.off") << inputMesh;
	if(CGAL::IO::write_STL(config.outputDir + "/" + "left.stl", leftCube) &&
	   CGAL::IO::write_STL(config.outputDir + "/" + "right.stl", rightCube))
		std::cout << "Mesh re-saved!" << std::endl;
	else
		throw std::runtime_error("Could not save mesh!");

	std::chrono::time_point<std::chrono::steady_clock> endIval =
		std::chrono::steady_clock::now();
	std::chrono::duration<double, std::milli> duration =
		endIval - startIval;

	std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;

	return EXIT_SUCCESS;
}

int main(int argc, char* argv[])
{
	try
	{
		return run(argc, argv);
	}
	catch(const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return EXIT_FAILURE;
	}
}

