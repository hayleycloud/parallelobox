#include "cgals.h"
#include "config.h"
#include "grid_cut.h"

#include <omp.h>

#include <limits>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

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

void recenter(Mesh& mesh)
{
	K::Point_3 c = CGAL::centroid(
		mesh.points().begin(), 
		mesh.points().end(),
		CGAL::Dimension_tag<0>());
	K::Vector_3 dc(-c.x(), -c.y(), -c.z());

	for(Mesh::Vertex_index vIndex: mesh.vertices())
	{
		mesh.point(vIndex) += dc;
	}
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

	recenter(inputMesh);

	std::cout << "Mesh re-centered." << std::endl;

	K::Point_3 min, max;
	bounds(min, max, inputMesh);

	std::cout << "Min: " << min << ", Max: " << max << std::endl;

	min = min - K::Vector_3(1.0, 1.0, 1.0);
	max = max + K::Vector_3(1.0, 1.0, 1.0);
	K::Vector_3 size(max.x() - min.x(), max.y() - min.y(), max.z() - min.z());

	auto fnormals = inputMesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(inputMesh, fnormals);

	std::cout << "Computed normals." << std::endl;

	Grid grid(size.x(), size.y(), size.z(), 2, 1, 2);
	
	std::vector<Mesh> subdivs;
	grid.clip(inputMesh, subdivs);

//	CGAL::Iso_cuboid_3<K> cube1(
//		K::Point_3(0, min.y(), min.z()), 
//		K::Point_3(max.x(), max.y(), max.z()));
//	CGAL::Iso_cuboid_3<K> cube2(
//		K::Point_3(min.x(), min.y(), min.z()), 
//		K::Point_3(0, max.y(), max.z()));

	fs::remove_all(config.outputDir);
	fs::create_directory(config.outputDir);

	size_t meshIndex = 0;
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

int main(int argc, char* argv[])
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

