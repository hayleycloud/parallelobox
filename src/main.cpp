#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "symmetry.h"
#include "multivec.h"

#include <omp.h>

#include <limits>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

typedef std::vector<MeshBox*> MeshBoxList;
typedef std::vector<std::pair<MeshBox*,double>> MeshBoxCostList;

void bounds(const Mesh& mesh, K::Point_3& min, K::Point_3& max)
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
		mesh.point(vIndex) += dc;
}

void getTriangles(const Mesh& mesh, std::vector<K::Triangle_3>& triangles)
{
	for(Mesh::Face_index fIndex: mesh.faces())
	{
		std::vector<K::Point_3> triangle;

		for(Mesh::Vertex_index vIndex: CGAL::vertices_around_face(
			mesh.halfedge(fIndex), mesh))
		{
			triangle.push_back(mesh.point(vIndex));
		}

		triangles.push_back(K::Triangle_3(
			triangle[0], triangle[1], triangle[2]));
	}
}

K::Plane_3 pca(Mesh& mesh)
{
	K::Plane_3 principalPlane;

//	std::vector<K::Triangle_3> triangles;
//	getTriangles(mesh, triangles);
//
//	CGAL::linear_least_squares_fitting_3(
//		triangles.begin(),
//		triangles.end(),
//		principalPlane,
//		CGAL::Dimension_tag<2>());

	CGAL::linear_least_squares_fitting_3(
		mesh.points().begin(),
		mesh.points().end(),
		principalPlane,
		CGAL::Dimension_tag<0>());

	return principalPlane;
}

K::Vector_3 rotationFromPCA(const K::Plane_3& pcaPlane)
{
	return K::Vector_3(0.0, 0.0, 0.0);
}

void orient(Mesh& mesh)
{
	K::Plane_3 principalPlane = pca(mesh);

	K::Vector_3 ortho = principalPlane.orthogonal_vector();

	std::cout << "P: " << ortho << std::endl;

	K::Vector_3 zx = K::Vector_3(ortho.x(), 0.0, ortho.z());
	K::Vector_3 z = K::Vector_3(0.0, 0.0, 1.0);
	double a = CGAL::approximate_angle(z, zx);
	if(zx.x() < 0)
		a = -a;

	std::cout << "a: " << a << std::endl;
	//K::Vector_3 xy = K::Vector_3(
}

bool isFloor(
	const Mesh& mesh, const face_descriptor& fd, const K::Vector_3& floor)
{
	constexpr double eps = 0.0001;

	for(Mesh::Vertex_index vIndex: CGAL::vertices_around_face(
		mesh.halfedge(fd), mesh))
	{
		const auto& vertex = mesh.point(vIndex);

		if(floor.x() < -eps || floor.x() > eps)
			return std::abs(vertex.x() - floor.x()) <= eps;
		else if(floor.y() < -eps || floor.y() > eps)
			return std::abs(vertex.y() - floor.y()) <= eps;
		else if(floor.z() < -eps || floor.z() > eps)
			return std::abs(vertex.z() - floor.z()) <= eps;
	}

	return false;
}

double overhangArea(
	const Config& config, 
	const Mesh& mesh, 
	const auto& fnormals,
	const K::Vector_3& floor)
{
	// TODO: Optimisations can be done here! Everything's in degrees, and the 
	// trig can be avoided!
	const double maxOverhang = 90.0 + config.printer.overhangTolerance;

	double overhangSurfaceArea = 0.0;
	K::Vector_3 up(0.0, 1.0, 0.0);

	for(face_descriptor fd: faces(mesh))
	{
		auto n = fnormals[fd];
		double cos_a = acos(n * up) * r;
		if(cos_a >= maxOverhang)
		{
			if(isFloor(mesh, fd, floor)) {}
				//std::cout << "Floor ";
			else
			{
				overhangSurfaceArea += PMP::face_area(fd, mesh);
				//std::cout << "Overhang ";
			}
		}

		//std::cout << n << " (" << cos_a << ")" << std::endl;
	}

	return overhangSurfaceArea;
}

double overhangCost(
	const Config& config, 
	const Mesh& mesh, 
	const auto& fnormals,
	const K::Vector_3& floor)
{
	return overhangArea(config, mesh, fnormals, floor);
}

double printingCost(const Config& config, const Mesh& mesh)
{
	const double volumeCost = config.printer.infillSpeed * PMP::volume(mesh);
	const double surfaceCost = config.printer.shellSpeed * PMP::area(mesh);
	// TODO: squared_face_area for improved performance?
	return volumeCost + surfaceCost;
}

double fitsVolumeCost(const Config& config, const Mesh& mesh)
{
	K::Point_3 min, max;
	bounds(mesh, min, max);

	const double width  = abs(max.x() - min.x());
	const double height = abs(max.y() - min.y());
	const double depth  = abs(max.z() - min.z());

	if(width < config.printer.volume.width &&
	   height < config.printer.volume.height &&
	   depth < config.printer.volume.depth)
		return 0.0;
	else
		return std::numeric_limits<double>::max();
}

double fitness(const Config& config, const Mesh& mesh)
{
	return printingCost(config, mesh) + fitsVolumeCost(config, mesh);
}

// TODO: PROPOSED SUB-ALGORITHM
// 
// Loop through all the boxes, simulate and score growth in each axis
// - Limit loop to just the "best" i.e. smallest?
// - Need to look in to the grid to find the unique "grown"/axial-adjacent boxes
// - Apply merge of best score maneuver
// - - Shrink or remove the mergees, expand merger; synchronise as refs with grid

void merge(
	mv::vector3<MeshBox>& meshboxGrid,
	MeshBox& merger,
	MeshBox& mergee)
{
	
}

void expand(mv::vector3<MeshBox>& meshboxGrid, MeshBox& box)
{

}

void mergeIterate(
	const Config& config, 
	mv::vector3<MeshBox>& meshboxGrid, 
	MeshBoxList& meshBoxes)
{
	// TODO: Replace with conditional check
	while(true)
	{
		MeshBoxCostList meshboxCosts; 
		for(MeshBox* meshBox: meshBoxes)
		{
			meshboxCosts.push_back(std::make_pair(
				meshBox, fitness(config, meshBox->mesh)));
		}

		// Sort the meshboxes such that highest cost is first
		std::sort(meshboxCosts.begin(), meshboxCosts.end(), [](auto& a, auto& b) {
			return a.second > b.second;
		});

		for(const auto& box: meshboxCosts)
		{
			std::cout << box.second << std::endl;
		}

		char c;
		std::cin >> c;
	}
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

	std::cout << "Mesh loaded!" << std::endl;

	recenter(inputMesh);

	std::cout << "Mesh re-centered." << std::endl;

	//orient(inputMesh);
	
	std::vector<HardPlane> symmetries;
	findSymmetries(inputMesh, &symmetries);

	std::cout << "Found symmetries." << std::endl;

	K::Point_3 min, max;
	bounds(inputMesh, min, max);

	std::cout << "Min: " << min << ", Max: " << max << std::endl;

	min = min - K::Vector_3(1.0, 1.0, 1.0);
	max = max + K::Vector_3(1.0, 1.0, 1.0);
	K::Vector_3 size(max.x() - min.x(), max.y() - min.y(), max.z() - min.z());

	auto fnormals = inputMesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(inputMesh, fnormals);

	std::cout << "Computed normals." << std::endl;

	Grid grid(size.x(), size.y(), size.z(), 5, 5, 5);

	mv::vector3<MeshBox> meshBoxes = getSurfaceBoxes(inputMesh, grid);
	mv::forEach<MeshBox>([](MeshBox& box) {
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
	}, meshBoxes);

	MeshBoxList boundaryMeshBoxes = mv::reduce<MeshBox,MeshBoxList>(
		[](MeshBoxList& out, MeshBox& in) {
			if(in.type == MeshBox::ContentType::Boundary)
				out.push_back(std::addressof(in));
	}, meshBoxes);

	const std::array<K::Vector_3,6> cardinalVecs = {
		K::Vector_3( 0,  1,  0),
		K::Vector_3( 0, -1,  0),
		K::Vector_3(-1,  0,  0),
		K::Vector_3( 1,  0,  0),
		K::Vector_3( 0,  0,  1),
		K::Vector_3( 0,  0, -1)
	};
	
	mv::forEach<MeshBox>([&](MeshBox& meshBox) {
		if(meshBox.type != MeshBox::ContentType::Boundary)
			return;

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

		std::cout << bestOverhangArea << " of overhang." << std::endl;
	}, meshBoxes);

	mv::vector3<MeshBox*> meshBoxRefs = mv::map<MeshBox,MeshBox*>(
		[](MeshBox& meshBox) -> MeshBox* {
			if(meshBox.type == MeshBox::ContentType::Empty)
				return nullptr;
			else
				return &meshBox;
	}, meshBoxes);

	mergeIterate(config, meshBoxes, boundaryMeshBoxes);

	return EXIT_SUCCESS;

	std::vector<Mesh> subdivs;
	grid.clipVolume(inputMesh, subdivs);

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

