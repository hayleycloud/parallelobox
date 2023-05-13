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

typedef std::vector<MeshBox*> MeshBoxPtrList;
typedef std::vector<std::pair<MeshBox*,double>> MeshBoxPtrCostList;

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


enum class Direction {
	Left, Right, Up, Down, In, Out
};

Direction invert(Direction direction)
{
	switch(direction)
	{
		case Direction::Left:
			return Direction::Right;
		case Direction::Right:
			return Direction::Left;
		case Direction::Up:
			return Direction::Down;
		case Direction::Down:
			return Direction::Up;
		case Direction::In:
			return Direction::Out;
		case Direction::Out:
			return Direction::In;
	}
	return Direction::Out;
}

struct GridPathed
{
	mv::vector3<GridCell> cells;
	unsigned int sideIndex;
};

bool getAdjacentBoxIfExists(
	GridPathed& gridCells, 
	int x, int y, int z,
	std::unordered_set<MeshBox*>& meshboxes)
{
	GridCell& cell = mv::get(gridCells.cells, x, y, z);
	if(cell.type == GridCell::ContentType::Boundary)
	{
		assert(cell.parent);
		meshboxes.insert(cell.sideParents[gridCells.sideIndex]);
		return true;
	}
	else
		return false;
}

void getAdjacentBox(
	GridPathed& gridCells, 
	int x, int y, int z,
	Direction direction,
	std::unordered_set<MeshBox*>& adjacentBoxes)
{
	switch(direction)
	{
		case Direction::Left:
			getAdjacentBoxIfExists(gridCells, x - 1, y, z, adjacentBoxes);
			break;                                                      
		case Direction::Right:                                          
			getAdjacentBoxIfExists(gridCells, x + 1, y, z, adjacentBoxes);
			break;                                                      
		case Direction::Up:                                             
			getAdjacentBoxIfExists(gridCells, x, y + 1, z, adjacentBoxes);
			break;                                                      
		case Direction::Down:                                           
			getAdjacentBoxIfExists(gridCells, x, y - 1, z, adjacentBoxes);
			break;                                                      
		case Direction::In:                                             
			getAdjacentBoxIfExists(gridCells, x, y, z + 1, adjacentBoxes);
			break;                                                      
		case Direction::Out:                                            
			getAdjacentBoxIfExists(gridCells, x, y, z - 1, adjacentBoxes);
			break;
	}
}

std::unordered_set<MeshBox*> getAdjacentBoxes(
	GridPathed& gridCells,
	MeshBox& box,
	Direction direction)
{
	std::unordered_set<MeshBox*> adjacentBoxes;

	for(GridCell* child: box.children)
	{
		assert(child);
		const Vector3D& position = child->position;
		getAdjacentBox(
			gridCells, 
			position.x, position.y, position.z, 
			direction, 
			adjacentBoxes);
	}

	adjacentBoxes.erase(std::addressof(box));
	
	return adjacentBoxes;
}

struct AdjacencyBranch
{
	MeshBox* parent;
	MeshBox* child;
	Direction direction;

	enum class Type { Expand, Shrink } type;
};

Direction determineOptimumShrinkDirection(
	MeshBox& predator, MeshBox& prey,
	Direction expandDirection)
{
	// TODO: Optimisation logic
	
	return invert(expandDirection);
}

AdjacencyBranch::Type toggleType(AdjacencyBranch::Type type)
{
	if(type == AdjacencyBranch::Type::Expand)
		return AdjacencyBranch::Type::Shrink;
	//else if(type == AdjacencyBranch::Type::Shrink)
	return AdjacencyBranch::Type::Expand;
}

bool nodeAlreadyVisited(
	MeshBox& node,
	const std::vector<AdjacencyBranch>& adjacencyBranches)
{
	for(const AdjacencyBranch& branch: adjacencyBranches)
	{
		if(branch.parent == std::addressof(node))
			return true;
	}
	return false;
}

void exploreBranches(
	GridPathed& gridCells,
	std::vector<AdjacencyBranch>& adjacencyBranches,
	MeshBox& box, 
	Direction direction,
	AdjacencyBranch::Type type = AdjacencyBranch::Type::Expand)
{
	AdjacencyBranch::Type nextType = toggleType(type);

	auto adjBoxes = getAdjacentBoxes(gridCells, box, direction);
	for(auto adjBox: adjBoxes)
	{
		if(nodeAlreadyVisited(*adjBox, adjacencyBranches))
			adjBoxes.erase(adjBox);
	}

	if(adjBoxes.size() == 0)
	{
		adjacencyBranches.emplace_back((AdjacencyBranch){
			std::addressof(box), nullptr, 
			determineOptimumShrinkDirection(),
			nextType
		});
		return;
	}

	std::vector<Direction> directions;
	for(auto adjBox: adjBoxes)
	{
		adjacencyBranches.emplace_back((AdjacencyBranch){
			std::addressof(box), std::addressof(adjBox), 
			determineOptimumShrinkDirection(),
			nextType
		});
	}

	for(auto adjBox: adjBoxes)
	{
		exploreBranches(
			gridCells, adjacencyBranches, *adjBox, direction, nextType);
	}
}

void removeMeshBoxCells(
	int sideIndex, 
	MeshBox& box, 
	std::function<bool(const Cuboid&,const Vector3D&)> test)
{
	std::vector<GridCell*> removal;
	for(GridCell* cell: box.children)
	{
		if(test(box.dims, cell->position))
		{
			removal.push_back(cell);
			cell->sideParents[sideIndex] = nullptr;
		}
	}

	for(auto cell: removal)
		box.children.remove(cell);
}

bool shrink(GridPathed& gridCells, MeshBox& box, Direction direction)
{
	int sideIndex = gridCells.sideIndex;

	// Direction indicates the side being shrunk!
	switch(direction)
	{
		case Direction::Left:
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.x == pos.x;
			});

			--box.dims.size.x;
			if(box.dims.size.x == 0)
				return false;

			++box.dims.origin.x;
		}
		break;                                                      
		case Direction::Right:                                          
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.x + (a.size.x - 1)) == pos.x;
			});

			--box.dims.size.x;
			if(box.dims.size.x == 0)
				return false;
		}
		break;                                                 
		case Direction::Up:                                             
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.y + (a.size.y - 1)) == pos.y;
			});

			--box.dims.size.y;
			if(box.dims.size.y == 0)
				return false;
		}
		break;                                                      
		case Direction::Down:                                           
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.y == pos.y;
			});

			--box.dims.size.y;
			if(box.dims.size.y == 0)
				return false;

			++box.dims.origin.y;
		}
		break;                                                      
		case Direction::In:                                             
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return (a.origin.z + (a.size.z - 1)) == pos.z;
			});

			--box.dims.size.z;
			if(box.dims.size.z == 0)
				return false;
		}
		break;                                                      
		case Direction::Out:                                            
		{
			removeMeshBoxCells(sideIndex, box, 
				[&](const Cuboid& a, const Vector3D& pos) {
					return a.origin.z == pos.z;
			});

			--box.dims.size.z;
			if(box.dims.size.z == 0)
				return false;

			++box.dims.origin.z;
		}
		break;
	}

	return true;
}

void addCellsToMeshBox(
	GridPathed& gridCells, 
	MeshBox& box,
	const Vector3D& a, const Vector3D& b) 
{
    for(int x = a.x; x < b.x; x++) 
	{
        for(int y = a.y; y < b.y; y++) 
		{
            for(int z = a.z; z < b.z; z++) 
			{
				GridCell* cell = std::addressof(mv::get(gridCells.cells, x, y, z));
				assert(cell->sideParent[gridCells.sideIndex] == nullptr);

				cell->sideParents[gridCells.sideIndex] = std::addressof(box);
                box.children.push_back(cell);
            }
        }
    }
}

void grow(GridPathed& gridCells, MeshBox& box, Direction direction)
{
	// Direction indicates the side being grown!
	switch(direction)
	{
		case Direction::Left:
		{
			--box.dims.origin.x;
			++box.dims.size.x;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topLeftIn = 
				btmLeftOut + Vector3D(0, box.dims.size.y, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmLeftOut, topLeftIn);
		}
		break;                                                      
		case Direction::Right:                                          
		{
			++box.dims.size.x;

			const Vector3D btmRightOut = 
				box.dims.origin + Vector3D(box.dims.size.x, 0, 0);
			const Vector3D topRightIn = 
				btmRightOut + Vector3D(0, box.dims.size.y, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmRightOut, topRightIn);
		}
		break;                                                 
		case Direction::Up:                                             
		{
			++box.dims.size.y;

			const Vector3D topLeftOut = 
				box.dims.origin + Vector3D(0, box.dims.size.y, 0);
			const Vector3D topRightIn = 
				topLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, topLeftOut, topRightIn);
		}
		break;                                                      
		case Direction::Down:                                           
		{
			--box.dims.origin.y;
			++box.dims.size.y;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D btmRightIn = 
				btmLeftOut + Vector3D(box.dims.size.x, 0, box.dims.size.z);
			addCellsToMeshBox(gridCells, box, btmLeftOut, btmRightIn);
		}
		break;                                                      
		case Direction::In:                                             
		{
			++box.dims.size.z;

			const Vector3D btmLeftIn = 
				box.dims.origin + Vector3D(0, 0, box.dims.size.z);
			const Vector3D topRightIn = 
				btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 0);
			addCellsToMeshBox(gridCells, box, btmLeftIn, topRightIn);
		}
		break;                                                      
		case Direction::Out:                                            
		{
			--box.dims.origin.z;
			++box.dims.size.z;

			const Vector3D& btmLeftOut = box.dims.origin;
			const Vector3D topRightOut = 
				btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 0);
			addCellsToMeshBox(gridCells, box, btmLeftOut, topRightOut);
		}
		break;
	}
}

void applyBranchShrinks(
	GridPathed& gridCells, 
	const std::vector<AdjacencyBranch>& branches)
{
	for(const AdjacencyBranch& branch: branches)
	{
		if(branch.type == AdjacencyBranch::Type::Shrink)
			shrink(gridCells, *branch.parent, branch.direction);
	}
}

void applyBranchGrows(
	GridPathed& gridCells, 
	const std::vector<AdjacencyBranch>& branches)
{
	for(const AdjacencyBranch& branch: branches)
	{
		if(branch.type == AdjacencyBranch::Type::Expand)
			grow(gridCells, *branch.parent, branch.direction);
	}
}

void feed(
	GridPathed& gridCells,
	MeshBox& prey,
	Direction direction)
{
	std::vector<AdjacencyBranch> adjacencyBranches;
	exploreBranches(gridCells, adjacencyBranches, prey, direction);

	applyBranchShrinks(gridCells, adjacencyBranches);
	applyBranchGrows(gridCells, adjacencyBranches);
}

// The Dialogue
// Okay, right, wtf are we doing here
// Okay, the gist is, we feed the smallest box to the neighbours in the best way
// 
// Get the adjacent neighbour of each box of the prey in the given direction?
// But this means that *all* of those adjacent that way need expanding into it
// And expanding those might cause shrinkage requirements on other models!
// ERRRRR
// Okay, OOPS, this needs thinking through
//
// Okay, so, yes, let's try this:
// - Copy of mesh state per initial feed direction?
// - Apply sequences of feeding and expansion until fully resolved
// - Compute costs for each initial feed direction
// - Select and forward best one
// - Grid cells might need 6 extra parent ptrs for each direction
// - - Set from current parent, change as needed

void mergeIterate(
	const Config& config, 
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes)
{
	// TODO: Replace with conditional check
	while(true)
	{
		MeshBoxPtrCostList meshboxCosts; 
		for(auto itr = meshBoxes.begin(); itr != meshBoxes.end(); ++itr)
		{
			auto& meshBox = *itr;
			meshboxCosts.push_back(std::make_pair(
				std::addressof(meshBox), fitness(config, meshBox.mesh)));
		}

		// Sort the meshboxes such that highest cost is first
		// TODO: Lowest cost ya dummy
		std::sort(meshboxCosts.begin(), meshboxCosts.end(), [](auto& a, auto& b) {
			return a.second > b.second;
		});

		for(const auto& box: meshboxCosts)
			std::cout << box.second << std::endl;

		MeshBox* prey = meshboxCosts.front().first;

		// TODO: Recalculate only altered mesh boxes?
		std::array<std::list<MeshBox>,6> sideInstances;
		std::array<Direction,6> sideDirections = {
			Direction::Left, Direction::Right,
			Direction::Up, Direction::Down,
			Direction::In, Direction::Out
		};

		for(unsigned int sideIndex = 0; sideIndex < sideInstances.size(); ++sideIndex)
		{
			GridPathed gridPath = {gridCells, sideIndex};
			feed(gridPath, *prey, sideDirections[sideIndex]);
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

	mv::vector3<GridCell> gridCells;
	std::list<MeshBox> meshBoxes;
    getSurfaceBoxes(inputMesh, grid, gridCells, meshBoxes);

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

	const std::array<K::Vector_3,6> cardinalVecs = {
		K::Vector_3( 0,  1,  0),
		K::Vector_3( 0, -1,  0),
		K::Vector_3(-1,  0,  0),
		K::Vector_3( 1,  0,  0),
		K::Vector_3( 0,  0,  1),
		K::Vector_3( 0,  0, -1)
	};
	
	for(auto item = meshBoxes.begin(); item != meshBoxes.end(); ++item)
	{
		MeshBox& meshBox = *item;
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
	}

	/*mv::vector3<MeshBox*> meshBoxRefs = mv::map<MeshBox,MeshBox*>(
		[](MeshBox& meshBox) -> MeshBox* {
			if(meshBox.type == MeshBox::ContentType::Empty)
				return nullptr;
			else
				return &meshBox;
	}, meshBoxes);*/

	mergeIterate(config, gridCells, meshBoxes);

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

