#include "meshbox.h"
#include <CGAL/Side_of_triangle_mesh.h>

//GridCell::GridCell()
//{
//}

inline GridCell::ContentType determineContentType(
	const Mesh& surfMesh, 
	const Mesh& volMesh)
{
	if(surfMesh.number_of_vertices() > 0)
		return GridCell::ContentType::Boundary;
	else 
	{
		if(volMesh.number_of_vertices() > 0)
			return GridCell::ContentType::Internal;
		else
			return GridCell::ContentType::Empty;
	}
}

void getSurfaceBoxes(
	const Mesh& mesh, 
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells,
	int numPrinters)
{
	CGAL::Side_of_triangle_mesh<Mesh, K> inside(mesh);

	gridCells.resize(grid.getNumBoxesZ());
	
	#pragma omp parallel for default(none) shared(gridCells, grid, inside) firstprivate(mesh)
	for(int z = 0; z < grid.getNumBoxesZ(); ++z)
	{
		gridCells[z].resize(grid.getNumBoxesY());
		for(int y = 0; y < grid.getNumBoxesY(); ++y)
		{
			gridCells[z][y].resize(grid.getNumBoxesX());
			for(int x = 0; x < grid.getNumBoxesX(); ++x)
			{
				const auto& dims = grid.get(x, y, z);

				Mesh surfMesh(mesh);
				grid.clipSurface(surfMesh, x, y, z);

				if(surfMesh.number_of_vertices() > 0)
				{
					Mesh volMesh(mesh);
					grid.clipVolume(volMesh, x, y, z);

					gridCells[z][y][x] = (GridCell) { 
						Vector3D(x, y, z), volMesh, {}, 
						GridCell::ContentType::Boundary
					};
				}
				else
				{
					CGAL::Bounded_side res = inside(dims.min());
					GridCell::ContentType contentType = 
						res == CGAL::ON_BOUNDED_SIDE ? 
						GridCell::ContentType::Internal : 
						GridCell::ContentType::Empty;

					gridCells[z][y][x] = (GridCell) { 
						Vector3D(x, y, z), {}, {}, contentType
					};
				}


				/*if(contentType == GridCell::ContentType::Boundary)
				{
                    #pragma omp critical
					meshBoxes.emplace_back(std::make_unique<MeshBox>((MeshBox){
						volMesh, 
						Cuboid(Vector3D(x, y, z), Vector3D(1, 1, 1)),
						{ std::addressof(gridCells[z][y][x]) }, 
						std::array<bool,NUM_SIDES>()
					}));

					gridCells[z][y][x].parent = meshBoxes.back().get();
					for(auto& sideParent: gridCells[z][y][x].sideParents)
						sideParent = gridCells[z][y][x].parent;
				}*/
			}
		}
	}
}

Cuboid getDimsFrom(const std::vector<const GridCell*>& children)
{
	constexpr int MAX_INT = std::numeric_limits<int>::max();
	constexpr int MIN_INT = std::numeric_limits<int>::min();
	Vector3D min(MAX_INT, MAX_INT, MAX_INT), max(MIN_INT, MIN_INT, MIN_INT);
	for(const GridCell* child: children)
	{
		if(child->position.x < min.x)
			min.x = child->position.x;
		if(child->position.x > max.x)
			max.x = child->position.x;
		
		if(child->position.y < min.y)
			min.y = child->position.y;
		if(child->position.y > max.y)
			max.y = child->position.y;

		if(child->position.z < min.z)
			min.z = child->position.z;
		if(child->position.z > max.z)
			max.z = child->position.z;
	}

	Vector3D size((max.x - min.x) + 1, (max.y - min.y) + 1, (max.z - min.z) + 1);

	return { min, size };
}

Cuboid getDimsFrom(std::vector<GridCell*>& children)
{
	constexpr int MAX_INT = std::numeric_limits<int>::max();
	constexpr int MIN_INT = std::numeric_limits<int>::min();
	Vector3D min(MAX_INT, MAX_INT, MAX_INT), max(MIN_INT, MIN_INT, MIN_INT);
	for(GridCell* child: children)
	{
		if(child->position.x < min.x)
			min.x = child->position.x;
		if(child->position.x > max.x)
			max.x = child->position.x;
		
		if(child->position.y < min.y)
			min.y = child->position.y;
		if(child->position.y > max.y)
			max.y = child->position.y;

		if(child->position.z < min.z)
			min.z = child->position.z;
		if(child->position.z > max.z)
			max.z = child->position.z;
	}

	Vector3D size((max.x - min.x) + 1, (max.y - min.y) + 1, (max.z - min.z) + 1);

	return { min, size };
}


void resetGridCells(mv::vector3<GridCell>& gridCells)
{
	mv::forEach<GridCell>([](GridCell& cell) {
		cell.parents.clear();
	}, gridCells);
}

/*bool getNeighbouringEmptyCells(
	const Vector3D& position,
	const Grid& grid, 
	const mv::vector3<GridCell>& gridCells,
	std::set<const GridCell*>& emptyCells)
{
	const GridCell& cell = mv::get(gridCells, position.x, position.y, position.z);
	if(cell.type != GridCell::ContentType::Boundary || !cell.parents.empty())
		return false;

	if(emptyCells.insert(std::addressof(cell)).second == false)
		return false;

	const std::vector<Vector3D> directions = {
		Vector3D(-1, 0, 0),	Vector3D(1, 0, 0),
		Vector3D(0, -1, 0), Vector3D(0, 1, 0),
		Vector3D(0, 0, -1), Vector3D(0, 0, 1)
	};
	std::vector<Vector3D> positions;
	for(const Vector3D& direction: directions)
	{
		Vector3D newPos = position + direction;
		if(newPos.x >= 0 && newPos.y >= 0 && newPos.z >= 0
			&& newPos.x < grid.getNumBoxesX() && newPos.y < grid.getNumBoxesY() && newPos.z < grid.getNumBoxesZ())
			getNeighbouringEmptyCells(newPos, grid, gridCells, emptyCells);
	}

	return true;
}

int calcDiscreteRegions(
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells, 
	std::vector<std::vector<const GridCell*>>& regions)
{
	std::set<const GridCell*> allEmptyCells;

	while(true)
	{
		std::set<const GridCell*> emptyCells;

		bool found = false;
		Vector3D seedPos;
		mv::forEachIndexed<GridCell>([&](const GridCell& cell, int x, int y, int z) {
			if(!found && 
			   cell.type == GridCell::ContentType::Boundary && 
			   cell.parents.empty())
			{
				if(!allEmptyCells.contains(std::addressof(cell)))
				{
					found = true;
					seedPos = Vector3D(x, y, z);
				}
			}
		}, gridCells);

		if(!found)
			return regions.size();

		getNeighbouringEmptyCells(seedPos, grid, gridCells, emptyCells);
		for(const GridCell* cell: emptyCells)
			allEmptyCells.insert(cell);
		regions.emplace_back(std::vector<const GridCell*>(emptyCells.cbegin(), emptyCells.cend()));
	}
}

int getDiscreteEmptyRegions(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	std::vector<Cuboid>& emptyRegions)
{
	std::vector<std::vector<const GridCell*>> regionCells;
	int numRegions = calcDiscreteRegions(grid, gridCells, regionCells);

	for(const auto& region: regionCells)
		emptyRegions.emplace_back(getDimsFrom(region));

	return numRegions;
}*/

void clipFromMesh(const Grid& grid, const Mesh& mesh, MeshBox& child)
{
	const Cuboid& mbDims = child.dims;
	const K::Point_3& origin = grid.getOrigin();
	const K::Point_3 meshOrigin(
		origin.x() + (mbDims.origin.x * grid.getElementSize()),
		origin.y() + (mbDims.origin.y * grid.getElementSize()),
		origin.z() + (mbDims.origin.z * grid.getElementSize()));
	const K::Vector_3 meshSize(
		mbDims.size.x * grid.getElementSize(),
		mbDims.size.y * grid.getElementSize(),
		mbDims.size.z * grid.getElementSize());
	const K::Point_3 meshEnd = meshOrigin + meshSize;

	const CGAL::Iso_cuboid_3<K> bbox(meshOrigin, meshEnd);

	//std::cout << "Clipping using " << meshOrigin << ", " << meshEnd << std::endl;

	child.mesh = mesh;
	PMP::clip(child.mesh, bbox, CGAL::parameters::clip_volume(true));
}

/*void getUniqueMeshBoxes(
	mv::vector3<GridCell>& gridCells, 
	std::vector<MeshBox*>& meshBoxes,
	int sideIndex)
{
	std::set<MeshBox*> meshBoxSet;
	
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type != GridCell::ContentType::Boundary)
			return;

		MeshBox* parent = cell.sideParents[sideIndex];
		assert(parent);
		meshBoxSet.insert(parent);
	}, gridCells);

	std::copy(meshBoxSet.begin(), meshBoxSet.end(), std::back_inserter(meshBoxes));
}

void extractUniqueMeshBoxes(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	const Mesh& parentMesh,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	int sideIndex)
{
	std::vector<MeshBox*> meshBoxPtrs;
	getUniqueMeshBoxes(gridCells, meshBoxPtrs, sideIndex);

	//for(MeshBox* meshBoxPtr: meshBoxPtrs)
	//	std::cout << "MeshBoxPtr: " << meshBoxPtr << std::endl;

	for(MeshBox* meshBoxPtr: meshBoxPtrs)
	{
		std::vector<GridCell*> children;
		getChildrenFromSide(gridCells, meshBoxPtr, children, sideIndex);

		Cuboid bbox = getDimsFrom(children);

		std::unique_ptr<MeshBox> newBox = std::make_unique<MeshBox>((MeshBox){
			Mesh(), bbox, children, std::array<bool,NUM_SIDES>()
		});
		reparentChildrenForSide(children, newBox.get(), sideIndex);
		clipFromMesh(grid, parentMesh, *newBox);
		meshBoxes.push_back(std::move(newBox));
	}

	//printParents(gridCells, sideIndex);
}

void clearMeshBoxChanges(MeshBox& meshbox)
{
	std::fill(meshbox.sideChanged.begin(), meshbox.sideChanged.end(), false);
}

void clearMeshBoxChildren(MeshBox& meshbox)
{
	meshbox.children.clear();
}

void assignParents(mv::vector3<GridCell>& gridCells, int sideIndex)
{
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type != GridCell::ContentType::Boundary)
			return;

		MeshBox* parent = cell.sideParents[sideIndex];
		assert(parent);
		cell.parent = parent;
		
		for(auto& sideParent: cell.sideParents)
			sideParent = parent;

		//parent->children.push_back(std::addressof(cell));
	}, gridCells);
}

size_t numParents(mv::vector3<GridCell>& gridCells, int sideIndex)
{
	size_t num = 0;
	mv::forEach<GridCell>([&](const GridCell& cell) {
		if(cell.type != GridCell::ContentType::Boundary)
			return;
		++num;
	}, gridCells);

	return num;
}

void printParents(mv::vector3<GridCell>& gridCells)
{
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type != GridCell::ContentType::Boundary)
			return;

		std::cout << cell.parent << std::endl;
		//parent->children.push_back(std::addressof(cell));
	}, gridCells);
}

void printParents(mv::vector3<GridCell>& gridCells, int sideIndex)
{
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.type != GridCell::ContentType::Boundary)
			return;

		std::cout << cell.sideParents[sideIndex] << std::endl;
		//parent->children.push_back(std::addressof(cell));
	}, gridCells);
}
*/
