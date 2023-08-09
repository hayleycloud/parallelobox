#include "meshbox.h"

//GridCell::GridCell()
//{
//}

GridCell::ContentType determineContentType(
	const Mesh& mesh, 
	const Mesh& surfMesh, 
	const Mesh& volMesh,
	int x, int y, int z)
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
	std::list<std::unique_ptr<MeshBox>>& meshBoxes)
{
	gridCells.resize(grid.getThickness());
	
	#pragma omp parallel for
	for(int z = 0; z < grid.getThickness(); ++z)
	{
		gridCells[z].resize(grid.getHeight());
		for(int y = 0; y < grid.getHeight(); ++y)
		{
			gridCells[z][y].resize(grid.getWidth());
			for(int x = 0; x < grid.getWidth(); ++x)
			{
				const auto& dims = grid.get(x, y, z);

				Mesh surfMesh(mesh);
				grid.clipSurface(surfMesh, x, y, z);

				Mesh volMesh(mesh);
				grid.clipVolume(volMesh, x, y, z);

				auto contentType = determineContentType(
					mesh, surfMesh, volMesh, x, y, z);

				gridCells[z][y][x] = (GridCell) { 
					Vector3D(x, y, z),
					contentType == GridCell::ContentType::Boundary ?
						volMesh : surfMesh, 
					nullptr, 
					std::array<MeshBox*,NUM_SIDES>(),
					contentType 
				};

				if(contentType == GridCell::ContentType::Boundary)
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
				}
			}
		}
	}
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

	Vector3D size(max.x - min.x, max.y - min.y, max.z - min.z);

	return Cuboid(min, size);
}

void getChildrenFromSide(
	mv::vector3<GridCell>& gridCells, 
	MeshBox* meshBox, 
	std::vector<GridCell*>& children, 
	int sideIndex)
{
	mv::forEach<GridCell>([&](GridCell& cell) {
		if(cell.sideParents[sideIndex] == meshBox)
			children.push_back(std::addressof(cell));
	}, gridCells);
}

void reparentChildrenForSide(
	std::vector<GridCell*>& children, MeshBox* parent, int sideIndex)
{
	for(GridCell* child: children)
		child->sideParents[sideIndex] = parent;
}

void clipFromMesh(const Grid& grid, const Mesh& mesh, MeshBox& child)
{
	const Cuboid& mbDims = child.dims;
	const K::Point_3& origin = grid.getOrigin();
	const K::Point_3 meshOrigin(
		origin.x() + (mbDims.origin.x * grid.getXStepSize()),
		origin.y() + (mbDims.origin.y * grid.getYStepSize()),
		origin.z() + (mbDims.origin.z * grid.getZStepSize()));
	const K::Vector_3 meshSize(
		(mbDims.size.x + 1) * grid.getXStepSize(),
		(mbDims.size.y + 1) * grid.getYStepSize(),
		(mbDims.size.z + 1) * grid.getZStepSize());
	const K::Point_3 meshEnd = meshOrigin + meshSize;

	const CGAL::Iso_cuboid_3<K> bbox(meshOrigin, meshEnd);

	//std::cout << "Clipping using " << meshOrigin << ", " << meshEnd << std::endl;

	child.mesh = mesh;
	PMP::clip(child.mesh, bbox, CGAL::parameters::clip_volume(true));
}

void getUniqueMeshBoxes(
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
