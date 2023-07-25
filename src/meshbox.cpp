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
	std::list<MeshBox>& meshBoxes)
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
					meshBoxes.push_back((MeshBox) {
						volMesh, 
						Cuboid(Vector3D(x, y, z), Vector3D(1, 1, 1)),
						{ std::addressof(gridCells[z][y][x]) }, 
						std::array<bool,NUM_SIDES>()
					});

					gridCells[z][y][x].parent = std::addressof(meshBoxes.back());
					for(auto& sideParent: gridCells[z][y][x].sideParents)
						sideParent = gridCells[z][y][x].parent;
				}
			}
		}
	}
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

	std::cout << "Clipping using " << meshOrigin << ", " << meshEnd << std::endl;

	child.mesh = mesh;
	PMP::clip(child.mesh, bbox, CGAL::parameters::clip_volume(true));
}

void extractUniqueMeshBoxes(
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox*>& meshBoxes,
	int sideIndex)
{
	std::set<MeshBox*> meshBoxSet;
	
	mv::forEach<GridCell>([&](GridCell& cell) {
		MeshBox* parent = cell.sideParents[sideIndex];
		if(parent)
			meshBoxSet.insert(parent);
	}, gridCells);

	std::copy(meshBoxSet.begin(), meshBoxSet.end(), std::back_inserter(meshBoxes));
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
		MeshBox* parent = cell.sideParents[sideIndex];
		if(parent == nullptr)
			return;
		cell.parent = parent;
		parent->children.push_back(std::addressof(cell));
	}, gridCells);
}
