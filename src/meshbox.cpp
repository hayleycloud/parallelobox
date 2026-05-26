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
	
	#pragma omp parallel for default(none) shared(mesh, gridCells, grid, inside)// firstprivate(mesh)
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

				bool hasValidSurface = surfMesh.number_of_vertices() > 0;
				surfMesh.clear();
				surfMesh.collect_garbage();

				if(hasValidSurface)
				{
					Mesh volMesh(mesh);
					grid.clipVolume(volMesh, x, y, z);
					validate(volMesh, false);

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

std::vector<GridCell*> sampleCells(
	mv::vector3<GridCell>& gridCells,
	const Vector3D& a, const Vector3D& b)
{
	std::vector<GridCell*> cells;

    for(int x = a.x; x < b.x; ++x)
	{
        for(int y = a.y; y < b.y; ++y)
		{
            for(int z = a.z; z < b.z; ++z)
			{
				GridCell* cell = std::addressof(mv::get(gridCells, x, y, z));
				assert(cell);
				cells.push_back(cell);
            }
        }
    }

	return cells;
}

void resetGridCells(mv::vector3<GridCell>& gridCells)
{
	mv::forEach<GridCell>([](GridCell& cell) {
		cell.parents.clear();
	}, gridCells);
}

void getFloorVectorsFrom(
	const Grid& grid,
	const MeshBox& meshBox, 
	const std::vector<Direction>& upDirections,
	std::vector<K::Vector_3>& floors)
{
	const Cuboid& bb = meshBox.dims;
	const Vector3D bbEnd = bb.last();

	const K::Point_3 min = grid.get(bb.origin.x, bb.origin.y, bb.origin.z).min();
	const K::Point_3 max = grid.get(bbEnd.x, bbEnd.y, bbEnd.z).max();

	for(Direction direction: upDirections)
	{
		switch(direction)
		{
			case Direction::Left:
			{
				floors.push_back(K::Vector_3(max.x(), 0.0, 0.0));
				break;
			}
			case Direction::Right:
			{
				floors.push_back(K::Vector_3(min.x(), 0.0, 0.0));
				break;
			}
			case Direction::Up:
			{
				floors.push_back(K::Vector_3(0.0, min.y(), 0.0));
				break;
			}
			case Direction::Down:
			{
				floors.push_back(K::Vector_3(0.0, max.y(), 0.0));
				break;
			}
			case Direction::In:
			{
				floors.push_back(K::Vector_3(0.0, 0.0, min.z()));
				break;
			}
			case Direction::Out:
			{
				floors.push_back(K::Vector_3(0.0, 0.0, max.z()));
				break;
			}
		}
	}
}

std::vector<MeshBox*> getNeighbours(
	const Grid& grid,
	const MeshBox& box,
	mv::vector3<GridCell>& gridCells)
{
	std::set<MeshBox*> neighbours;

	if(box.dims.origin.x > 0)
	{
		Vector3D btmLeftOut = box.dims.origin;
		--btmLeftOut.x;

		Vector3D topLeftIn = 
			btmLeftOut + Vector3D(1, box.dims.size.y, box.dims.size.z);

		for(const GridCell* cell: sampleCells(gridCells, btmLeftOut, topLeftIn))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());

	}

	if(box.dims.end().x < grid.getNumBoxesX())
	{
		const Vector3D btmRightOut = 
			box.dims.origin + Vector3D(box.dims.size.x, 0, 0);
		const Vector3D topRightIn = 
			btmRightOut + Vector3D(1, box.dims.size.y, box.dims.size.z);

		for(const GridCell* cell: sampleCells(gridCells, btmRightOut, topRightIn))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());
	}

	if(box.dims.origin.y > 0)
	{
		Vector3D btmLeftOut = box.dims.origin;
		--btmLeftOut.y;

		Vector3D btmRightIn =
			btmLeftOut + Vector3D(box.dims.size.x, 1, box.dims.size.z);

		for(const GridCell* cell: sampleCells(gridCells, btmLeftOut, btmRightIn))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());
	}

	if(box.dims.end().y < grid.getNumBoxesY())
	{
		const Vector3D topLeftOut = 
			box.dims.origin + Vector3D(0, box.dims.size.y, 0);
		const Vector3D topRightIn = 
			topLeftOut + Vector3D(box.dims.size.x, 1, box.dims.size.z);

		for(const GridCell* cell: sampleCells(gridCells, topLeftOut, topRightIn))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());
	}

	if(box.dims.origin.z > 0)
	{
		Vector3D btmLeftOut = box.dims.origin;
		--btmLeftOut.z;

		Vector3D topRightOut =
			btmLeftOut + Vector3D(box.dims.size.x, box.dims.size.y, 1);

		for(const GridCell* cell: sampleCells(gridCells, btmLeftOut, topRightOut))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());
	}

	if(box.dims.end().z < grid.getNumBoxesZ())
	{
		const Vector3D btmLeftIn = 
			box.dims.origin + Vector3D(0, 0, box.dims.size.z);
		const Vector3D topRightIn = 
			btmLeftIn + Vector3D(box.dims.size.x, box.dims.size.y, 1);

		for(const GridCell* cell: sampleCells(gridCells, btmLeftIn, topRightIn))
			if(!cell->parents.empty())
				neighbours.insert(cell->parents.front());
	}


	return {neighbours.begin(), neighbours.end()};
}

// Merge without children
bool mergeSoft(MeshBox& a, MeshBox& b)
{
	const Mesh& aConst = a.mesh, bConst = b.mesh;
	Mesh aTemp(aConst);
	Mesh bTemp(bConst);
	
	if(PMP::does_self_intersect(aTemp))
		throw std::runtime_error("Mesh A self-intersects!");
	
	if(PMP::does_self_intersect(bTemp))
		throw std::runtime_error("Mesh B self-intersects!");
	
	bool success = PMP::corefine_and_compute_union(aTemp, bTemp, aTemp);
	if(success)
	{
		if(INVALID(validate(aTemp, false)))
			return false;

		const Vector3D A = a.dims.end(), B = b.dims.end();
		int x = std::min(a.dims.origin.x, b.dims.origin.x);
		int y = std::min(a.dims.origin.y, b.dims.origin.y);
		int z = std::min(a.dims.origin.z, b.dims.origin.z);
		int w = std::max(A.x, B.x) - x;
		int h = std::max(A.y, B.y) - y;
		int d = std::max(A.z, B.z) - z;
		a.dims = Cuboid({x, y, z}, {w, h, d});

		a.mesh = aTemp;
		recomputeNormals(a);
	}

	return success;
}

// No children
bool mergeSoft(MeshBox& a, MeshBox& b, MeshBox& c)
{
	throw std::runtime_error("Function not implemented!");
	return false;
}

bool mergeSoft(MeshBox& a, MeshBox& b, Mesh& outMesh, Cuboid& outDims)
{
	throw std::runtime_error("Function not implemented!");
	return false;
}

void transferChildren(MeshBox& into, MeshBox& from)
{
	for(GridCell* child: from.children)
	{
		into.children.push_back(child);
		child->parents.clear();
		child->parents.push_back(std::addressof(into));
	}

	from.children.clear();
}

void assignNormals(Mesh& mesh)
{
	auto fnormals = mesh.add_property_map<face_descriptor, K::Vector_3>(
		"f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_face_normals(mesh, fnormals);
}

void assignNormals(MeshBox& meshBox)
{
	assignNormals(meshBox.mesh);
}

void recomputeNormals(Mesh& mesh)
{
	auto oldMap = mesh.property_map<face_descriptor,K::Vector_3>("f:normals");
	if(oldMap)
		mesh.remove_property_map(*oldMap);

	assignNormals(mesh);
}

void recomputeNormals(MeshBox& meshBox)
{
	recomputeNormals(meshBox.mesh);
}


MeshErrorSet clipFromMesh(const Grid& grid, const Mesh& mesh, const Cuboid& dims, Mesh& out)
{
	MeshErrorSet errors = NO_MESH_ERRORS;

	const K::Point_3& origin = grid.getOrigin();
	const K::Point_3 meshOrigin(
		origin.x() + (dims.origin.x * grid.getElementSize()),
		origin.y() + (dims.origin.y * grid.getElementSize()),
		origin.z() + (dims.origin.z * grid.getElementSize()));
	const K::Vector_3 meshSize(
		dims.size.x * grid.getElementSize(),
		dims.size.y * grid.getElementSize(),
		dims.size.z * grid.getElementSize());
	const K::Point_3 meshEnd = meshOrigin + meshSize;

	const CGAL::Iso_cuboid_3<K> bbox(meshOrigin, meshEnd);

	//std::cout << "Clipping using " << meshOrigin << ", " << meshEnd << std::endl;

	out = mesh;
	if(!PMP::clip(out, bbox, CGAL::parameters::clip_volume(true)))
		SET_MESH_ERROR(errors, MeshErrors::UncertainManifoldness);

	SET_MESH_ERROR(errors, validate(out, false));

	assignNormals(out);

	return errors;
}

MeshErrorSet clipFromMesh(const Grid& grid, const Mesh& mesh, MeshBox& child)
{
	return clipFromMesh(grid, mesh, child.dims, child.mesh);
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
 
