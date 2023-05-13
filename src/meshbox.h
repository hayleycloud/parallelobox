#pragma once

#include "cgals.h"
#include "grid_cut.h"
#include "multivec.h"
#include "vec.h"

struct MeshBox;

struct GridCell
{
	Vector3D position;

	Mesh mesh;
	MeshBox* parent;
	std::array<MeshBox*,6> sideParents;

	enum class ContentType {
		Internal,
		Boundary,
		Empty
	} type;
};

struct MeshBox
{
	Mesh mesh;
	std::list<GridCell*> children;
	
	Cuboid dims;
	//K::Iso_cuboid_3 dims;
};

void getSurfaceBoxes(
	const Mesh& mesh, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes);

