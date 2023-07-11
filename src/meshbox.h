#pragma once

#include "cgals.h"
#include "grid_cut.h"
#include "multivec.h"
#include "vec.h"

struct MeshBox;

constexpr int NUM_SIDES = 6;

struct GridCell
{
	Vector3D position;

	Mesh mesh;
	MeshBox* parent;
	std::array<MeshBox*,NUM_SIDES> sideParents;

	enum class ContentType {
		Internal,
		Boundary,
		Empty
	} type;
};

struct MeshBox
{
	Mesh mesh;
	Cuboid dims;
	std::list<GridCell*> children;
	
	//K::Iso_cuboid_3 dims;
	std::array<bool,NUM_SIDES> sideChanged;
};

void getSurfaceBoxes(
	const Mesh& mesh, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes);

void extractUniqueMeshBoxes(
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox*>& meshBoxes,
	int sideIndex);

void assignAsParent(mv::vector3<GridCell>& gridCells, int sideIndex);

