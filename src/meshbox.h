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
	std::vector<GridCell*> children;
	
	//K::Iso_cuboid_3 dims;
	std::array<bool,NUM_SIDES> sideChanged;
};

void getSurfaceBoxes(
	const Mesh& mesh, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes);

void getChildrenFromSide(
	mv::vector3<GridCell>& gridCells, 
	MeshBox* meshBox, 
	std::vector<GridCell*>& children, 
	int sideIndex);

void clipFromMesh(const Grid& grid, const Mesh& mesh, MeshBox& child);

void getUniqueMeshBoxes(
	mv::vector3<GridCell>& gridCells, 
	std::vector<MeshBox*>& meshBoxes,
	int sideIndex);

void extractUniqueMeshBoxes(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	const Mesh& parentMesh,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	int sideIndex);

void clearMeshBoxChanges(MeshBox& meshbox);

void clearMeshBoxChildren(MeshBox& meshbox);

void assignParents(mv::vector3<GridCell>& gridCells, int sideIndex);

[[nodiscard]]
size_t numParents(mv::vector3<GridCell>& gridCells, int sideIndex);

void printParents(mv::vector3<GridCell>& gridCells);

void printParents(mv::vector3<GridCell>& gridCells, int sideIndex);

