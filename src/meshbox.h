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
	std::list<MeshBox*> parents;

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

	MeshBox(const Cuboid& dims) : dims(dims) {}

	MeshBox(
		const Mesh& mesh, 
		const Cuboid& dims) : mesh(mesh), dims(dims) {}

	MeshBox(
		const Mesh& mesh,
		const Cuboid& dims,
		const std::vector<GridCell*>& children)
		: mesh(mesh), dims(dims), children(children.begin(), children.end()) {}
};

void getSurfaceBoxes(
	const Mesh& mesh, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	int numPrinters);

Cuboid getDimsFrom(const std::vector<const GridCell*>& children);

Cuboid getDimsFrom(std::vector<GridCell*>& children);

std::vector<GridCell*> sampleCells(
	mv::vector3<GridCell>& gridCells,
	const Vector3D& a, const Vector3D& b);

MeshErrorSet clipFromMesh(const Grid& grid, const Mesh& mesh, const Cuboid& dims, Mesh& out);

MeshErrorSet clipFromMesh(const Grid& grid, const Mesh& mesh, MeshBox& child);

void assignNormals(Mesh& mesh);

void assignNormals(MeshBox& meshBox);

void recomputeNormals(MeshBox& meshBox);

void resetGridCells(mv::vector3<GridCell>& gridCells);

void getFloorVectorsFrom(
	const Grid& grid,
	const MeshBox& meshBox, 
	const std::vector<Direction>& upDirections,
	std::vector<K::Vector_3>& floors);

std::vector<MeshBox*> getNeighbours(
	const Grid& grid,
	const MeshBox& box,
	mv::vector3<GridCell>& gridCells);

bool mergeSoft(MeshBox& a, MeshBox& b);

bool mergeSoft(MeshBox& a, MeshBox& b, MeshBox& c);

bool mergeSoft(MeshBox& a, MeshBox& b, Cuboid& outDims);

bool mergeSoft(MeshBox& a, MeshBox& b, Mesh& outMesh, Cuboid& outDims);

void transferChildren(MeshBox& into, MeshBox& from);

