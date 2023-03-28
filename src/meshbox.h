#pragma once

#include "cgals.h"
#include "grid_cut.h"
#include "multivec.h"

struct MeshBox
{
	Mesh mesh;
	K::Iso_cuboid_3 dims;

	enum class ContentType {
		Internal,
		Boundary,
		Empty
	} type;
};

mv::vector3<MeshBox> getSurfaceBoxes(Mesh& mesh, const Grid& grid);

