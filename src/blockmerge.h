#pragma once

#include "config.h"
#include "meshbox.h"
#include "grid_cut.h"

void mergeIterate(
	const Config& config, 
	const Mesh& parent,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes);
