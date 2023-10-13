#pragma once

#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid);

