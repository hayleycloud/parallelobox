#pragma once

#include "grid_cut.h"
#include "meshbox.h"


void regionGrowth(
	Mesh& parent,
	std::vector<MeshBox*>& sourceBoxes, 
	mv::vector3<GridCell>& gridCells,
	Grid& grid);

