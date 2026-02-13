#pragma once

#include "grid_cut.h"
#include "meshbox.h"

bool enumerateConflicts(
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	mv::vector3<GridCell>& gridCells);

//void resolveConflicts(
//	const Mesh& parent,
//	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
//	mv::vector3<GridCell>& gridCells,
//	Grid& grid);
 