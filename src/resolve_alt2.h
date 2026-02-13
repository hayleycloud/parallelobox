#pragma once

#include "grid_cut.h"
#include "meshbox.h"

int enumerateConflicts(const mv::vector3<GridCell>& gridCells);

void resolveGaps(
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes,
	const std::vector<Cuboid>& emptyRegions,
	mv::vector3<GridCell>& gridCells,
	Grid& grid);

 