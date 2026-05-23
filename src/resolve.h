#pragma once

#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"

bool resolveMeshExcess(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	int numPrintersAvailable,
	double currentParallelScore,
	bool constrainParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells);

