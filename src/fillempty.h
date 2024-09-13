#pragma once

#include "grid_cut.h"
#include "vec.h"
#include "multivec.h"
#include "meshbox.h"

bool getDiscreteEmptyRegions(
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	int numAvailablePrinters,
	std::vector<Cuboid>& emptyRegions);
