#pragma once

#include "grid_cut.h"
#include "vec.h"
#include "config.h"
#include "multivec.h"
#include "meshbox.h"
#include "objective.h"

/*bool getDiscreteEmptyRegions(
	const Config& config,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	int numAvailablePrinters,
	std::vector<Cuboid>& emptyRegions);

int getDiscreteEmptyRegions(
	const Config& config,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	std::vector<Cuboid>& emptyRegions);*/

bool fillEmptyBoundarySpaces(
	const Config& config,
	const Mesh& mesh,
	std::vector<std::unique_ptr<MeshBox>>& outMeshBoxes,
	double currentParallelCost,
	const Grid& grid, 
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache);
