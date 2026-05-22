#pragma once

#include "cgals.h"
#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "objective.h"

size_t idOf(const MeshBox* meshBox, const std::vector<const MeshBox*>& meshBoxes);

bool mergeEmptyRegions(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<const MeshBox*>& meshBoxPtrs,
	double currentParallelScore,
	int numPrintersAvailable,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache,
	bool constrainParallelScore);

