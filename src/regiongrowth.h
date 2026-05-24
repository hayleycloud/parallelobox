#pragma once

#include "config.h"
#include "grid_cut.h"
#include "meshbox.h"
#include "objective.h"
#include <optional>

[[nodiscard]] std::vector<GridCell*> sampleExpand(
	Direction direction, mv::vector3<GridCell>& gridCells, const MeshBox& box);

std::optional<Cuboid> extendRegionIn(
	Direction direction, Cuboid cuboid, const Grid& grid);

std::optional<Cuboid> safeExpand(
	Direction direction,
	const Cuboid& region,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	std::vector<GridCell*>& newCells);

void grow(
	Direction direction, 
	const Mesh& parent,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells, 
	MeshBox& box);

std::optional<Direction> getBestDirection(
	const std::unordered_map<Direction,double>& costs);

void regionGrowth(
	const Config& config,
	const Mesh& parent,
	std::vector<std::unique_ptr<MeshBox>>& sourceBoxes, 
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& cache);

void sampleCells(
	mv::vector3<GridCell>& gridCells, 
	const Cuboid& cuboid,
	std::vector<GridCell*>& samples);

 
