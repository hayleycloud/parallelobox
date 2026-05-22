#pragma once

#include "config.h"
#include "cgals.h"
#include "grid_cut.h"
#include "meshbox.h"


int calcMinNumPrinters(const Config& config, const Mesh& mesh);

double overhangCost(
	const Config& config, 
	const Mesh& mesh, 
	const MeshNormalsMap& fnormals,
	const K::Vector_3& up,
	const K::Vector_3& floor);

double printingCost(const Config& config, const Mesh& mesh);

bool fitsVolume(const Config& config, const Mesh& mesh);

bool fitsVolume(const Config& config, double width, double height, double depth);

std::vector<Direction> calcOrientationsThatFit(const Config& config, const Mesh& mesh);

double printingCost(
	const Config& config,
	const Grid& grid, 
	const MeshBox& meshBox);
	

struct PrintingCostCacheData
{
	Cuboid lastDims;
	double printCost;

	PrintingCostCacheData(const Cuboid& dims, double printCost) 
	: lastDims(dims), printCost(printCost) {};
};

typedef std::unordered_map<const MeshBox*,PrintingCostCacheData> PrintingCostCache;

void pruneCache(
	const std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	PrintingCostCache& cache);

void pruneCache(
	const std::vector<const MeshBox*>& meshBoxPtrs,
	PrintingCostCache& cache);
	

double printingCost(
	const Config& config,
	const Grid& grid, 
	const MeshBox& meshBox,
	PrintingCostCache& cache);

double parallelPrintingCost(
	const Config& config, 
	const Grid& grid, 
	const std::vector<const MeshBox*>& printers,
	PrintingCostCache& cache);

 
