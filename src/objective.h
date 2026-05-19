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

std::vector<Direction> calcOrientationsThatFit(const Config& config, const Mesh& mesh);

double printingCost(const Config& config, const Grid& grid, const MeshBox& meshBox);

/*double fitness(const Config& config, const Mesh& mesh);*/

double fullScore(const Config& config, const std::vector<const Mesh*>& printers);

 
