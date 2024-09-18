#pragma once

#include "config.h"
#include "cgals.h"

int calcMinNumPrinters(const Config& config, const Mesh& mesh);

double overhangCost(
	const Config& config, 
	const Mesh& mesh, 
	const MeshNormalsMap& fnormals,
	const K::Vector_3& floor);

double printingCost(const Config& config, const Mesh& mesh);

bool fitsVolume(
	const Config& config, 
	const std::vector<Direction>& upDirs, 
	const Mesh& mesh,
	std::vector<Direction>& allowedUpDirs);

/*double fitness(const Config& config, const Mesh& mesh);*/

double fullScore(const Config& config, const std::vector<const Mesh*>& printers);

