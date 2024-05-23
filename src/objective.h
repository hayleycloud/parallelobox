#pragma once

#include "config.h"
#include "cgals.h"

double overhangCost(
	const Config& config, 
	const Mesh& mesh, 
	const auto& fnormals,
	const K::Vector_3& floor);

double printingCost(const Config& config, const Mesh& mesh);

bool fitsVolume(const Config& config, const Mesh& mesh);

double fitness(const Config& config, const Mesh& mesh);

double fullScore(const Config& config, const std::vector<const Mesh*>& printers);

