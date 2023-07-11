#pragma once

#include "config.h"
#include "meshbox.h"

void mergeIterate(
	const Config& config, 
	mv::vector3<GridCell>& gridCells, 
	std::list<MeshBox>& meshBoxes);
