#pragma once

#include "cgals.h"
#include "config.h"

void bounds(const Mesh& mesh, K::Point_3& min, K::Point_3& max);

void boundsLocal(const Mesh& mesh, K::Point_3& min, K::Point_3& max);

double overhangArea(
	const Config& config, 
	const Mesh& mesh, 
	const MeshNormalsMap& fnormals,
	const K::Vector_3& up,
	const K::Vector_3& floor);

void recenter(Mesh& mesh);

void alignMeshToGrid(Mesh& mesh);

 
