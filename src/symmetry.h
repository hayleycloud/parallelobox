#pragma once

#include "cgals.h"

struct LLNormal
{
	double u, v;
	double theta, phi;
	K::Vector_3 n;

	LLNormal(double u, double v);

	double len() const;

	double angleBetween(const LLNormal& other) const;
};

typedef std::pair<K::Point_3, K::Vector_3> HardPlane;
typedef std::pair<LLNormal,double> NormalFitnessPair;

void findSymmetries(const Mesh& mesh, std::vector<HardPlane>* planes);

bool symmetrySplit(const Mesh& mesh, Mesh* outLeft, Mesh* outRight);

 