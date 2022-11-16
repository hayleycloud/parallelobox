#pragma once

#include "cgals.h"

class Grid
{
public:
	Grid(
		double width, double height, double thickness,
		size_t xElements, size_t yElements, size_t zElements);

	const auto& get() const { return m_Grid; }

	const K::Iso_cuboid_3& get(size_t x, size_t y, size_t z) const;

	void clip(Mesh& mesh, std::vector<Mesh>& outMeshes) const;

	void clip(Mesh& mesh, size_t x, size_t y, size_t z) const;


private:
	const size_t m_Width, m_Height, m_Thickness;
	const size_t m_ZStride, m_YStride;
	std::vector<K::Iso_cuboid_3> m_Grid;

	size_t getOffset(size_t x, size_t y, size_t z) const;
};

