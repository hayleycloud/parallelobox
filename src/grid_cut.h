#pragma once

#include "cgals.h"

class Grid
{
public:
	struct Coord
	{
		size_t x, y, z;
	};

	Grid(
		double width, double height, double thickness,
		size_t xElements, size_t yElements, size_t zElements);

	size_t getWidth() const { return m_Width; }

	size_t getHeight() const { return m_Height; }

	size_t getThickness() const { return m_Thickness; }

	const auto& get() const { return m_Grid; }

	const K::Iso_cuboid_3& get(size_t x, size_t y, size_t z) const;

	const K::Iso_cuboid_3& get(const Coord& coord) const;

	void clipVolume(Mesh& mesh, std::vector<Mesh>& outMeshes) const;

	void clipVolume(Mesh& mesh, size_t x, size_t y, size_t z) const;

	void clipVolume(Mesh& mesh, const Coord& coord) const;

	void clipSurface(Mesh& mesh, std::vector<Mesh>& outMeshes) const;

	void clipSurface(Mesh& mesh, size_t x, size_t y, size_t z) const;

	void clipSurface(Mesh& mesh, const Coord& coord) const;

private:
	const size_t m_Width, m_Height, m_Thickness;
	const size_t m_ZStride, m_YStride;
	std::vector<K::Iso_cuboid_3> m_Grid;

	size_t getOffset(size_t x, size_t y, size_t z) const;

	void clip(Mesh& mesh, std::vector<Mesh>& outMeshes, bool fill) const;

	void clip(Mesh& mesh, size_t x, size_t y, size_t z, bool fill) const;
};

